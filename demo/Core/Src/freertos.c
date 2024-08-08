/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pcf8574.h"
#include <stdio.h>
#include <stdbool.h>
#include "usart.h"
#include "lwip/sockets.h"
#include "lwip/netif.h"
#include "lwip/ip_addr.h"
#include <string.h>
#include "lwip/sockets.h"
#include "ethernet.h"
//#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
bool sendflag = false;

bool arpReplyflag = false;

bool testflag = false;

uint8_t rxbuf[17] = {0};
uint8_t net_init_flag;

extern ETH_HandleTypeDef heth;
extern ETH_TxPacketConfig TxConfig;
extern osSemaphoreId TxPktSemaphore;
extern osSemaphoreId RxPktSemaphore;

//mutex for arp interaction
SemaphoreHandle_t  mutex;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId dmaReceiveTaskHandle;
osThreadId receiveEthTaskHandle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId transmitEthTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void StartTask03(void const * argument);

void StartTask04(void const * argument);

//  ARP请求结构
typedef  struct  {
      uint8_t  hardware_type[2];  //  硬件类型，以太网为1
      uint8_t  protocol_type[2];  //  协议类型，IPv4为0x0800
      uint8_t  hardware_size;      //  硬件地址长度，以太网为6
      uint8_t  protocol_size;      //  协议地址长度，IPv4为4
      uint16_t  operation;          //  操作码，1表示ARP请求
      uint8_t  sender_mac[6];      //  发送者MAC地址
      uint8_t  sender_ip[4];       //  发送者IP地址
      uint8_t  target_mac[6];      //  目标MAC地址（ARP请求时为全0）
      uint8_t  target_ip[4];       //  目标IP地址
}  ARP_Packet;

uint8_t  my_mac_address[]  =  {0x00,  0x80,  0xE1,  0x00,  0x00,  0x00};
uint8_t  my_ip_address[]  =  {192,  168,  1,  10};
uint8_t  target_ip_address[]  =  {192,  168,  1,  5};  //  目标IP地址
uint8_t  target_mac_address[]  =  {0x08,  0x00,  0x27,  0xA0,  0x12,  0x40};

//  构建ARP请求
void  build_arp_request(ARP_Packet  *arp_packet,  uint8_t  *sender_mac,  uint8_t  *sender_ip,  uint8_t  *target_ip)  {
      //  硬件类型和协议类型
      arp_packet->hardware_type[0]  =  0x00;
      arp_packet->hardware_type[1]  =  0x01;
      arp_packet->protocol_type[0]  =  0x08;
      arp_packet->protocol_type[1]  =  0x00;
      //  硬件地址和协议地址长度
      arp_packet->hardware_size  =  0x06;
      arp_packet->protocol_size  =  0x04;
      //  操作码，ARP请求
      arp_packet->operation  =  0x0100;   //0001 ——》网络序0100
      //  发送者MAC地址和IP地址
      memcpy(arp_packet->sender_mac,  sender_mac,  6);
      memcpy(arp_packet->sender_ip,  sender_ip,  4);
      //  目标MAC地址，ARP请求时为全0
      memset(arp_packet->target_mac,  0,  6);
      //  目标IP地址
      memcpy(arp_packet->target_ip,  target_ip,  4);
}

//  构建ARP reply
void  build_arp_reply(ARP_Packet  *arp_packet,  uint8_t  *sender_mac, uint8_t *target_mac,  uint8_t  *sender_ip,  uint8_t  *target_ip)  {
      //  硬件类型和协议类型
      arp_packet->hardware_type[0]  =  0x00;
      arp_packet->hardware_type[1]  =  0x01;
      arp_packet->protocol_type[0]  =  0x08;
      arp_packet->protocol_type[1]  =  0x00;
      //  硬件地址和协议地址长度
      arp_packet->hardware_size  =  0x06;
      arp_packet->protocol_size  =  0x04;
      //  操作码，ARP请求
      arp_packet->operation  =  0x0200;
      //  发送者MAC地址和IP地址
      memcpy(arp_packet->sender_mac,  sender_mac,  6);
      memcpy(arp_packet->sender_ip,  sender_ip,  4);
      //  目标MAC地址，ARP请求时为全0
      memcpy(arp_packet->target_mac,  target_mac,  6);
      //  目标IP地址
      memcpy(arp_packet->target_ip,  target_ip,  4);
}

err_t transmit_frame(struct pbuf *p){
    uint32_t i = 0U;
    struct pbuf *q = NULL;
    err_t errval = ERR_OK;
    ETH_BufferTypeDef Txbuffer[ETH_TX_DESC_CNT] = {0};

    memset(Txbuffer, 0 , ETH_TX_DESC_CNT*sizeof(ETH_BufferTypeDef));

    for(q = p; q != NULL; q = q->next)
    {
      if(i >= ETH_TX_DESC_CNT)
        return ERR_IF;

      Txbuffer[i].buffer = q->payload;
      Txbuffer[i].len = q->len;

      if(i>0)
      {
        Txbuffer[i-1].next = &Txbuffer[i];
      }

      if(q->next == NULL)
      {
        Txbuffer[i].next = NULL;
      }

      i++;
    }

    TxConfig.Length = p->tot_len;
    TxConfig.TxBuffer = Txbuffer;
    TxConfig.pData = p;

    pbuf_ref(p);

    if (HAL_ETH_Transmit_IT(&heth, &TxConfig) == HAL_OK) {
      while(osSemaphoreWait(TxPktSemaphore, portMAX_DELAY)!=osOK)

      {
      }

      //must release
      HAL_ETH_ReleaseTxPacket(&heth);
    } else {
      pbuf_free(p);
    }

    return errval;
}

/**
 * @brief rewrite HAL_ETH_ReleaseTxPacket when tx dont use pbuf
 * 
 * @param heth 
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef HAL_ETH_ReleaseTxPacket_rewr(ETH_HandleTypeDef *heth)
{
  ETH_TxDescListTypeDef *dmatxdesclist = &heth->TxDescList;
  uint32_t numOfBuf =  dmatxdesclist->BuffersInUse;
  uint32_t idx =       dmatxdesclist->releaseIndex;
  uint8_t pktTxStatus = 1U;
  uint8_t pktInUse;

  /* Loop through buffers in use.  */
  while ((numOfBuf != 0U) && (pktTxStatus != 0U))
  {
    pktInUse = 1U;
    numOfBuf--;
    /* If no packet, just examine the next packet.  */
    if (dmatxdesclist->PacketAddress[idx] == NULL)
    {
      /* No packet in use, skip to next.  */
      idx = (idx + 1U) & (ETH_TX_DESC_CNT - 1U);
      pktInUse = 0U;
    }

    if (pktInUse != 0U)
    {
      /* Determine if the packet has been transmitted.  */
      if ((heth->Init.TxDesc[idx].DESC0 & ETH_DMATXDESC_OWN) == 0U)
      {
        /* Release the packet.  */
        //dont need anymore
        //HAL_ETH_TxFreeCallback(dmatxdesclist->PacketAddress[idx]);

        /* Clear the entry in the in-use array.  */
        dmatxdesclist->PacketAddress[idx] = NULL;

        /* Update the transmit relesae index and number of buffers in use.  */
        idx = (idx + 1U) & (ETH_TX_DESC_CNT - 1U);
        dmatxdesclist->BuffersInUse = numOfBuf;
        dmatxdesclist->releaseIndex = idx;
      }
      else
      {
        /* Get out of the loop!  */
        pktTxStatus = 0U;
      }
    }
  }
  return HAL_OK;
}

//arp req frame len 42
bool transmit_frame_nopbuf(void *p, uint32_t len){
    ETH_BufferTypeDef Txbuffer[ETH_TX_DESC_CNT] = {0};

    memset(Txbuffer, 0 , ETH_TX_DESC_CNT*sizeof(ETH_BufferTypeDef));

    Txbuffer[0].buffer = p;
    Txbuffer[0].len = len;
    Txbuffer[0].next = NULL;
    TxConfig.Length = len;
    TxConfig.TxBuffer = Txbuffer;
    TxConfig.pData = p;   //是否保存

    if (HAL_ETH_Transmit_IT(&heth, &TxConfig) == HAL_OK) {
      while(osSemaphoreWait(TxPktSemaphore, portMAX_DELAY)!=osOK)
      {
      }

      //printf("one frame has been transmited\r\n");

      vPortFree(p);

      // must release because ETH_TX_DESC_CNT is 4 according to configuration
      HAL_ETH_ReleaseTxPacket_rewr(&heth);
      return true;
    } else {
      printf("transmit error\r\n");
      return false;
    }
}

//arp req frame len 42
bool transmit_frame_nopbuf_nofree(void *p, uint32_t len){
    ETH_BufferTypeDef Txbuffer[ETH_TX_DESC_CNT] = {0};

    memset(Txbuffer, 0 , ETH_TX_DESC_CNT*sizeof(ETH_BufferTypeDef));

    Txbuffer[0].buffer = p;
    Txbuffer[0].len = len;
    Txbuffer[0].next = NULL;
    TxConfig.Length = len;
    TxConfig.TxBuffer = Txbuffer;
    TxConfig.pData = p;   //是否保存

    if (HAL_ETH_Transmit_IT(&heth, &TxConfig) == HAL_OK) {
      while(osSemaphoreWait(TxPktSemaphore, portMAX_DELAY)!=osOK)
      {
      }

      //printf("one frame has been transmited\r\n");

      //vPortFree(p);

      // must release because ETH_TX_DESC_CNT is 4 according to configuration
      HAL_ETH_ReleaseTxPacket_rewr(&heth);
      return true;
    } else {
      printf("transmit error\r\n");
      return false;
    }
}

uint8_t* buildframe(){
  //ARP数据包
  ARP_Packet  arp_packet;
  build_arp_request(&arp_packet,  my_mac_address,  my_ip_address,  target_ip_address);

  //以太网帧头
  struct eth_addr dst_mac = {0xFF,  0xFF,  0xFF,  0xFF,  0xFF,  0xFF};  //  广播地址
  struct eth_addr src_mac = {arp_packet.sender_mac[0],  arp_packet.sender_mac[1],  arp_packet.sender_mac[2],
                              arp_packet.sender_mac[3],  arp_packet.sender_mac[4],  arp_packet.sender_mac[5]};
  struct eth_hdr ethhdr;
  ethhdr.type = 0x0608;    //0806 ——》 网络序 0608
  SMEMCPY(&ethhdr.dest, &dst_mac, ETH_HWADDR_LEN);
  SMEMCPY(&ethhdr.src,  &src_mac, ETH_HWADDR_LEN);

  uint8_t *frame = (uint8_t*)pvPortMalloc(sizeof(struct eth_hdr) + sizeof(ARP_Packet));

  memcpy(frame,  &ethhdr,  sizeof(struct eth_hdr));
  memcpy(frame  +  sizeof(struct eth_hdr),  &arp_packet,  sizeof(ARP_Packet));

  //vPortFree();
  return frame;
}

uint8_t* buildframe_NULL(){
  //ARP数据包
  ARP_Packet  arp_packet;
  build_arp_request(&arp_packet,  my_mac_address,  my_ip_address,  target_ip_address);

  //以太网帧头
  struct eth_addr dst_mac = {0xFF,  0xFF,  0xFF,  0xFF,  0xFF,  0xFF};  //  广播地址
  struct eth_addr src_mac = {arp_packet.sender_mac[0],  arp_packet.sender_mac[1],  arp_packet.sender_mac[2],
                              arp_packet.sender_mac[3],  arp_packet.sender_mac[4],  arp_packet.sender_mac[5]};
  struct eth_hdr ethhdr;
  ethhdr.type = 0x0000;    //0806 ——》 网络序 0608
  SMEMCPY(&ethhdr.dest, &dst_mac, ETH_HWADDR_LEN);
  SMEMCPY(&ethhdr.src,  &src_mac, ETH_HWADDR_LEN);

  uint8_t *frame = (uint8_t*)pvPortMalloc(sizeof(struct eth_hdr) + sizeof(ARP_Packet));

  memcpy(frame,  &ethhdr,  sizeof(struct eth_hdr));
  memcpy(frame  +  sizeof(struct eth_hdr),  &arp_packet,  sizeof(ARP_Packet));

  //vPortFree();
  return frame;
}

uint8_t* buildframe_reply(){
  //ARP数据包
  ARP_Packet  arp_packet;
  build_arp_reply(&arp_packet,  my_mac_address, target_mac_address,  my_ip_address,  target_ip_address);

  //以太网帧头
  struct eth_addr dst_mac = {arp_packet.target_mac[0],  arp_packet.target_mac[1],  arp_packet.target_mac[2], 
                              arp_packet.target_mac[3],  arp_packet.target_mac[4],  arp_packet.target_mac[5]};
  // struct eth_addr dst_mac = {target_mac_address[0],  target_mac_address[1],  target_mac_address[2], 
  //                             target_mac_address[3],  target_mac_address[4],  target_mac_address[5]};
  struct eth_addr src_mac = {arp_packet.sender_mac[0],  arp_packet.sender_mac[1],  arp_packet.sender_mac[2],
                              arp_packet.sender_mac[3],  arp_packet.sender_mac[4],  arp_packet.sender_mac[5]};
  struct eth_hdr ethhdr;
  ethhdr.type = 0x0608;    //0806 ——》 网络序 0608
  SMEMCPY(&ethhdr.dest, &dst_mac, ETH_HWADDR_LEN);
  SMEMCPY(&ethhdr.src,  &src_mac, ETH_HWADDR_LEN);

  uint8_t *frame = (uint8_t*)pvPortMalloc(sizeof(struct eth_hdr) + sizeof(ARP_Packet));

  memcpy(frame,  &ethhdr,  sizeof(struct eth_hdr));
  memcpy(frame  +  sizeof(struct eth_hdr),  &arp_packet,  sizeof(ARP_Packet));

  //vPortFree();
  return frame;
}

static struct pbuf * receive_frame()
{
  struct pbuf *p = NULL;

  // if(RxAllocStatus == RX_ALLOC_OK)
  // {
    HAL_ETH_ReadData(&heth, (void **)&p);
  // }

  return p;
}

bool compare_mac(uint8_t *a, uint8_t *b){
  bool same = true;
  int i = 0;
  while(i<ETH_HWADDR_LEN){
    //printf("0000000  %02X %02X\r\n", a[i], b[i]);
    if(a[i] != b[i]){
      same = false;
      break;
    }
    ++i;
  }
  return same;
}

bool compare_ip(uint8_t *a, uint8_t *b){
  bool same = true;
  int i = 0;
  while(i<4){
    //printf("0000000  %u %u\r\n", a[i], b[i]);
    if(a[i] != b[i]){
      same = false;
      break;
    }
    ++i;
  }
  return same;
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  // osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 2048);
  // defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of transmitEthTask */
  // osThreadDef(transmitEthTask, StartTask02, osPriorityRealtime, 0, 2048);
  // transmitEthTaskHandle = osThreadCreate(osThread(transmitEthTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  // osThreadDef(dmaReceiveTask, StartTask03, osPriorityIdle, 0, 2048);
  // dmaReceiveTaskHandle = osThreadCreate(osThread(dmaReceiveTask), NULL);
  
  osThreadDef(receiveEthTask, StartTask04, osPriorityRealtime, 0, 2048);
  receiveEthTaskHandle = osThreadCreate(osThread(receiveEthTask), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */

  net_init_flag = 1;

  mutex = xSemaphoreCreateMutex();
  if(mutex == NULL){
    printf("arp mutex create failed!!!\r\n");
  }

  for(;;)
  {
	  HAL_GPIO_WritePin(GPIOB, LED0_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET);
	  osDelay(500);
	  HAL_GPIO_WritePin(GPIOB, LED0_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET);
	  osDelay(500);

  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the transmitEthTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */

    MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */

  net_init_flag = 1;

  mutex = xSemaphoreCreateMutex();
  if(mutex == NULL){
    printf("arp mutex create failed!!!\r\n");
  }
  
  //等待网络初始化
   while (!net_init_flag)
   {
     printf("not init\r\n");
   }
  osDelay(500);

  //building once
  uint8_t *f1 = buildframe();
  uint8_t *f3 = buildframe_NULL();
  /* Infinite loop */
  for(;;)
  {
    /*send arp request*/
    if(sendflag){
      printf("build one\r\n");
      //uint8_t *f1 = buildframe();
      printf("will transmit one\r\n");
      transmit_frame_nopbuf(f1, 42);
      sendflag = false;
    }

    /*send arp reply*/
    if(arpReplyflag){
      printf("build one\r\n");
      uint8_t *f2 = buildframe_reply();
      printf("will transmit one\r\n");
      transmit_frame_nopbuf(f2, 42);
      arpReplyflag = false;
    }

    if(testflag){
      int i = 0;
      while (i<10000)
      {
        // printf("%d\r\n", i);
        transmit_frame_nopbuf_nofree(f1, 42);
        ++i;
      }
      
      transmit_frame_nopbuf_nofree(f3, 42);
      //testflag = false;
      
    }

    /**/
  }

  /* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartTask03(void const * argument)
{
  static bool flag = true;
  rxbuf[16] = '\0';
  /**
   * @brief DMA接收
   * 
   */
  HAL_UART_Receive_DMA(&huart1, rxbuf, sizeof(rxbuf)-1);

  osDelay(50);

  printf("reveive dma state: %d\r\n", hdma_usart1_rx.State);

  while(1){
    if(hdma_usart1_rx.State == HAL_DMA_STATE_READY)
		{
      printf("reveive string: %s\r\n", rxbuf);
      HAL_UART_Receive_DMA(&huart1, rxbuf, sizeof(rxbuf)-1);
    }
  }

  /**
   * @brief 中断接收
   * 
   */
  // HAL_UART_Receive_IT(&huart1, rxbuf, sizeof(rxbuf));

  // osDelay(50);

  // printf("reveive dma state: %x\r\n", huart1.RxState);

  // while(1){
  //   if(huart1.RxState == HAL_UART_STATE_READY)
	// 	{
  //     printf("reveive string: %s\r\n", rxbuf);
  //     while(huart1.RxState != HAL_UART_STATE_BUSY_RX){
  //       HAL_UART_Receive_IT(&huart1, rxbuf, sizeof(rxbuf));
  //     }
  //     // not work
  //     //while(HAL_UART_Receive_IT(&huart1, rxbuf, sizeof(rxbuf)) != HAL_OK) ;
  //   }
    
  // }
  
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    //
}

void StartTask04(void const * argument)
{
  //等待网络初始化
MX_LWIP_Init();
net_init_flag = 1;

mutex = xSemaphoreCreateMutex();
  if(mutex == NULL){
    printf("arp mutex create failed!!!\r\n");
  }

  while (!net_init_flag)
  {
    printf("not init\r\n");
  }
  osDelay(500);

  /*接收ARP请求*/
  struct pbuf *p = NULL;
  struct eth_hdr *ethhdr;
  u16_t type;

  int total = 0;
  for( ;; )
  {
    if (osSemaphoreWait(RxPktSemaphore, portMAX_DELAY) == osOK)
    {
      do
      {
        p = receive_frame();
        if (p != NULL)
        {
          if(p->len == p->tot_len){
            //printf("receive one frame, len == tot_len, len without FCS(4): %u\r\n", p->len);
            
            //数据帧打印
            // uint8_t *tmp = (uint8_t *)p->payload;
            // for(int i=0;i<p->len;++i){
            //   printf("%02X  ", tmp[i]);
            // }
            // printf("\r\n");

          }
          else printf("len != tot_len\r\n");

          //数据帧处理
          ethhdr = (struct eth_hdr *)p->payload;

          type = ethhdr->type;

          switch (type) {
            /* IP packet? */
            case PP_HTONS(ETHTYPE_IP):
              break;

            /* ARP packet*/
            case PP_HTONS(ETHTYPE_ARP):
              // printf("src MAC: %02X %02X %02X %02X %02X %02X, type: %04X\r\n", (uint8_t)ethhdr->src.addr[0], (uint8_t)ethhdr->src.addr[1], (uint8_t)ethhdr->src.addr[2],
              //                               (uint8_t)ethhdr->src.addr[3], (uint8_t)ethhdr->src.addr[4], (uint8_t)ethhdr->src.addr[5],
              //                               lwip_htons(ethhdr->type));

              ARP_Packet * arpdata = (ARP_Packet *)((uint8_t*)ethhdr + sizeof(struct eth_hdr));
              uint16_t op = ntohs(arpdata->operation);
              
              if(op == 0x0001){
                if(compare_mac( (ethhdr->src).addr, target_mac_address)){
                  //printf("same\r\n");
                  total++;
                }
                // printf("this is an ARP request\r\n");
                // if(compare_ip(arpdata->sender_ip, target_ip_address)){
                //   printf("same\r\n");
                //   arpReplyflag = true;
                // }
              }
              else if(op == 0x0002){
                //printf("this is an ARP reply\r\n");
              }
              else if(op == 0x0000){
                //是否再判断下MAC？
                if(total < 10001)
                  printf("recv %d packets, %6.2f%% loss \r\n", total, (10000.0f - total) / 100.0f);
                total = 0;
              }
              else{
                //printf("other arp type\r\n");
              }
              
              break;

            default:
              break;
          }

          pbuf_free(p);
        }
      } while(p!=NULL);
    }
  }
}


/* USER CODE END Application */


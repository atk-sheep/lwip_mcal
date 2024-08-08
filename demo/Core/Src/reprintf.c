/*
 * reprintf.c
 *
 *  Created on: Mar 31, 2024
 *      Author: ECA1WX
 */

#include <stdio.h>

#include "usart.h"
#include "stm32f7xx.h"
#include "stm32f7xx_hal_dma.h"

#include "RePrintf.h"

#ifdef __GNUC__

/**
 * @brief 收发同时的情况下，HAL库接口HAL_UART_GetState既判断接收也判断发送状态，返回busy状态，这里只判断发送状态
 * 
 * @param huart 
 * @return HAL_UART_StateTypeDef 
 */
HAL_UART_StateTypeDef UART_GetTXState(const UART_HandleTypeDef *huart)
{
  uint32_t temp1;
  temp1 = huart->gState;

  return (HAL_UART_StateTypeDef)(temp1);
}

int _write(int file, char *ptr, int len){
	//中断发送
	//HAL_UART_Transmit_IT(&huart1, ptr, len);		//STM32 HAL库中断方式发送数据，需开启NVIC中断（提前配置好发送和接收引脚PA9，PA10）

	//阻塞发送
	//HAL_UART_Transmit(&huart1,(uint8_t*)ptr, len, 0xFFFF);

	//DMA发送
	//SCB_CleanDCache_by_Addr(ptr, len);	//更新cache到内存（如果未开启透写）
	HAL_UART_Transmit_DMA(&huart1, ptr, len);	//DMA发送
	
	while (UART_GetTXState(&huart1) != HAL_UART_STATE_READY)
	{
	}

	// while (1)
	// {
	// 	uint32_t isrflags = READ_REG((huart1.Instance)->ISR);
	// 	if((isrflags & USART_ISR_TC) != 0U){
	// 		break;
	// 	}
	// 	/* code */
	// }

	//not work
	// while (HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_READY)
	// {
	// }
	return len;
}

#else

int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

#endif

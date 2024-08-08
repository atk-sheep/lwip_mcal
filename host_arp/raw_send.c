#include <stdio.h>      
#include <stdlib.h>  
#include <string.h>
#include <unistd.h>
 
#include <sys/socket.h>     
#include <sys/ioctl.h>      
#include <arpa/inet.h>      
 
#include <linux/if.h>
#include <linux/if_packet.h> 
#include <linux/if_ether.h>     
#include <linux/if_arp.h>

#include <pthread.h>
#include <stdbool.h>
 
#define IPV4_LENGTH 4
#define Dev "enp0s3"     //网卡名
#define buffer_len 60   //ARP请求包大小为60B,,抓包时会抓到一些42B的包，这是抓包软件没有显示18B的Padding字段，Padding全0填充在包的末尾
unsigned char sender_ip[4] = {192,168,1,5};    //ARP请求的源IP
unsigned char target_ip[4] = {192,168,1,10};     //ARP请求的目标IP
/*ARP包结构*/
/*字段顺序不可更改，发包时是直接将buffer发出*/
struct arp_head
{
    unsigned short hardware_type;   //硬件类型#1:Ethernet
	unsigned short protocol_type;   //协议类型#0x0800:IPv4
    unsigned char hardware_size;    //MAC地址长度#6
    unsigned char protocol_size;    //IP地址长度#4
    unsigned short opcode;          //ARP类型#1:request;2:reply
    unsigned char sender_mac[ETH_ALEN];    //源MAC地址
    unsigned char sender_ip[IPV4_LENGTH];  //源IP地址
    unsigned char target_mac[ETH_ALEN];    //目标MAC地址
    unsigned char target_ip[IPV4_LENGTH];  //目标IP地址
};

bool flag_sendarp = false;
bool flag_exit = false;
void *getInput(void *arg){
    char input[5] = {};
    
    while(1){

        printf("enter a string: \n");

        fgets(input, sizeof(input), stdin);

        input[1] = 0;

        if(strcmp(input, "q") == 0){
            printf("will exit\n");
            flag_exit = true;
        }
        else if(strcmp(input, "s") == 0){
            printf("will sendarp\n");
            flag_sendarp = true;
        }
        else{
            printf("not valid\n");
        }

        sleep(1);

        if(flag_exit) break;
    }
}
 
int main()
{
    pthread_t tid = -1;

    int ret = pthread_create(&tid, NULL, getInput, NULL);

    if(ret != 0){
        fprintf(stderr, "pthread_create error: %s\n", strerror(ret));
    }
    else{
        fprintf(stdout, "pthread_create success\n");
    }

    //创建buffer
    unsigned char buffer[buffer_len];  
    memset(buffer, 0, buffer_len);
    //创建以太网头部指针,指向buffer
    struct ethhdr *eth_req = (struct ethhdr*)buffer;
    //创建ARP包指针，指向buffer的后46字节，因为以太网头包含：2*6B(MAC地址)+2B(协议地址)=14B
    struct arp_head *arp_req = (struct arp_head*)(buffer+14);
    //创建sockaddr_ll结构地址
    struct sockaddr_ll sock_addr;
    //创建socket
    /***int socket(int __domain, int __type, int __protocol)
     *** __domain
     * PF_PACKET指示为二层协议簇
     * 使用AF_PACKET也可,socket.h中有#define AF_PACKET PF_PACKET
     *** __type
     * 使用PF_PACKET的后，__type只能选择SOCK_RAW或者SOCK_DGRAM
     * 其中SOCK_RAW可以自己构造帧头，SOCK_DGRAM不行
     * 帧头使用sockaddr_ll结构体构建，这个结构体在if_packet.h中
     * 有一些资料这里选择的是SOCK_PACKET，这个类型目前已经被建议弃用
     *** __protocol
     * ETH_P_ARP意味着我们仅仅接受ARP类型
     * 如果是ETH_P_ALL就意味着我们接受所有类型帧
     * 更多选项参看if_ether.h中定义
     */
    int sock_fd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ARP));
    if(sock_fd == -1){
        perror("socket()");
        exit(-1);
    }
    /**获取网卡等需要的信息
     * ifreq结构体可以用于设置或者获取网卡等相关信息,定义在if.h中
     * 配合ioctl()一起使用
     * ioctl()的具体参数用法和系统实现相关，不是通用的，具体参见ioctls.h
     * 以下获取的信息都会保存在ifreq不同字段之中
     */ 
    struct ifreq ifr;
 
    /*根据网卡设备名获取Index*/
    strcpy(ifr.ifr_name, Dev);
    if(ioctl(sock_fd, SIOCGIFINDEX, &ifr) == -1)
    {
        perror("SIOCGIFINDEX");
        exit(-1);
    }
    int ifindex = ifr.ifr_ifindex;
    printf("网卡索引为:%d\n",ifindex);
 
    /*获取网卡设备MAC地址*/
    if(ioctl(sock_fd, SIOCGIFHWADDR, &ifr) == -1)
    {
        perror("SIOCGIFHWADDR");
        exit(-1);
    }
 
    /*将MAC地址写入所需结构*/
    for(int i=0;i<6;i++)
    {
        //以太网帧的目标MAC，即广播MAC，全1
        eth_req->h_dest[i] = (unsigned char)0xff;
        //ARP请求包目标MAC，全0
        arp_req->target_mac[i] = (unsigned char)0x00;
        //以太网帧源MAC，即本机MAC
        //ifr_hwaddr是sockaddr结构体格式
        eth_req->h_source[i] = (unsigned char)ifr.ifr_hwaddr.sa_data[i];
        //ARP请求包源MAC，即本机MAC
        arp_req->sender_mac[i] = (unsigned char)ifr.ifr_hwaddr.sa_data[i];
        //sockaddr中的MAC，也是本地MAC
        sock_addr.sll_addr[i] = (unsigned char)ifr.ifr_hwaddr.sa_data[i];
    }
    
    /*打印MAC地址*/
    printf("网卡MAC地址: %02X:%02X:%02X:%02X:%02X:%02X\n",
            eth_req->h_source[0],
            eth_req->h_source[1],
            eth_req->h_source[2],
            eth_req->h_source[3],
            eth_req->h_source[4],
            eth_req->h_source[5]);
 
    /*完善sockaddr_ll结构体*/
    sock_addr.sll_family = PF_PACKET;  
    sock_addr.sll_protocol = htons(ETH_P_ARP);
    sock_addr.sll_ifindex = ifindex;
    sock_addr.sll_hatype = htons(ARPHRD_ETHER);
    sock_addr.sll_halen = ETH_ALEN;
 
    /*完善以太网帧头*/
    eth_req->h_proto = htons(ETH_P_ARP);
 
    /*完善ARP包头*/
    arp_req->hardware_type = htons(0x01);
    arp_req->protocol_type = htons(ETH_P_IP);
    arp_req->hardware_size = ETH_ALEN;
    arp_req->protocol_size = IPV4_LENGTH;
    arp_req->opcode = htons(ARPOP_REQUEST);
    memcpy(arp_req->sender_ip,sender_ip,IPV4_LENGTH);
    memcpy(arp_req->target_ip,target_ip,IPV4_LENGTH);

    uint16_t *type = ((uint8_t*)buffer + 20);
    //while sendto
    while(1){
        /*发送ARP请求*/
        if(flag_sendarp){
            int i = 0;

            *type = 0x0100;

            while(i<10000){
                if(sendto(sock_fd, buffer, 60, 0, (struct sockaddr*)&sock_addr, sizeof(sock_addr)) == -1)
                {
                    perror("sendto()");
                    exit(-1);
                }

                //去掉，如果再开wireshark监控流量，很容易卡住
                //usleep(1);

                ++i;
            }
            
            *type = 0x0000;

            if(sendto(sock_fd, buffer, 60, 0, (struct sockaddr*)&sock_addr, sizeof(sock_addr)) == -1)
            {
                perror("sendto()");
                exit(-1);
            }

            //flag_sendarp = false;

            // if(sendto(sock_fd, buffer, 60, 0, (struct sockaddr*)&sock_addr, sizeof(sock_addr)) == -1)
            // {
            //     perror("sendto()");
            //     exit(-1);
            // }
            // // printf("发送ARP请求包:");
            // // for(int i=0;i<60;i++)
            // // {
            // //     if(i%16==0)
            // //         printf("\n\t");
            // //     printf("%02X ",buffer[i]);
            // // }

            // flag_sendarp = false;
        }
        
        sleep(0.5);
    }
    
    close(sock_fd);
    return 0;
}
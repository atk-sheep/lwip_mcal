#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netpacket/packet.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/if.h>
#include <stdbool.h>

#include <linux/if_ether.h>  

#define BUFFER_SIZE 2048

uint8_t  remote_mac_address[]  =  {0x00,  0x80,  0xE1,  0x00,  0x00,  0x00};

bool compare_mac(uint8_t *a, uint8_t *b){
  bool same = true;
  int i = 0;
  while(i<6){
    //printf("0000000  %02X %02X\r\n", a[i], b[i]);
    if(a[i] != b[i]){
      same = false;
      break;
    }
    ++i;
  }
  return same;
}

bool flag_exit = false;

int main() {
    int sockfd, n;
    char buffer[BUFFER_SIZE];
    struct sockaddr_ll sa;
    struct ifreq ifr; // For setting promiscuous mode
    char *interface_name = "enp0s3";

    sockfd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (sockfd == -1) {
        perror("socket");
        exit(EXIT_FAILURE);
    }

    // Enable promiscuous mode
    strncpy(ifr.ifr_name, interface_name, IFNAMSIZ);
    if (ioctl(sockfd, SIOCGIFFLAGS, &ifr) == -1) {
        perror("ioctl SIOCGIFFLAGS");
        close(sockfd);
        exit(EXIT_FAILURE);
    }
    ifr.ifr_flags |= IFF_PROMISC;
    if (ioctl(sockfd, SIOCSIFFLAGS, &ifr) == -1) {
        perror("ioctl SIOCSIFFLAGS");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    memset(&sa, 0, sizeof(struct sockaddr_ll));
    sa.sll_family = AF_PACKET;
    sa.sll_protocol = htons(ETH_P_ALL);
    sa.sll_ifindex = if_nametoindex(interface_name);
    if (bind(sockfd, (struct sockaddr*)&sa, sizeof(struct sockaddr_ll)) == -1) {
        perror("bind");
        close(sockfd);
        exit(EXIT_FAILURE);
    }
    
    printf("Listening on %s in promiscuous mode...\n", interface_name);
    
    int total = 0;
    while (1) {
        //接收所有该dev的帧
        n = recvfrom(sockfd, buffer, BUFFER_SIZE, 0, NULL, NULL);
        if (n == -1) {
            perror("recvfrom");
            break;
        }
        struct ethhdr *frame = (struct ethhdr *)buffer;

        if(frame->h_proto == 0x0000){
            if(total <= 10000){
                printf("recv %d packets, %6.2f%% loss \n", total, (10000.0f - total) / 100.0f);
            }
            total = -1;
        }

        //处理指定源MAC的帧
        //丢包率测试
        if(compare_mac(frame->h_source, remote_mac_address)){
            // printf("Received a frame of length: %d bytes\n", n);
            ++total;
        }
    }
    
    close(sockfd);
    return 0;
}

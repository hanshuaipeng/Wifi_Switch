#ifndef _tcp_H
#define _tcp_H

void ICACHE_FLASH_ATTR WIFIAPInit();
void ICACHE_FLASH_ATTR WIFIServerMode();
void ICACHE_FLASH_ATTR TcpServerListen_PCon(void *arg);
void ICACHE_FLASH_ATTR WIFI_TCP_SendNews(unsigned char *dat,uint16 len);
void ICACHE_FLASH_ATTR dhcps_lease(void);
void ICACHE_FLASH_ATTR WIFI_TCP_SendNews(unsigned char *dat,uint16 len);
void ICACHE_FLASH_ATTR ICACHE_FLASH_ATTR station_server_init(struct ip_addr *local_ip,int port);
#endif


/*
 * udp_server.h
 *
 *  Created on: 2017年7月3日
 *      Author: Administrator
 */

#ifndef _UDP_SERVER_H_
#define _UDP_SERVER_H_

#include "c_types.h"

/* UDP Server 初始化 */
void udp_server_init(uint32 port,struct ip_addr *local_ip,uint8 mode);
void ICACHE_FLASH_ATTR WIFI_UDP_SendNews(unsigned char *dat,uint16 len);

#endif /* _UDP_SERVER_H_ */

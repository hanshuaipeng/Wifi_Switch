/*
 * udp_server.h
 *
 *  Created on: 2017��7��3��
 *      Author: Administrator
 */

#ifndef _UDP_SERVER_H_
#define _UDP_SERVER_H_

#include "c_types.h"

/* UDP Server ��ʼ�� */
void udp_server_init(uint32 port,struct ip_addr *local_ip,uint8 mode);
void ICACHE_FLASH_ATTR WIFI_UDP_SendNews(unsigned char *dat,uint16 len);

#endif /* _UDP_SERVER_H_ */

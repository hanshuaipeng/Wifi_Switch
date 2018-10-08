/*
 * udp_server.c
 *
 *  Created on: 2017年7月3日
 *      Author: Administrator
 */

#include "ets_sys.h"
#include "osapi.h"
#include "user_interface.h"
#include "mem.h"
#include "espconn.h"
#include "user_config.h"
#include "udp_server.h"

#define UDP_BUFF_SIZE		128
static u8 g_udp_buff[UDP_BUFF_SIZE];

struct espconn *sendesp_conn;
extern uint8 tcp_send;

extern uint8 mqtt_buff[200];
extern uint8 dev_sid[15];


extern uint8 pub_flag;
/*
 * 函数: udp_get_peer_conn
 * 说明：获取对端的UDP信息
 */
static u8 ICACHE_FLASH_ATTR
udp_get_peer_conn(void *arg)
{
	struct espconn *pesp_conn = arg;
	remot_info *premot = NULL;

	if (espconn_get_connection_info(pesp_conn, &premot, 0) == ESPCONN_OK){
		pesp_conn->proto.udp->remote_port = premot->remote_port;
		os_memcpy(pesp_conn->proto.udp->remote_ip, premot->remote_ip, 4);
		return 1;
	}
	return 0;
}

/*
 * 函数：udp_print_conn
 * 说明：打印UDP对面连接的信息
 */
static void ICACHE_FLASH_ATTR
udp_print_conn(void *arg)
{
  struct espconn *pesp_conn = arg;

  os_printf("[INFO] udp recv:"IPSTR":%d\r\n",
    IP2STR(pesp_conn->proto.udp->remote_ip), pesp_conn->proto.udp->remote_port);
}

/*
 * 函数：udp_server_sent_cb
 * 说明：发送回调
 */
static void ICACHE_FLASH_ATTR
udp_server_sent_cb(void *arg)
{
	os_printf("UDP发送成功\r\n");
}

/*
 * 函数名:void WIFI_UDP_SendNews(unsigned char *dat)
 * 功能:像UDP服务器发送消息
 */
void ICACHE_FLASH_ATTR WIFI_UDP_SendNews(unsigned char *dat,uint16 len)
{
	espconn_send(sendesp_conn,dat,len);
}
/*
 * 函数：udp_server_recv
 * 说明：UDP Server接收回调
 */
static void ICACHE_FLASH_ATTR
udp_server_recv(void *arg, char *pdata, uint16 len)
{
	struct espconn *pesp_conn = arg;
	sendesp_conn = arg;
	u16 send_data_len = 0;
	static uint8 pos,i,ssid_len,password_len;
	uint8 ssid[20]={},password[20]={};

	udp_get_peer_conn(arg);
	udp_print_conn(arg);
	os_printf("UDP收到消息：%s\r\n",pdata);
	if(strstr(pdata,"\"wifi_config\"")!=NULL)
	{
		/****************截取ssid*********************/
		pos=GetSubStrPos(pdata,"\"ssid_len\"");
		ssid_len=pdata[pos+11]-'0';
		if(pdata[pos+12]>='0'&&pdata[pos+12]<='9')
		{
			ssid_len=ssid_len*10+(pdata[pos+12]-'0');
		}
		else
			ssid_len=ssid_len;
		pos=GetSubStrPos(pdata,"\"ssid\"");

		os_strncpy(ssid,pdata+pos+8,ssid_len);
		/****************截取password*********************/
		pos=GetSubStrPos(pdata,"\"password_len\"");
		password_len=pdata[pos+15]-'0';
		if(pdata[pos+16]>='0'&&pdata[pos+16]<='9')
		{
			password_len=password_len*10+(pdata[pos+16]-'0');
		}
		else
			password_len=password_len;
		pos=GetSubStrPos(pdata,"\"password\"");

		os_strncpy(password,pdata+pos+12,password_len);
		 wifi_station_disconnect();
		 WIFI_Connect(ssid,password,wifiConnectCb);

	}
	else if(pdata[0]=='0'&&pdata[1]=='0')
	{
		WIFI_UDP_SendNews("00",2);
	}
	else
	{
		os_strcpy(mqtt_buff,pdata);
		pub_flag=1;
		tcp_send=1;
	}
	os_memset(pdata,0,os_strlen(pdata));
}


/*
 * 函数：udp_server_init
 * 说明：UDP Server 初始化
 * mode:1->AP模式   不需要IP
 * 		0->STATION模式 需要IP
 */
void ICACHE_FLASH_ATTR
udp_server_init(uint32 port,struct ip_addr *local_ip,uint8 mode)
{
	static struct espconn s_udp_server;
	static esp_udp s_espudp;

	s_udp_server.type = ESPCONN_UDP;
	s_udp_server.state = ESPCONN_NONE;
	s_udp_server.proto.udp = &s_espudp;
	s_udp_server.proto.udp->local_port = port;
	if(mode==0)
	{
		os_memcpy(s_udp_server.proto.tcp->local_ip,local_ip,4);
	}
	espconn_regist_recvcb(&s_udp_server, udp_server_recv);		//注册UDP接收回调函数
	espconn_regist_sentcb(&s_udp_server, udp_server_sent_cb); 	//注册UDP发送回调函数
	espconn_create(&s_udp_server);

	os_printf("udp_server_init\r\n");
}

/*
 * udp_server.c
 *
 *  Created on: 2017��7��3��
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
 * ����: udp_get_peer_conn
 * ˵������ȡ�Զ˵�UDP��Ϣ
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
 * ������udp_print_conn
 * ˵������ӡUDP�������ӵ���Ϣ
 */
static void ICACHE_FLASH_ATTR
udp_print_conn(void *arg)
{
  struct espconn *pesp_conn = arg;

  os_printf("[INFO] udp recv:"IPSTR":%d\r\n",
    IP2STR(pesp_conn->proto.udp->remote_ip), pesp_conn->proto.udp->remote_port);
}

/*
 * ������udp_server_sent_cb
 * ˵�������ͻص�
 */
static void ICACHE_FLASH_ATTR
udp_server_sent_cb(void *arg)
{
	os_printf("UDP���ͳɹ�\r\n");
}

/*
 * ������:void WIFI_UDP_SendNews(unsigned char *dat)
 * ����:��UDP������������Ϣ
 */
void ICACHE_FLASH_ATTR WIFI_UDP_SendNews(unsigned char *dat,uint16 len)
{
	espconn_send(sendesp_conn,dat,len);
}
/*
 * ������udp_server_recv
 * ˵����UDP Server���ջص�
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
	os_printf("UDP�յ���Ϣ��%s\r\n",pdata);
	if(strstr(pdata,"\"wifi_config\"")!=NULL)
	{
		/****************��ȡssid*********************/
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
		/****************��ȡpassword*********************/
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
 * ������udp_server_init
 * ˵����UDP Server ��ʼ��
 * mode:1->APģʽ   ����ҪIP
 * 		0->STATIONģʽ ��ҪIP
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
	espconn_regist_recvcb(&s_udp_server, udp_server_recv);		//ע��UDP���ջص�����
	espconn_regist_sentcb(&s_udp_server, udp_server_sent_cb); 	//ע��UDP���ͻص�����
	espconn_create(&s_udp_server);

	os_printf("udp_server_init\r\n");
}

#include "ets_sys.h"
#include "osapi.h"
#include "user_interface.h"
#include "mem.h"
#include "espconn.h"
#include "driver/uart.h"
#include "tcp.h"
#include "user_config.h"
typedef enum
{
  teClient,
  teServer
}teType;
typedef struct
{
    BOOL linkEn;
  BOOL teToff;
    uint8_t linkId;
    teType teType;
    uint8_t repeaTime;
    uint8_t changType;
    uint8 remoteIp[4];
    int32_t remotePort;
    struct espconn *pCon;
}linkConType;

typedef struct
{
  BOOL linkEn;
  BOOL teToff;
  uint8_t linkId;
  teType teType;
  uint8_t repeaTime;
  struct espconn *pCon;
} espConnectionType;

linkConType pLink;
espConnectionType user_contype;
static struct espconn *pTcpServer;

extern uint8 mqtt_buff[200];
extern uint8 dev_sid[15];


extern uint8 pub_flag;

uint8 tcp_send=0;
//os_event_t    procTaskQueue[procTaskQueueLen];

/*
 *函数名:void TcpServer_Listen_Recv(void *arg, char *pdata, unsigned short len)
 *功能:接收监听函数
 */
void ICACHE_FLASH_ATTR TcpServer_Listen_Recv(void *arg, char *pdata, unsigned short len)
{
	static uint8 pos,i,ssid_len,password_len;
	uint8 ssid[20]={},password[20]={},ack[50]={};
#if tcp_debug
    os_printf("收到PC发来的数据：%s\r\n",pdata);//将客户端发过来的数据打印出来
#endif

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
		WIFI_TCP_SendNews("00",2);
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
 * 函数名:void TcpServer_Listen_Recb(void *arg, sint8 errType)
 * 功能:连接监听函数
 */
void ICACHE_FLASH_ATTR ICACHE_FLASH_ATTR TcpServer_Listen_recon_cb(void *arg, sint8 errType)
{

    struct espconn *pespconn = (struct espconn *)arg;
      linkConType *linkTemp = (linkConType *)pespconn->reverse;
}
/*
 * 函数名:void Tcp_Server_Listen_discon_cb(void *arg)
 * 功能:正常断开时监听函数
 */
void ICACHE_FLASH_ATTR Tcp_Server_Listen_discon_cb(void *arg)
{
      struct espconn *pespconn = (struct espconn *) arg;
      linkConType *linkTemp = (linkConType *) pespconn->reverse;
#if tcp_debug
      os_printf("连接已经断开\r\n");
#endif
}
/*
 * 函数名:void Tcp_Server_Listen_sent_cb(void *arg)
 * 功能:发送成功监听函数
 */
void ICACHE_FLASH_ATTR ICACHE_FLASH_ATTR Tcp_Server_Listen_sent_cb(void *arg)
{
      struct espconn *pespconn = (struct espconn *) arg;
     linkConType *linkTemp = (linkConType *) pespconn->reverse;
#if tcp_debug
     os_printf("发送数据成功！！\r\n");
#endif
}
/*
 * 函数名:void TcpServer_Listen_PCon(void *arg)
 * 功能:手机连入AP监听函数
 */
void ICACHE_FLASH_ATTR ICACHE_FLASH_ATTR TcpServerListen_PCon(void *arg)
{

    struct espconn *pespconn = (struct espconn *)arg;
      pLink.teToff = FALSE;
      pLink.linkId = 1;
      pLink.teType = teServer;
      pLink.repeaTime = 0;
      pLink.pCon = pespconn;
      pespconn->reverse = &pLink;
      espconn_regist_recvcb(pespconn, TcpServer_Listen_Recv);                           //注册接收监听函数
      espconn_regist_reconcb(pespconn, TcpServer_Listen_recon_cb);                      //注册连接监听函数
      espconn_regist_disconcb(pespconn, Tcp_Server_Listen_discon_cb);                   //注册正常断开时监听函数
      espconn_regist_sentcb(pespconn, Tcp_Server_Listen_sent_cb);                       //注册发送成功监听函数
#if tcp_debug
      os_printf("连接已经成功\r\n");
#endif
}
/*
 * 函数名:void WIFI_Server_MODE()
 * 功能:设置服务器模式
 */
void ICACHE_FLASH_ATTR WIFIServerMode()
{
	espconn_tcp_set_max_con(5);                                         //设置TCP连接的最大多少
	pTcpServer = (struct espconn *)os_zalloc(sizeof(struct espconn));
	pTcpServer->type = ESPCONN_TCP;                                     //TCP服务
	pTcpServer->state = ESPCONN_NONE;                                   //状态
	pTcpServer->proto.tcp = (esp_tcp *)os_zalloc(sizeof(esp_tcp));
	pTcpServer->proto.tcp->local_port = 8888;                           //端口号
	espconn_regist_connectcb(pTcpServer, TcpServerListen_PCon);
	espconn_accept(pTcpServer);
	espconn_regist_time(pTcpServer, 180, 0);                            //设置超时断开时间 单位s
}
/*
 * 函数名:void WIFI_TCP_SendNews(unsigned char *dat)
 * 功能:像TCP服务器发送消息
 */
void ICACHE_FLASH_ATTR WIFI_TCP_SendNews(unsigned char *dat,uint16 len)
{
    espconn_send(pLink.pCon,dat,len);
}


void ICACHE_FLASH_ATTR station_server_init(struct ip_addr *local_ip,int port){
    LOCAL struct espconn esp_conn;

    espconn_tcp_set_max_con(5);                                         //设置TCP连接的最大多少

    esp_conn.type=ESPCONN_TCP;
    esp_conn.state=ESPCONN_NONE;
    esp_conn.proto.tcp=(esp_tcp *)os_malloc(sizeof(esp_tcp));

    os_memcpy(esp_conn.proto.tcp->local_ip,local_ip,4);
    esp_conn.proto.tcp->local_port=port;


    //注册连接成功的回调函数和连接失败重新连接的回调函数
    espconn_regist_connectcb(&esp_conn,TcpServerListen_PCon);//注册一个连接成功回调函数

    espconn_accept(&esp_conn);
	 espconn_regist_time(&esp_conn,60,0);//设置tcp超时时间，60S后与不通信的客户端断开连接
}

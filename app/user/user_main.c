/* main.c -- MQTT client example
*
* Copyright (c) 2014-2015, Tuan PM <tuanpm at live dot com>
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Redis nor the names of its contributors may be used
* to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
#include "user_config.h"
#include "ota.h"
#if tcp_server
	#include "tcp.h"
#else
	#include "udp_server.h"
#endif
/**********************************************************************/
#define SYS_VER  			"gp08-slkg01-sw-v1.2"//版本号
#define HARD_VER  			"gp08-slkg01-hw-v1.4"//版本号

#define DEVICE_TYPE 		"gh_9e2cff3dfa51" //wechat public number
#define DEVICE_ID 			"122475" //model ID

#define DEFAULT_LAN_PORT 	12476

/*******************************74HC595***************************/
/* Q7   左侧灯
 * Q6	继电器
 * Q5	继电器
 * Q2	右侧灯
 *
 */
//SH_CP
#define RCK_PIN_NUM         12
#define RCK_PIN_FUNC        FUNC_GPIO12
#define RCK_PIN_MUX         PERIPHS_IO_MUX_MTDI_U
//ST_CP
#define SCK_PIN_NUM         14
#define SCK_PIN_FUNC        FUNC_GPIO14
#define SCK_PIN_MUX         PERIPHS_IO_MUX_MTMS_U
//DS
#define DS_PIN_NUM          15
#define DS_PIN_FUNC         FUNC_GPIO15
#define DS_PIN_MUX          PERIPHS_IO_MUX_MTDO_U

//按键
#define KEY1_PIN_NUM         2
#define KEY1_PIN_FUNC        FUNC_GPIO2
#define KEY1_PIN_MUX         PERIPHS_IO_MUX_GPIO2_U

#define KEY2_PIN_NUM         5
#define KEY2_PIN_FUNC        FUNC_GPIO5
#define KEY2_PIN_MUX         PERIPHS_IO_MUX_GPIO5_U
//蜂鸣器
#define BEEP_PIN_NUM         13
#define BEEP_PIN_FUNC        FUNC_GPIO13
#define BEEP_PIN_MUX         PERIPHS_IO_MUX_MTCK_U

#define POWER_PIN_NUM         0
#define POWER_PIN_FUNC        FUNC_GPIO0
#define POWER_PIN_MUX         PERIPHS_IO_MUX_GPIO0_U

//#define Smart_LED_ON  GPIO_OUTPUT_SET(GPIO_ID_PIN(SMART_LED_PIN_NUM), 0);
//#define Smart_LED_OFF GPIO_OUTPUT_SET(GPIO_ID_PIN(SMART_LED_PIN_NUM), 1);

/**********************************************************/
#define SCK_HIGH 	GPIO_OUTPUT_SET(GPIO_ID_PIN(SCK_PIN_NUM), 1);
#define SCK_LOW		GPIO_OUTPUT_SET(GPIO_ID_PIN(SCK_PIN_NUM), 0);

#define RCK_HIGH	GPIO_OUTPUT_SET(GPIO_ID_PIN(RCK_PIN_NUM), 1);
#define RCK_LOW		GPIO_OUTPUT_SET(GPIO_ID_PIN(RCK_PIN_NUM), 0);

#define DS_HIGH		GPIO_OUTPUT_SET(GPIO_ID_PIN(DS_PIN_NUM), 1);
#define DS_LOW		GPIO_OUTPUT_SET(GPIO_ID_PIN(DS_PIN_NUM), 0);
/***********************************************************/
#define BEEP_ON  	GPIO_OUTPUT_SET(GPIO_ID_PIN(BEEP_PIN_NUM), 1);
#define BEEP_OFF 	GPIO_OUTPUT_SET(GPIO_ID_PIN(BEEP_PIN_NUM), 0);

#define POWER_ON  	GPIO_OUTPUT_SET(GPIO_ID_PIN(POWER_PIN_NUM), 0);
#define POWER_OFF 	GPIO_OUTPUT_SET(GPIO_ID_PIN(POWER_PIN_NUM), 1);

#define RELAY1_ON  	(1<<6)
#define RELAY1_OFF 	~(1<<6)
#define RELAY2_ON  	(1<<5)
#define RELAY2_OFF 	~(1<<5)
#define RELAY3_ON  	(1<<4)
#define RELAY3_OFF 	~(1<<4)

#define LED1_ON  	~(1<<6)
#define LED1_OFF 	(1<<6)
#define LED2_ON  	~(1<<2)
#define LED2_OFF 	(1<<2)
#define LED3_ON  	~(1<<1)
#define LED3_OFF 	(1<<1)

LOCAL os_timer_t flash_light_timer;

LOCAL os_timer_t serv_timer;
uint8 mqtt_buff[200];				//mqtt接收数据缓存
uint8 pub_topic[50],sub_topic[50];	//mqtt发布和订阅主题
uint8 service_topic[80];			//向服务器返回状态主题
uint8 pub_flag=0;
uint8 on_off_flag=0;
uint8 send595_data=0x00;
uint8 key1_pass=0,key2_pass=0,key3_pass=0;
uint8 long_pass=0;
uint8 first_in=0;


uint8 channel_sta[3];//[0]left,[1]right
uint8 send_serv;

extern uint8 tcp_send;

uint8 local_ip[20];					//记录本地IP，用于station模式的tcp service

uint8 dev_sid[15];					//记录设备SID

struct	softap_config	ap_config;
LOCAL os_timer_t sys_restart_timer;

LOCAL esp_udp ssdp_udp;
LOCAL struct espconn pssdpudpconn;
LOCAL os_timer_t ssdp_time_serv;

//按键相关
static struct keys_param switch_param;
static struct single_key_param *switch_signle;

char temp_str[30];    // 临时子串，查找字符串相关

LOCAL os_timer_t pub_timer;;
LOCAL os_timer_t check_ip_timer;
LOCAL os_timer_t key_timer;

void ICACHE_FLASH_ATTR  socket_timer_callback();
/*************************74HC595***************************/
void SendTo595(uint8 data)
{
	uint8 i=0;
	uint8 num=0;
	num=data;
	for(i=0;i<8;i++)
	{
		if(num&0x80)
		{
			DS_HIGH;
		}
		else
		{
			DS_LOW;
		}
		num=num<<1;
		SCK_LOW;
		os_delay_us(10);
		SCK_HIGH;
	}
	RCK_LOW;
	os_delay_us(10);
	RCK_HIGH;
}
/****************************************************************************
						MQTT
******************************************************************************/
MQTT_Client mqttClient;
typedef unsigned long u32_t;
static ETSTimer sntp_timer;

void  ICACHE_FLASH_ATTR sys_restart_timer_callback()
{
	static uint8 i;
	i++;
	if(i>6)
	{
		i=0;
		os_timer_disarm(&sys_restart_timer);
		system_restart();
	}
}
void sys_restart()
{
	 os_timer_disarm(&sys_restart_timer);
	 os_timer_setfn(&sys_restart_timer, (os_timer_func_t *)sys_restart_timer_callback, NULL);
	 os_timer_arm(&sys_restart_timer, 1000, 1);//1s
}


void sntpfn()
{
    u32_t ts = 0;
    char* current_time="Wed Dec 07 16:34:45 2016";

    ts = sntp_get_current_timestamp();
    current_time=sntp_get_real_time(ts);

    if (ts == 0)
    {
        //os_printf("did not get a valid time from sntp server\n");
    } else
    {
    	//os_printf("current time : %s\n", current_time);
    }

}
void ICACHE_FLASH_ATTR
my_sntp_init(void)
{
	 sntp_setservername(0, "pool.ntp.org");        // set sntp server after got ip address
	 sntp_init();
	 os_timer_disarm(&sntp_timer);
	 os_timer_setfn(&sntp_timer, (os_timer_func_t *)sntpfn, NULL);
	 os_timer_arm(&sntp_timer, 1000, 1);//1s
}

void ICACHE_FLASH_ATTR  wifiConnectCb(uint8_t status)
{


	/*struct ip_info info; //用于获取IP地址的信息
    if(status == STATION_GOT_IP){
    	wifi_get_ip_info(STATON_IF,&info);
    	station_server_init(&info.ip,8888);

    } else {
          MQTT_Disconnect(&mqttClient);
    }*/
}



void mqttConnectedCb(uint32_t *args)
{
	uint8 init_buff[200];
	os_sprintf(init_buff,"{\"cmd\":\"i am ok\",\"dev\":\"switchsl\",\"sys_ver\":\"%s\",\"hard_ver\":\"%s\",\"sid\":\"%s\"}",SYS_VER,HARD_VER,dev_sid);
    MQTT_Client* client = (MQTT_Client*)args;
#if 1
    INFO("MQTT: Connected\r\n");
#endif
    wifi_set_opmode_current(0x01);
    MQTT_Subscribe(client,  sub_topic, 1);
    MQTT_Publish(&mqttClient,  pub_topic,init_buff, os_strlen(init_buff), 0, 0);


}

void mqttDisconnectedCb(uint32_t *args)
{
    MQTT_Client* client = (MQTT_Client*)args;

#if 1
    INFO("MQTT: Disconnected\r\n");
#endif
}

void mqttPublishedCb(uint32_t *args)
{
    MQTT_Client* client = (MQTT_Client*)args;

    pub_flag=0;
    os_memset(mqtt_buff,0,sizeof(mqtt_buff));
#if 1
    INFO("MQTT: Published\r\n");
#endif
}

void mqttDataCb(uint32_t *args, const char* topic, uint32_t topic_len, const char *data, uint32_t data_len)
{
    char *topicBuf = (char*)os_zalloc(topic_len+1),
            *dataBuf = (char*)os_zalloc(data_len+1);

    MQTT_Client* client = (MQTT_Client*)args;

    os_memcpy(topicBuf, topic, topic_len);
    topicBuf[topic_len] = 0;

    os_memset(mqtt_buff,0,sizeof(mqtt_buff));
    os_memcpy(dataBuf, data, data_len);
    os_memcpy(mqtt_buff, data, data_len);
    dataBuf[data_len] = 0;
    pub_flag=1;
#if 1
    INFO("Receive topic: %s, data: %s \r\n", topicBuf, dataBuf);
#endif
    os_free(topicBuf);
    os_free(dataBuf);
}
/*
 * 函数名:void dhcps_lease()
 * 功能:分配ip范围
 */
void ICACHE_FLASH_ATTR dhcps_lease(void)
{

	struct	dhcps_lease	dhcp_lease;
	struct ip_info info;
	wifi_softap_dhcps_stop();//设置前关闭DHCP
	IP4_ADDR(&dhcp_lease.start_ip,192,168,5,1);

	IP4_ADDR(&dhcp_lease.end_ip,192,168,5,100);

	IP4_ADDR(&info.ip, 192, 168, 5, 1);
	IP4_ADDR(&info.gw, 192, 168, 5, 1);
	IP4_ADDR(&info.netmask, 255, 255, 255, 0);
	wifi_set_ip_info(SOFTAP_IF, &info);

	wifi_softap_set_dhcps_lease(&dhcp_lease);
	wifi_softap_dhcps_start();

}
/*
 * 函数名:void Wifi_AP_Init()
 * 功能wifi_ap初始化
 */
void ICACHE_FLASH_ATTR WIFIAPInit()
{
    struct softap_config apConfig;

    /***************************模式设置************************************/
         if(wifi_set_opmode(0x03)){          //  设置为AP模式

         }else{

         }
    /***************************名字设通道置************************************/
	  os_bzero(&apConfig, sizeof(struct softap_config));
	  wifi_softap_get_config(&apConfig);
	  apConfig.ssid_len=0;                      //设置ssid长度
	  os_memset(apConfig.ssid,' ',strlen(apConfig.ssid));

	  os_sprintf(apConfig.ssid,"grasp_switchsl-%s",dev_sid);			//设置ssid名字

	 // os_strcpy(apConfig.password,"12345678");  //设置密码
	 // apConfig.authmode =3;                     //设置加密模式
	  wifi_softap_set_config(&apConfig);        //配置

	  dhcps_lease();
}
void ICACHE_FLASH_ATTR  save_flash(uint32 des_addr,uint32* data)
{
	spi_flash_erase_sector(des_addr);
	spi_flash_write(des_addr * SPI_FLASH_SEC_SIZE,data, sizeof(data));
}
void  ICACHE_FLASH_ATTR load_flash(uint32 des_addr,uint32* data)
{
	spi_flash_read(des_addr * SPI_FLASH_SEC_SIZE,data, sizeof(data));
}

void ICACHE_FLASH_ATTR  serv_timer_callback()//用于发送状态给服务器，服务器保存当前状态
{
	send_serv=1;
	os_timer_disarm(&serv_timer);
	os_printf("send to serv\r\n");
}
/****************************************收到数据开始处理*******************************************/
void ICACHE_FLASH_ATTR  pub_timer_callback()
{
	uint8 frist_pos=0;
	uint16 data=0;
	static uint8 pub_buff[200];		//发布数据缓存
	static uint8 state1[10];
	static uint8 state2[10];
	static uint8 state3[10];
	static uint8 ip[4]={192,168,1,3};
	if(pub_flag==1)
	{
		pub_flag=0;
		os_memset(state1,0,os_strlen(state1));
		os_memset(state2,0,os_strlen(state2));
		os_memset(pub_buff,0,os_strlen(pub_buff));
		if(strstr(mqtt_buff,dev_sid)!=NULL)
		{
/************************************读开关状态************************************************************/
			if(strstr(mqtt_buff,"\"cmd\":\"wifi_switchsl_read\"")!=NULL)
			{
				on_off_flag=1;
			}
/*****************************************ota**********************************************************/
			if(strstr(mqtt_buff,"\"cmd\":\"wifi_switchsl_update\"")!=NULL)
			{
				os_sprintf(pub_buff,"{\"cmd\":\"wifi_switchsl_update_ack\",\"sid\":\"%s\"}",dev_sid);
				if(tcp_send==1)
				{
					tcp_send=0;
#if tcp_server
					WIFI_TCP_SendNews(pub_buff,os_strlen(pub_buff));
#else
					WIFI_UDP_SendNews(pub_buff,os_strlen(pub_buff));
#endif
				}
				else
					MQTT_Publish(&mqttClient,  pub_topic,pub_buff, os_strlen(pub_buff), 0, 0);
				ota_start_Upgrade(ip, 80,"8266update/WiFi_Switchsl/");
			}
/*********************************************获取IP*************************/
			if(strstr(mqtt_buff,"\"cmd\":\"wifi_equipment_ping\"")!=NULL)
			{
				os_sprintf(pub_buff,"{\"cmd\":\"wifi_equipment_ping_ack\",\"ip\":\"%s\",\"sid\":\"%s\"}",local_ip,dev_sid);
				MQTT_Publish(&mqttClient,  pub_topic,pub_buff, os_strlen(pub_buff), 0, 0);
			}
/************************************开关*****************************************/
			if(strstr(mqtt_buff,"\"cmd\":\"wifi_switchsl\"")!=NULL)
			{
				frist_pos=GetSubStrPos(mqtt_buff,"\"channel\"");
				data=mqtt_buff[frist_pos+10]-'0';
				if(strstr(mqtt_buff,"\"on\"")!=NULL)
				{
					if(data==1)
					{
						channel_sta[0]=1;
					}
					if(data==2)
					{
						channel_sta[1]=1;
					}
					if(data==3)
					{
						channel_sta[2]=1;
					}
				}
				if(strstr(mqtt_buff,"\"off\"")!=NULL)
				{
					if(data==1)
					{
						channel_sta[0]=0;
					}
					if(data==2)
					{
						channel_sta[1]=0;
					}
					if(data==3)
					{
						channel_sta[2]=0;
					}
				}
				on_off_flag=1;
			}
		}
	}
	if(on_off_flag==1)
	{
		if(channel_sta[0]==0)
		{
			send595_data=send595_data|(LED1_OFF);
			send595_data=send595_data&(RELAY1_OFF);
			os_strcpy(state1,"\"off\"");
		}
		else
		{
			send595_data=send595_data&(LED1_ON);
			send595_data=send595_data|(RELAY1_ON);
			os_strcpy(state1,"\"on\"");
		}
		if(channel_sta[1]==0)//key2
		{
			send595_data=send595_data|(LED2_OFF);
			send595_data=send595_data&(RELAY2_OFF);
			os_strcpy(state2,"\"off\"");
		}
		else
		{
			os_strcpy(state2,"\"on\"");
			send595_data=send595_data&(LED2_ON);
			send595_data=send595_data|(RELAY2_ON);
		}
		if(channel_sta[2]==0)//key3
		{
			send595_data=send595_data|(LED3_OFF);
			send595_data=send595_data&(RELAY3_OFF);
			os_strcpy(state3,"\"off\"");
		}
		else
		{
			os_strcpy(state3,"\"on\"");
			send595_data=send595_data&(LED3_ON);
			send595_data=send595_data|(RELAY3_ON);
		}
		SendTo595(send595_data);
		//os_printf("send595_data=%x\r\n",send595_data);
		os_sprintf(pub_buff,"{\"cmd\":\"wifi_switchsl_ack\",\"channel_1\":%s,\"channel_2\":%s,\"channel_3\":%s,\"sys_ver\":\"%s\",\"hard_ver\":\"%s\",\"sid\":\"%s\"}"
				,state1,state2,state3,SYS_VER,HARD_VER,dev_sid);
		if(tcp_send==1)
		{
			tcp_send=0;

#if tcp_server
			WIFI_TCP_SendNews(pub_buff,os_strlen(pub_buff));
#else
			WIFI_UDP_SendNews(pub_buff,os_strlen(pub_buff));
#endif
			//***********向服务器，3S后发送数据给服务器保存数据，连续触发UDP开关后，会不断刷新清零定时器*************************/
			os_timer_disarm(&serv_timer);
			os_timer_setfn(&serv_timer, (os_timer_func_t *)serv_timer_callback, NULL);
			os_timer_arm(&serv_timer, 3000, 1);//3000ms
			//****************************************************************/
		}
		else
			MQTT_Publish(&mqttClient,  pub_topic,pub_buff, os_strlen(pub_buff), 0, 0);

		save_flash(CFG_LOCATION + 4,(uint32 *)&channel_sta);
		on_off_flag=0;
	}
	if(send_serv==1)
	{
		MQTT_Publish(&mqttClient,  service_topic,pub_buff, os_strlen(pub_buff), 0, 0);
		send_serv=0;
	}

}
/********************************配网指示灯闪烁回调函数**********************************************/
void ICACHE_FLASH_ATTR  flash_light_timer_callback()
{
	static uint8 flag=0;
	if(flag==0)
	{
		flag=1;
		//Smart_LED_OFF;
	}
	else
	{
		flag=0;
		//Smart_LED_ON;
	}
}

//长按按键开始配网
static void Switch_LongPress_Handler( void )
{
	struct ip_info ipConfig;
	static uint8 ip[10];
#if smartconfig
		wifi_station_dhcpc_start();
		smartconfig_set_type(SC_TYPE_ESPTOUCH_AIRKISS); //SC_TYPE_ESPTOUCH,SC_TYPE_AIRKISS,SC_TYPE_ESPTOUCH_AIRKISS
		wifi_set_opmode(STATION_MODE);
		smartconfig_start(smartconfig_done);
#endif
		//Smart_LED_OFF;
		os_timer_disarm(&sntp_timer);
		os_timer_disarm(&check_ip_timer);
/****************************配网指示灯开始闪烁*****************************************/
		os_timer_disarm(&flash_light_timer);
		os_timer_setfn(&flash_light_timer, (os_timer_func_t *)flash_light_timer_callback, NULL);
		os_timer_arm(&flash_light_timer, 300, 1);//300ms
/*******************************************************************************/
		wifi_station_disconnect();
		MQTT_Disconnect(&mqttClient);
		os_delay_us(60000);
		WIFIAPInit();
#if tcp_server
	    WIFIServerMode();
#else
	    udp_server_init(8888,&ipConfig.ip,1);
#endif
	    os_printf("long pass \n");

}

void key_timer_callback()
{
	struct ip_info ipConfig;
	static uint8 ip[10];

	os_timer_disarm(&key_timer);
	first_in=0;
	if(long_pass==1)
	{
		long_pass=2;
		key3_pass=0;
		key2_pass=0;
		key1_pass=0;
		BEEP_OFF;
		os_timer_disarm(&sntp_timer);
		os_timer_disarm(&check_ip_timer);

		wifi_station_disconnect();
		MQTT_Disconnect(&mqttClient);
		os_delay_us(60000);
		WIFIAPInit();
#if tcp_server
		WIFIServerMode();
#else
		udp_server_init(8888,&ipConfig.ip,1);
#endif
		os_printf("long pass\n");
	}
	else
	{
		if(long_pass==2)
		{
			long_pass=0;
			key1_pass=0;
			key2_pass=0;
			key3_pass=0;
			system_restart();
			return;
		}
		if(key1_pass==1)
		{
			key1_pass=0;
			//os_printf("key1 pass\n");
			if(channel_sta[0]==0)
			{
				channel_sta[0]=1;
			}
			else
				channel_sta[0]=0;
		}
		else if(key2_pass==1)
		{
			key2_pass=0;
			//os_printf("key2 pass\n");
			if(channel_sta[1]==0)
			{
				channel_sta[1]=1;
			}
			else
				channel_sta[1]=0;
		}
		else if(key3_pass==1)
		{
			key3_pass=0;
			if(channel_sta[2]==0)
			{
				channel_sta[2]=1;
			}
			else
				channel_sta[2]=0;
		}
		pub_flag=1;
		on_off_flag=1;
	}
}
void gpio_intr_handler()
{
	static uint32 time1=0,time2=0,err=0;
	static uint16 i=0,delay=500;
	static uint8 num=0,flag=0;
	uint32 gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
	ETS_GPIO_INTR_DISABLE();//关闭中断
	if(first_in==0)
	{
		first_in=1;
		time1=system_get_time();
	}
	else
	{
		first_in=0;
		time2=system_get_time();
		err=time2-time1;
		//os_printf("time=%d\n",err);//输出1111
		if(key1_pass==0&&key2_pass==0&&key3_pass==0)
		{
			if(err>1000&&err<3000)
			{
				if(flag==0)
					flag=1;
				else if(flag==1)
				{
					flag=2;
				}
			}
			if(flag==1)
			{
				if(err<500)
				{
					num++;
				}
				else if(err>3000)
				{
					flag=0;
					key3_pass=1;
					delay=500;
					os_printf("key3 pass\n");
				}
			}
			if(flag==2)
			{
				flag=0;
				if(num==2)
				{
					key2_pass=1;
					delay=1000;
					os_printf("key2 pass\n");
				}
				else if(num==3)
				{
					key1_pass=1;
					delay=800;
					os_printf("key1 pass\n");
				}
				//os_printf("i=%d\n",num);//输出1111
				num=0;
			}
		}

		if(--delay==0)
		{
			delay=500;
			BEEP_ON;
			long_pass=1;
		}
	}
	os_timer_disarm(&key_timer);
	os_timer_setfn(&key_timer, (os_timer_func_t *)key_timer_callback, NULL);
	os_timer_arm(&key_timer, 10, 1);//10ms

	GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status);//清中断标志
	ETS_GPIO_INTR_ENABLE();//打开中断
}
void gpio_init(void)
{
	PIN_FUNC_SELECT(SCK_PIN_MUX,SCK_PIN_FUNC);
	PIN_FUNC_SELECT(RCK_PIN_MUX,RCK_PIN_FUNC);
	PIN_FUNC_SELECT(DS_PIN_MUX,DS_PIN_FUNC);
	PIN_FUNC_SELECT(BEEP_PIN_MUX,BEEP_PIN_FUNC);
	PIN_FUNC_SELECT(POWER_PIN_MUX,POWER_PIN_FUNC);

	GPIO_DIS_OUTPUT(GPIO_ID_PIN(SCK_PIN_NUM));
	GPIO_DIS_OUTPUT(GPIO_ID_PIN(RCK_PIN_NUM));
	GPIO_DIS_OUTPUT(GPIO_ID_PIN(DS_PIN_NUM));


	PIN_FUNC_SELECT(KEY1_PIN_MUX,KEY1_PIN_FUNC);
	GPIO_DIS_OUTPUT(GPIO_ID_PIN(KEY1_PIN_NUM));
	//PIN_FUNC_SELECT(KEY2_PIN_MUX,KEY2_PIN_FUNC);
	//GPIO_DIS_OUTPUT(GPIO_ID_PIN(KEY2_PIN_NUM));

	PIN_PULLUP_EN(KEY1_PIN_MUX);//GPIO5使能上拉
	PIN_PULLUP_EN(KEY2_PIN_MUX);//GPIO4使能上拉


	ETS_GPIO_INTR_DISABLE();//关闭中断
	ETS_GPIO_INTR_ATTACH(&gpio_intr_handler, NULL);//设置中断函数
	gpio_pin_intr_state_set(GPIO_ID_PIN(KEY1_PIN_NUM), GPIO_PIN_INTR_ANYEDGE);//设置中断类型双边沿
	//gpio_pin_intr_state_set(GPIO_ID_PIN(KEY2_PIN_NUM), GPIO_PIN_INTR_ANYEDGE);//设置中断类型

	ETS_GPIO_INTR_ENABLE();//打开中断
	POWER_ON;
}


/************************************************************************


获取字符串中某个子字符串的首字母的下标

*************************************************************************/
//
void ICACHE_FLASH_ATTR  ReadStrUnit(char * str,char *temp_str,int idx,int len)
{
   static uint16 index = 0;
   for(index=0; index < len; index++)
   {
       temp_str[index] = str[idx+index];
   }
   temp_str[index] = '\0';
}

int  ICACHE_FLASH_ATTR GetSubStrPos(char *str1,char *str2)
{
   int idx = 0;
   int len1 = strlen(str1);
   int len2 = strlen(str2);

   if( len1 < len2)
   {
       os_printf("error 1 \n");
       return -1;
   }

   while(1)
   {
       ReadStrUnit(str1,temp_str,idx,len2);
       if(strcmp(str2,temp_str)==0)break;
       idx++;
       if(idx>=len1)return -1;
   }

   return idx;
}



/***********************************************
 初始化
 **********************************/
void  ICACHE_FLASH_ATTR MQTT_Init()
{
	os_sprintf(sub_topic,"iotbroad/iot/switchsl/%s",dev_sid);
	os_sprintf(pub_topic,"iotbroad/iot/switchsl_ack/%s",dev_sid);
	os_sprintf(service_topic,"iotbroad/iot/dev/switchsl_ack/%s",dev_sid);
	MQTT_InitConnection(&mqttClient, sysCfg.mqtt_host, sysCfg.mqtt_port, sysCfg.security);

	MQTT_InitClient(&mqttClient, sysCfg.device_id, sysCfg.mqtt_user, sysCfg.mqtt_pass, sysCfg.mqtt_keepalive, 1);

	MQTT_InitLWT(&mqttClient, "/lwt", "offline", 0, 0);
	MQTT_OnConnected(&mqttClient, mqttConnectedCb);
	MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
	MQTT_OnPublished(&mqttClient, mqttPublishedCb);
	MQTT_OnData(&mqttClient, mqttDataCb);
}

/*************************************脸上wifi后检测IP*******************************************/
void  ICACHE_FLASH_ATTR check_ip_timer_callback()
{
	static uint8_t wifiStatus = STATION_IDLE,flag=0;
	struct ip_info ipConfig;
	uint8 ip[10];

	wifi_get_ip_info(STATION_IF, &ipConfig);
	wifiStatus = wifi_station_get_connect_status();
	if (wifiStatus == STATION_GOT_IP && ipConfig.ip.addr != 0)
	{
		if(flag==1)
		{
			flag=0;
			wifi_set_opmode(0x01);
			 my_sntp_init();//获取网络时间
#if tcp_server
			 station_server_init(&ipConfig.ip,8888);
#else
			 udp_server_init(8888,&ipConfig.ip,0);
#endif
			 os_memcpy(ip,&ipConfig.ip,4);

			 os_sprintf(local_ip,"%d.%d.%d.%d",ip[0],ip[1],ip[2],ip[3]);

			 MQTT_Connect(&mqttClient);
		}
		 //os_timer_disarm(&check_ip_timer);
	}
	else
		flag=1;
}

void  ICACHE_FLASH_ATTR check_ip()
{
	 os_timer_disarm(&check_ip_timer);
	 os_timer_setfn(&check_ip_timer, (os_timer_func_t *)check_ip_timer_callback, NULL);
	 os_timer_arm(&check_ip_timer, 100, 1);//100ms
}

/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
 *******************************************************************************/
uint32 ICACHE_FLASH_ATTR
user_rf_cal_sector_set(void)
{
    enum flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;
    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        	 rf_cal_sec = 512 - 5;
        	 break;

        case FLASH_SIZE_16M_MAP_1024_1024:
			rf_cal_sec = 512 - 5;
			break;

		case FLASH_SIZE_32M_MAP_512_512:
			rf_cal_sec = 1024 - 5;
			break;
		case FLASH_SIZE_32M_MAP_1024_1024:
			rf_cal_sec = 1024 - 5;
			break;

		case FLASH_SIZE_64M_MAP_1024_1024:
			rf_cal_sec = 2048 - 5;
			break;
		case FLASH_SIZE_128M_MAP_1024_1024:
			rf_cal_sec = 4096 - 5;
			break;
		default:
			rf_cal_sec = 0;
			break;
    }

    return rf_cal_sec;
}



void  ICACHE_FLASH_ATTR to_scan(void)
{
	/***************************开启任务**********************************/
	os_timer_disarm(&pub_timer);
	os_timer_setfn(&pub_timer, (os_timer_func_t *)pub_timer_callback, NULL);
	os_timer_arm(&pub_timer, 200, 1);//200ms
}
/* Create a bunch of objects as demonstration. */

void user_init(void)
{
	uart_init(BIT_RATE_115200, BIT_RATE_115200);
	BEEP_ON;
	os_delay_us(60000);
	BEEP_OFF;
   	os_sprintf(dev_sid,"%x%x",system_get_chip_id(),spi_flash_get_id());

   	WIFIAPInit();

	CFG_Load();

	MQTT_Init();

	gpio_init();

	load_flash(CFG_LOCATION + 4,(uint32 *)channel_sta);

	on_off_flag=1;
	

 //检测到连接ip之后连接mqtt
	check_ip();

	wifi_set_sleep_type(MODEM_SLEEP_T);

   	system_init_done_cb(to_scan);
	INFO("\r\nSystem started ...\r\n");
	os_printf("SYS_Ver is %s\r\n",SYS_VER);
	os_printf("Hard_Ver is %s\r\n",HARD_VER);
	os_printf("dev_sid is %s\r\n",dev_sid);
}




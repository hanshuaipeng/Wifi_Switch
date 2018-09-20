/*
 * wifi.c
 *
 *  Created on: Dec 30, 2014
 *      Author: Minh
 */
#include "wifi.h"
#include "user_interface.h"
#include "osapi.h"
#include "espconn.h"
#include "os_type.h"
#include "mem.h"
#include "mqtt_msg.h"
#include "debug.h"
#include "user_config.h"
#include "config.h"


//1,接收成功
//2,密码错误
//3，连接错误
uint8 connect_sta=0;
extern uint8 dev_sid[15];
struct ip_info ipConfig;
static ETSTimer WiFiLinker;
WifiCallback wifiCb = NULL;
static uint8_t wifiStatus = STATION_IDLE, lastWifiStatus = STATION_IDLE;
static void ICACHE_FLASH_ATTR wifi_check_ip(void *arg)
{

	uint8 ack[60]={};
	os_timer_disarm(&WiFiLinker);
	wifi_get_ip_info(STATION_IF, &ipConfig);
	wifiStatus = wifi_station_get_connect_status();
	if (wifiStatus == STATION_GOT_IP)
	{
		INFO("STATION_GOT_IP\r\n");
		connect_sta=1;
		os_sprintf(ack,"{\"cmd\":\"wifi_config_ok\",\"sid\":\"%s\",\"connect_sta\":%d}",dev_sid,connect_sta);

		WIFI_TCP_SendNews(ack,os_strlen(ack));
		sys_restart();
	}
	if (wifiStatus == STATION_GOT_IP && ipConfig.ip.addr != 0)
	{
		os_timer_setfn(&WiFiLinker, (os_timer_func_t *)wifi_check_ip, NULL);
		os_timer_arm(&WiFiLinker, 2000, 0);

	}
	else
	{
		if(wifi_station_get_connect_status() == STATION_WRONG_PASSWORD)
		{

			INFO("STATION_WRONG_PASSWORD\r\n");
			wifi_set_opmode(SOFTAP_MODE);
			WIFIServerMode();
			connect_sta=2;
			//wifi_station_connect();
			 os_sprintf(ack,"{\"cmd\":\"wifi_config_ok\",\"sid\":\"%s\",\"connect_sta\":%d}",dev_sid,connect_sta);
			WIFI_TCP_SendNews(ack,os_strlen(ack));

		}
		else if(wifi_station_get_connect_status() == STATION_NO_AP_FOUND)
		{

			INFO("STATION_NO_AP_FOUND\r\n");
			wifi_set_opmode(SOFTAP_MODE);
			WIFIServerMode();
			//wifi_station_connect();
			connect_sta=3;
			 os_sprintf(ack,"{\"cmd\":\"wifi_config_ok\",\"sid\":\"%s\",\"connect_sta\":%d}",dev_sid,connect_sta);
			WIFI_TCP_SendNews(ack,os_strlen(ack));
		}
		else if(wifi_station_get_connect_status() == STATION_CONNECT_FAIL)
		{

			INFO("STATION_CONNECT_FAIL\r\n");
			wifi_set_opmode(SOFTAP_MODE);
			WIFIServerMode();
			connect_sta=3;
			 os_sprintf(ack,"{\"cmd\":\"wifi_config_ok\",\"sid\":\"%s\",\"connect_sta\":%d}",dev_sid,connect_sta);
			WIFI_TCP_SendNews(ack,os_strlen(ack));
			//wifi_station_connect();

		}
		else
		{
			INFO("STATION_IDLE\r\n");
			os_timer_setfn(&WiFiLinker, (os_timer_func_t *)wifi_check_ip, NULL);
			os_timer_arm(&WiFiLinker, 500, 0);
		}
	}
	if(wifiStatus != lastWifiStatus){
		lastWifiStatus = wifiStatus;
		if(wifiCb)
			wifiCb(wifiStatus);
	}
}

void ICACHE_FLASH_ATTR WIFI_Connect(uint8_t* ssid, uint8_t* pass, WifiCallback cb)
{
	struct station_config stationConf;

	INFO("WIFI_INIT\r\n");
	wifi_set_opmode_current(0x03);

	//wifi_station_set_auto_connect(FALSE);
	wifiCb = cb;

	os_memset(&stationConf, 0, sizeof(struct station_config));

	os_sprintf(stationConf.ssid, "%s", ssid);
	os_sprintf(stationConf.password, "%s", pass);

	wifi_station_set_config(&stationConf);

	os_timer_disarm(&WiFiLinker);
	os_timer_setfn(&WiFiLinker, (os_timer_func_t *)wifi_check_ip, NULL);
	os_timer_arm(&WiFiLinker, 1000, 0);

	//wifi_station_set_auto_connect(TRUE);
	wifi_station_connect();
}


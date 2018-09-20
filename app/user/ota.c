/*
 * ota.c
 *
 *  Created on: 2018年9月18日
 *      Author: GP
 */
#include "ets_sys.h"
#include "osapi.h"
#include "user_interface.h"
#include "mem.h"
#include "espconn.h"
#include "driver/uart.h"
#include "ota.h"
#include "user_config.h"
#include "upgrade.h"


void ICACHE_FLASH_ATTR ota_finished_callback(void* arg)
{
	 struct upgrade_server_info *update = arg;
	    if (update->upgrade_flag == true){
	        os_printf("OTA  Success ! rebooting!\n");
	        system_upgrade_reboot();
	    }else{
	        os_printf("OTA failed!\n");
	    }
}


/**
 * server_ip: 服务器地址
 * port:服务器端口
 * path:文件路径
 */
void ICACHE_FLASH_ATTR ota_start_Upgrade(const char *server_ip, uint16_t port,const char *path) {
    const char* file;
    //获取系统的目前加载的是哪个bin文件
    uint8_t userBin = system_upgrade_userbin_check();

    switch (userBin) {

    //如果检查当前的是处于user1的加载文件，那么拉取的就是user2.bin
    case UPGRADE_FW_BIN1:
        file = "user2.4096.new.6.bin";
        break;

        //如果检查当前的是处于user2的加载文件，那么拉取的就是user1.bin
    case UPGRADE_FW_BIN2:
        file = "user1.4096.new.6.bin";
        break;

        //如果检查都不是，可能此刻不是OTA的bin固件
    default:
        os_printf("Fail read system_upgrade_userbin_check! \n\n");
        return;
    }

    struct upgrade_server_info* update =
            (struct upgrade_server_info *) os_zalloc(
                    sizeof(struct upgrade_server_info));
    update->pespconn = (struct espconn *) os_zalloc(sizeof(struct espconn));
    //设置服务器地址
    os_memcpy(update->ip, server_ip, 4);
    //设置服务器端口
    update->port = port;
    //设置OTA回调函数
    update->check_cb = ota_finished_callback;
    //设置定时回调时间
    update->check_times = 10000;
    //从 4M *1024 =4096申请内存
    update->url = (uint8 *)os_zalloc(4096);
#if 1
    //打印下請求地址
    os_printf("Http Server Address:%d.%d.%d.%d ,port: %d,filePath: %s,fileName: %s \n",
            IP2STR(update->ip), update->port, path, file);
#endif
    //拼接完整的 URL去请求服务器
    os_sprintf((char*) update->url, "GET /%s%s HTTP/1.1\r\n"
            "Host: "IPSTR":%d\r\n"
    "Connection: keep-alive\r\n"
    "\r\n", path, file, IP2STR(update->ip), update->port);

    if (system_upgrade_start(update) == false) {
        os_printf(" Could not start upgrade\n");
        //释放资源
        os_free(update->pespconn);
        os_free(update->url);
        os_free(update);
    } else {
        os_printf(" Upgrading...\n");
    }
}

/*
 * ota.c
 *
 *  Created on: 2018��9��18��
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
 * server_ip: ��������ַ
 * port:�������˿�
 * path:�ļ�·��
 */
void ICACHE_FLASH_ATTR ota_start_Upgrade(const char *server_ip, uint16_t port,const char *path) {
    const char* file;
    //��ȡϵͳ��Ŀǰ���ص����ĸ�bin�ļ�
    uint8_t userBin = system_upgrade_userbin_check();

    switch (userBin) {

    //�����鵱ǰ���Ǵ���user1�ļ����ļ�����ô��ȡ�ľ���user2.bin
    case UPGRADE_FW_BIN1:
        file = "user2.4096.new.6.bin";
        break;

        //�����鵱ǰ���Ǵ���user2�ļ����ļ�����ô��ȡ�ľ���user1.bin
    case UPGRADE_FW_BIN2:
        file = "user1.4096.new.6.bin";
        break;

        //�����鶼���ǣ����ܴ˿̲���OTA��bin�̼�
    default:
        os_printf("Fail read system_upgrade_userbin_check! \n\n");
        return;
    }

    struct upgrade_server_info* update =
            (struct upgrade_server_info *) os_zalloc(
                    sizeof(struct upgrade_server_info));
    update->pespconn = (struct espconn *) os_zalloc(sizeof(struct espconn));
    //���÷�������ַ
    os_memcpy(update->ip, server_ip, 4);
    //���÷������˿�
    update->port = port;
    //����OTA�ص�����
    update->check_cb = ota_finished_callback;
    //���ö�ʱ�ص�ʱ��
    update->check_times = 10000;
    //�� 4M *1024 =4096�����ڴ�
    update->url = (uint8 *)os_zalloc(4096);
#if 1
    //��ӡ��Ո���ַ
    os_printf("Http Server Address:%d.%d.%d.%d ,port: %d,filePath: %s,fileName: %s \n",
            IP2STR(update->ip), update->port, path, file);
#endif
    //ƴ�������� URLȥ���������
    os_sprintf((char*) update->url, "GET /%s%s HTTP/1.1\r\n"
            "Host: "IPSTR":%d\r\n"
    "Connection: keep-alive\r\n"
    "\r\n", path, file, IP2STR(update->ip), update->port);

    if (system_upgrade_start(update) == false) {
        os_printf(" Could not start upgrade\n");
        //�ͷ���Դ
        os_free(update->pespconn);
        os_free(update->url);
        os_free(update);
    } else {
        os_printf(" Upgrading...\n");
    }
}

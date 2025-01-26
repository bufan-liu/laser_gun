#include <stdio.h>
#include "sdkconfig.h"
#include "nvs_flash.h"

#include "gun_adc.h"
#include "gun_charge.h"
#include "gun_presskey.h"
#include "gun_ws2812.h"
#include "gun_infrared.h"
#include "gun_gatt_server.h"
#include "gun_ble_app.h"

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    //初始化蓝牙
    gun_ble_init();
    gun_ble_app_init();
    
    //初始化按键
    gun_presskey_init();
    gun_presskey_config();
    //使能adc
    gun_adc_init();
    //初始化充电task
    gun_charge_init();
    //初始化红外发送
     gun_ir_tx_init();
    //初始化红外接收
    gun_ir_rx_init();
    //初始化ws2812
    gun_ws2812_init();
}

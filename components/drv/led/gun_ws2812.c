#include "gun_ws2812.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <string.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/task.h"
#include "gun_charge.h"
#include "gun_gatt_server.h"
#include "msg_handle.h"

static const char *TAG = "gun_led_body";

#define WS2812_TH               0xE0//0xE0 F8
#define WS2812_TL               0x80//0x80 E0

#define WS2812_LED_BODY_NUMS     8
#define WS2812_DATA_BITS         8   //一个颜色8位
#define WS2812_DATA_BYTES        3   //一颗灯三个字节
#define WS2812_RESET_BYTE        30

#define SPI_MASTER_FREQ          4500000     //4.5MHz
#define GUN_ESP_LED_BODY         GPIO_NUM_5

#define WS2812_LED_BATTERY_NUMS  3
#define GUN_ESP_LED_BATTERY      GPIO_NUM_35

#define CONSTANT_ON              0x01
#define BREATHING                0X02
#define BLINKING                 0x03

uint8_t *g_ws2812_bit_buffer = NULL;
  
static ws2812_config_t led_body_config = {
    .spi_setting = {
        .clock_speed_hz = SPI_MASTER_FREQ,
        .dma_channel = SPI_DMA_CH_AUTO, 
        .host = SPI2_HOST,
        .max_transfer_sz = WS2812_LED_BODY_NUMS*WS2812_DATA_BYTES*WS2812_DATA_BITS,
        .mosi = GUN_ESP_LED_BODY,
    },
    .led_nums = WS2812_LED_BODY_NUMS
};

static ws2812_config_t led_battery_config = {
    .spi_setting = {
        .clock_speed_hz = SPI_MASTER_FREQ,
        .dma_channel = SPI_DMA_CH_AUTO, 
        .host = SPI3_HOST,
        .max_transfer_sz = WS2812_LED_BATTERY_NUMS*WS2812_DATA_BYTES*WS2812_DATA_BITS,
        .mosi = GUN_ESP_LED_BATTERY,
    },
    .led_nums = WS2812_LED_BATTERY_NUMS
};

#define LDE_BODY_CONTROL_CASE(ctrl, func)                                                          \
    if((data->led_ctr[0] == ctrl)) {                                                               \
        func((ws2812_select_color)(data->led_ctr[1]), led_body_config, led_body_config.spi_setting.spi_device_led_body, led_body_config.led_nums);  \
    }

static uint8_t grb_data[7][3] = {
    {0x64, 0x64, 0x64},     //0
    {0x64, 0x00, 0x00},     //1
    {0x00, 0x64, 0x00},     //2
    {0x00, 0x00, 0x64},     //3
    {0x64, 0x64, 0x00},     //4
    {0x00, 0x64, 0x64},     //5
    {0x64, 0x00, 0x64},     //6
};

void gun_ws2812_set_pixel(ws2812_color_t color, uint8_t* buffer, uint16_t index)
{
    for(uint8_t i = 0; i < WS2812_DATA_BITS; i++){
        buffer[index++] = (color.green & (1 << (7 - i))) ? WS2812_TH : WS2812_TL;
    }

    for(uint8_t i = 0; i < WS2812_DATA_BITS; i++){
        buffer[index++] = (color.red & (1 << (7 - i))) ? WS2812_TH : WS2812_TL;
    }

    for(uint8_t i = 0; i < WS2812_DATA_BITS; i++){
        buffer[index++] = (color.blue & (1 << (7 - i))) ? WS2812_TH : WS2812_TL;
    }
}

void gun_ws812_flush_data(ws2812_select_color color_index, ws2812_config_t ws2812_config, spi_device_handle_t spi_device, uint8_t led_num)
{
    uint16_t buff_size = led_num*WS2812_DATA_BYTES*WS2812_DATA_BITS + WS2812_RESET_BYTE; 
    uint16_t index_g = WS2812_RESET_BYTE;

    ws2812_config.grb.green = grb_data[color_index][0];
    ws2812_config.grb.red = grb_data[color_index][1];
    ws2812_config.grb.blue = grb_data[color_index][2];

    memset(g_ws2812_bit_buffer, 0x00, buff_size);

    for(uint8_t i = 0; i < ws2812_config.led_nums; i++){
        gun_ws2812_set_pixel(ws2812_config.grb, g_ws2812_bit_buffer, index_g);
        index_g += WS2812_DATA_BYTES*WS2812_DATA_BITS;
    }

    gun_spi_data_queue(spi_device, g_ws2812_bit_buffer, buff_size);
}

//周期6s 亮:120*25=3000ms 灭:120*25=3000ms
void gun_ws2812_set_breath(ws2812_select_color color_index, ws2812_config_t ws2812_config, spi_device_handle_t spi_device, uint8_t led_num)
{
    uint16_t buff_size = led_num*WS2812_DATA_BYTES*WS2812_DATA_BITS + WS2812_RESET_BYTE; 
    uint16_t index_g = WS2812_RESET_BYTE;
    uint8_t green = 0, red = 0, blue = 0;
    static bool breath_flag = false;
    static uint8_t brightness = 100;

    green = grb_data[color_index][0];
    red = grb_data[color_index][1];
    blue = grb_data[color_index][2];

    memset(g_ws2812_bit_buffer, 0x00, buff_size);
    
    if(!breath_flag){
        brightness -= 2;
        if(brightness <= 2)
            breath_flag = true;
    } else {
        brightness += 2;
        if(brightness >= 100){ 
            breath_flag = false;
            brightness = 100;
        }
    }

    for(uint8_t i = 0; i < ws2812_config.led_nums; i++){
        ws2812_config.grb.green = (brightness * green) / 100;
        ws2812_config.grb.red = (brightness * red) / 100;
        ws2812_config.grb.blue = (brightness * blue) / 100;
        // ESP_LOGI(TAG, "ws2812_config.grb.green = %d, ws2812_config.grb.red = %d, ws2812_config.grb.blue = %d", ws2812_config.grb.green, ws2812_config.grb.red, ws2812_config.grb.blue);
        gun_ws2812_set_pixel(ws2812_config.grb, g_ws2812_bit_buffer, index_g);
        index_g += WS2812_DATA_BYTES*WS2812_DATA_BITS;
    }

    gun_spi_data_queue(spi_device, g_ws2812_bit_buffer, buff_size);
    // index_g = WS2812_DATA_BYTES*WS2812_DATA_BITS;
}

void gun_ws2812_set_blink(ws2812_select_color color_index, ws2812_config_t ws2812_config, spi_device_handle_t spi_device, uint8_t led_num)
{
    uint16_t buff_size = led_num*WS2812_DATA_BYTES*WS2812_DATA_BITS + WS2812_RESET_BYTE; 
    uint16_t index_g = WS2812_RESET_BYTE;
    uint8_t green = 0, red = 0, blue = 0;
    static bool blink_flag = false;
    static uint8_t cnt = 0;

    green = grb_data[color_index][0];
    red = grb_data[color_index][1];
    blue = grb_data[color_index][2];

    memset(g_ws2812_bit_buffer, 0x00, buff_size);

    cnt++;
    if(cnt < 5) {
        return;
    }
    cnt = 0;
    blink_flag = !blink_flag;
    uint8_t brightness = blink_flag ? 0 : 100;

    for(uint8_t i = 0; i < (ws2812_config.led_nums - 1); i++) {
        ws2812_config.grb.green = (brightness * green) / 100;
        ws2812_config.grb.red = (brightness * red) /100;
        ws2812_config.grb.blue = (brightness * blue) /100;
        gun_ws2812_set_pixel(ws2812_config.grb, g_ws2812_bit_buffer, index_g);
        index_g += WS2812_DATA_BYTES*WS2812_DATA_BITS;
    }
    gun_spi_data_queue(spi_device, g_ws2812_bit_buffer, buff_size);
}

void ws2812_control_task(void* arg)
{
    app_to_gun_data_t *data;
    gun_charge_t charge_info;
    ws2812_effect_t ws2812_effect = WS2812_EFFECT_NONE;

    for(; ;)
    {
        data = get_app_to_gun_data();
        uint8_t ble_cnn_status = get_gun_ble_connect_status();

        if(data->start == 0x68 && ble_cnn_status == 1) {
            //连接
            ws2812_effect = WS2812_EFFECT_BLE_CNNENT;
        } else if(ble_cnn_status == 0) {
            //未连接
            ws2812_effect = WS2812_EFFECT_BLE_DISCONNECT;
        }

        gun_charge_info(&charge_info);
        if(charge_info.gun_bat_level > 1) {
           gun_ws812_flush_data(WS2812_WRITE, 
                                led_battery_config, 
                                led_battery_config.spi_setting.spi_device_led_battery,
                                (charge_info.gun_bat_level-1));
        } else {
            //电量等级为1格子时候
            gun_ws2812_set_blink(WS2812_WRITE,
                                 led_battery_config, 
                                 led_battery_config.spi_setting.spi_device_led_battery,
                                 led_battery_config.led_nums);
        }

        //测试
        // gun_ws2812_set_blink(WS2812_WRITE, 
        //                      led_battery_config, 
        //                      led_battery_config.spi_setting.spi_device_led_battery, 
        //                      led_battery_config.led_nums);
        // gun_ws2812_set_blink(WS2812_GREEN, 
        //                      led_body_config, 
        //                      led_body_config.spi_setting.spi_device_led_body, 
        //                      led_body_config.led_nums);

        switch(ws2812_effect)
        {
            case WS2812_EFFECT_BLE_CNNENT: 
                /*根据APP下发的数据改变等效及颜色*/
                LDE_BODY_CONTROL_CASE(CONSTANT_ON, gun_ws812_flush_data);
                LDE_BODY_CONTROL_CASE(BREATHING, gun_ws2812_set_breath);
                LDE_BODY_CONTROL_CASE(BLINKING, gun_ws2812_set_blink);
                break;
            case WS2812_EFFECT_BLE_DISCONNECT:
                 if((charge_info.gun_charing == 0) && (charge_info.gun_charge_stat == 1)) {
                    /*USB插入充电*/
                    gun_ws2812_set_breath(WS2812_GREEN, 
                                          led_body_config, 
                                          led_body_config.spi_setting.spi_device_led_body,
                                          led_body_config.led_nums);
                 } else {
                    /*USB插入但未充电 未连接未充电*/
                    gun_ws812_flush_data(WS2812_GREEN, 
                                         led_body_config, 
                                         led_body_config.spi_setting.spi_device_led_body,
                                         led_body_config.led_nums);
                 }
                 break; 
            case WS2812_EFFECT_NONE:
                 break;
        }

       vTaskDelay(150 / portTICK_PERIOD_MS);
    }
}

void gun_ws2812_init(void)
{
    gun_spi_init(&led_body_config.spi_setting);
    gun_spi_init(&led_battery_config.spi_setting);
    g_ws2812_bit_buffer = heap_caps_malloc(led_body_config.led_nums*WS2812_DATA_BYTES*WS2812_DATA_BITS + WS2812_RESET_BYTE, MALLOC_CAP_DMA);

    xTaskCreate(ws2812_control_task, "ws2812_control_task", 2560, NULL, 4, NULL);
}

#include "gun_spi.h"
#include "esp_log.h"
#include <string.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "gun_spi";

// spi_device_handle_t spi_device_led_body;
// spi_device_handle_t spi_device_led_battery;

// static spi_setting_t spi_setting = {
//     .dma_channel = SPI_DMA_CH_AUTO,
//     .host = SPI3_HOST,
//     .buscfg = {
//         .miso_io_num = -1,
//         .mosi_io_num = GUN_ESP_LED,
//         .sclk_io_num = GPIO_NUM_35,
//         .quadwp_io_num = -1,
//         .quadhd_io_num = -1,
//         .max_transfer_sz = LED_NUMS*24,
//     },
//     .devcfg = {
//         .clock_speed_hz = SPI_MASTER_FREQ,
//         .mode = 0,
//         .spics_io_num = -1,
//         .queue_size = 1,
//         .command_bits = 0,
// 		.address_bits = 0,
//     }
// };  

void gun_spi_init(spi_setting_t *spi_setting)
{
    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = spi_setting->mosi,
        .sclk_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = spi_setting->max_transfer_sz
    };         

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = spi_setting->clock_speed_hz,
        .command_bits = 0,
		.address_bits = 0,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1
    };      

    if(spi_setting->host == SPI2_HOST) {
        spi_bus_initialize(SPI2_HOST, &buscfg, spi_setting->dma_channel);
        spi_bus_add_device(SPI2_HOST, &devcfg, &spi_setting->spi_device_led_body); 
    } else if(spi_setting->host == SPI3_HOST) {
        spi_bus_initialize(SPI3_HOST, &buscfg, spi_setting->dma_channel);   
        spi_bus_add_device(SPI3_HOST, &devcfg, &spi_setting->spi_device_led_battery);
    }
}

void gun_spi_data_queue(spi_device_handle_t spi_device, void *data, int lenth)
{
    if(data == NULL || lenth == 0)
        return;

    spi_transaction_t trans;
    memset(&trans, 0, sizeof(trans));

    trans.length = lenth * 8;
    trans.tx_buffer = data;

    //spi_device_queue_trans(spi_device, &trans, 0);
    spi_device_polling_transmit(spi_device, &trans);
}

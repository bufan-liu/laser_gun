#include "msg_handle.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <string.h>
#include "driver/timer.h"
#include "esp_timer.h"

static const char *TAG = "msg_handle";

#define APP_TO_GUN_DATA_SIZE    sizeof(app_to_gun_data_t)
#define QUEUE_LENGTH            10 

QueueHandle_t msg_Queue;
static app_to_gun_data_t app_to_gun_data = {0x00};
static esp_timer_handle_t msg_timer;

app_to_gun_data_t *get_app_to_gun_data(void)
{
    return &app_to_gun_data; 
}

static void msg_handle_timer_cb(void *arg)
{
    app_to_gun_data_t data = {0x00};

    if(xQueueReceive(msg_Queue, &data, 0) == pdTRUE) {
        memcpy(&app_to_gun_data, &data, sizeof(app_to_gun_data_t));
    } else {
        memset(&app_to_gun_data, 0x00, sizeof(app_to_gun_data_t));
    }
}

void msg_handle_send(void *p_msg)
{
    if(p_msg == NULL) {
        ESP_LOGE(TAG, "p_msg is NULL");
        return;
    }
    xQueueSend(msg_Queue, p_msg, portMAX_DELAY);
}

void msg_handle_init(void)
{
    ESP_LOGI(TAG, "data_msg_handle init");

    msg_Queue = xQueueCreate(QUEUE_LENGTH, APP_TO_GUN_DATA_SIZE);
    if (msg_Queue == NULL) {
        ESP_LOGE(TAG, "data_msg_handle queue create failed");
    }

    ESP_LOGI(TAG, "data_msg_handle end");

    /*创建 esp_timer 定时器*/
    esp_timer_create_args_t timer_args = {
        .callback = &msg_handle_timer_cb, // 定时器回调函数
        .arg = NULL,                      // 用户参数
        .name = "msg_timer"               // 定时器名称
    };

    if (esp_timer_create(&timer_args, &msg_timer) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create timer");
        return;
    }

    // 启动定时器，每 500ms 回调一次
    if (esp_timer_start_periodic(msg_timer, 500000) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start timer");
        return;
    }
}

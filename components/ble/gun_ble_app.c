#include "gun_ble_app.h"
#include "msg_handle.h"
#include <string.h>
#include "gun_infrared.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gun_gatt_server.h"
#include "freertos/queue.h"

static const char *TAG = "gun_ble_app";

#define GUN_LA GPIO_NUM_36

#define CTR_LA 			0x01
#define CTR_SHAKE 		0x02
#define CTR_IR 			0x04
#define CTR_IR_1 		0x04
#define CTR_IR_2 		0x08

extern QueueHandle_t ble_recv_msg_Queue;
ble_notify_msg_t feedback_msg = {0};

static void gun_la_init(void)
{
	gpio_config_t la_config = {
		.mode = GPIO_MODE_OUTPUT,
		.intr_type = GPIO_INTR_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.pull_up_en = GPIO_PULLUP_DISABLE,
	};
	la_config.pin_bit_mask = 1ULL << GUN_LA;
	gpio_config(&la_config);

	gpio_set_level(GUN_LA, 0);
}

static void gun_la_on(void)
{
	gpio_set_level(GUN_LA, 1);
}

static void gun_la_off(void)
{
	gpio_set_level(GUN_LA, 0);
}

static void gun_la_ctr(uint8_t state)
{
	if (state)
	{
		gun_la_on();
		feedback_msg.data[1] |= CTR_LA;
	} else {
		gun_la_off();
		feedback_msg.data[1] &= ~CTR_LA;
	}
}

static void gun_infrared_ctr(int8_t state, uint8_t channel)
{
	if (state)
	{
		gun_ir_tx_task(channel);
		feedback_msg.data[1] |= CTR_IR;
	} else {
		feedback_msg.data[1] &= ~CTR_IR;
	}
}

static uint8_t gun_get_ctr_bit(uint8_t value, uint8_t mask)
{
	return value & mask;
}

void gun_ble_app_task(void *arg)
{
	app_to_gun_data_t *data;

	for (; ;)
	{
		data = get_app_to_gun_data();

		if (data->user_code != 0x00) {
			gun_la_ctr(gun_get_ctr_bit(data->output_ctr, CTR_LA));
			gun_infrared_ctr(gun_get_ctr_bit(data->output_ctr, CTR_IR_1), 0);
			gun_infrared_ctr(gun_get_ctr_bit(data->output_ctr, CTR_IR_2), 1);

			feedback_msg.handle_type = BLE_FEEDBACK_EVENT;
			feedback_msg.data[0] = 0x68;
			feedback_msg.data[2] = 0x00;
			feedback_msg.data[3] = 0x16;
			feedback_msg.len = sizeof(feedback_msg.data);
			/*将消息发送到队列*/
			xQueueSend(ble_recv_msg_Queue, &feedback_msg, portMAX_DELAY);
		}

		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}

void gun_ble_app_init(void)
{
	gun_la_init();

	xTaskCreate(gun_ble_app_task, "gun_ble_app_task", 2048, NULL, 3, NULL);
}

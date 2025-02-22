#ifndef _MSG_HANDLE_H_
#define _MSG_HANDLE_H_ 

#include <string.h>

#define BLE_NOTIFY_MSG_MAX_NUM       3      //后续根据需要添加

typedef enum{
    BLE_KEY_EVENT = 0,
    BLE_CONTROL_EVENT,
    BLE_INFRARED_EVENT,
}ble_type_t;

typedef struct{ 
    ble_type_t handle_type;
    uint8_t data[4];
    uint8_t len;
}ble_notify_msg_t;

typedef void (*ble_event_callback)(ble_notify_msg_t *msg_t);

typedef struct{
    uint8_t start;              //开始标志
    uint8_t ver_dev;            //版本&设备号
    uint8_t led_ctr[2];         //RGB灯控制
    uint8_t output_ctr;         //输出控制
    uint16_t user_code;         //用户码 战局信息
}__attribute__((packed))app_to_gun_data_t;          //取消结构体的对齐

void msg_handle_init(void);
void msg_handle_send(void *p_msg);
app_to_gun_data_t *get_app_to_gun_data(void);
void msg_handle_notify(ble_type_t type, void *data, uint8_t len);
void msg_handle_register(ble_type_t type, ble_event_callback event_cb);
void msg_handle_task(void);

#endif

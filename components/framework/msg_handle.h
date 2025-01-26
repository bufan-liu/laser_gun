#ifndef _MSG_HANDLE_H_
#define _MSG_HANDLE_H_ 

#include <string.h>

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

#endif

#ifndef _GUN_GATT_SERVER_H_
#define _GUN_GATT_SERVER_H_

#include <string.h>

void gun_ble_init(void);

/* Attributes State Machine */
enum
{
    IDX_SVC,

    IDX_CHAR_APP_TO_GUN,
    IDX_CHAR_VAL_APP_TO_GUN,

    IDX_CHAR_GUN_TO_APP_HEART,
    IDX_CHAR_VAL_GUN_TO_APP_HEART,
    IDX_CHAR_CFG_GUN_HEART,

    IDX_CHAR_GUN_TO_APP_KEY,
    IDX_CHAR_VAL_GUN_TO_APP_KEY,
    IDX_CHAR_CFG_GUN_KEY,

    IDX_CHAR_GUN_TO_APP_FEEDBACK,
    IDX_CHAR_VAL_GUN_TO_APP_FEEDBACK,
    IDX_CHAR_CFG_GUN_FEEDBACK,

    HRS_IDX_NB,
};

typedef enum{
    BLE_KEY_EVENT,
    BLE_FEEDBACK_EVENT,
}ble_type_t;

typedef struct{
    ble_type_t handle_type;
    uint8_t data[4];
    uint8_t len;
}ble_notify_msg_t;

uint8_t get_gun_ble_connect_status(void);

#endif

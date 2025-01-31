#ifndef _GUN_INFRARED_H_
#define _GUN_INFRARED_H_

#include <stdio.h>

void gun_ir_tx_init(void);
void gun_ir_tx_task(uint8_t channel);

void gun_ir_rx_init(void);
void gun_ir_rx_task(void *arg);

#endif

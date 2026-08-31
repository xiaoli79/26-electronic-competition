#ifndef KEY_H
#define KEY_H

#include "ti_msp_dl_config.h"

#define KEY_STATE_NONE  0x00U
#define KEY_STATE_KEY1  0x01U
#define KEY_STATE_KEY2  0x02U

uint8_t get_key_state(uint32_t key);
uint8_t get_key_states(void);
void encoder_quadrature_init(void);

/* Stable wiring: input pull-down, released=low, pressed to 3.3 V=high. */

/*
 * AB 正交四倍频有符号计数：正反转分别加减。
 * 变量名保留 counter_x_A 以兼容旧代码，实际已包含 A、B 两相全部有效边沿。
 */
extern volatile int32_t counter_1_A;
extern volatile int32_t counter_2_A;
extern volatile int32_t encoder_total_count_1;
extern volatile int32_t encoder_total_count_2;

#endif



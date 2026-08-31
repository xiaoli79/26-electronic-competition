#ifndef HUIDU_H
#define HUIDU_H

#include "ti_msp_dl_config.h"
#include "motor.h"

/*
 * 八路红外模块（板载 MCU，I2C 直读 1 字节位图）
 * SDA -> PA28 / I2C0_SDA
 * SCL -> PA31 / I2C0_SCL
 * huidu_value[0] 为最左路，huidu_value[7] 为最右路。
 */

/* 模块返回 1 表示黑线时保持 0；返回 0 表示黑线时改为 1。 */
#define EIGHT_ROAD_IR_ACTIVE_LOW          1U

/* 模块 bit0 为最左路时保持 0；bit7 为最左路时改为 1。 */
#define EIGHT_ROAD_IR_REVERSE_BITS        0U

/* 启动时通过调试串口输出模块初始化结果。 */
#define EIGHT_ROAD_IR_STARTUP_DEBUG       1U

/* 每 500 ms 输出一次原始位图；正常运行时建议保持关闭。 */
#define EIGHT_ROAD_IR_RAW_DEBUG_ENABLE    0U

extern uint8_t huidu_value[8];
extern float   target_speed_8[];

uint8_t Eight_Road_IR_Init(void);
uint8_t Eight_Road_IR_IsReady(void);
uint8_t Eight_Road_IR_GetRawValue(void);
uint8_t Eight_Road_IR_GetValueBits(void);
uint32_t Eight_Road_IR_GetErrorCount(void);
void    huidu_get_value(void);
void    adjust_motor(void);
void    adjust_motor_pwm(void);

#endif

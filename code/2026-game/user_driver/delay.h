#ifndef DELAY_H
#define DELAY_H
#include "ti_msp_dl_config.h"

/**
 * @brief 毫秒级阻塞延时
 * @param ms 延时毫秒数
 * @note  基于 CPUCLK_FREQ 计算周期数, 调用 delay_cycles() 实现
 */
void delay_ms(uint32_t ms);

/**
 * @brief 微秒级阻塞延时
 * @param us 延时微秒数
 * @note  基于 CPUCLK_FREQ 计算周期数, 调用 delay_cycles() 实现
 *        用于 CD4051 通道切换后的信号稳定等待 (50us)
 */
void delay_us(uint32_t us);

#endif // DELAY_H

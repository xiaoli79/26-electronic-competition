#include "delay.h"

/**
 * @brief 毫秒级阻塞延时
 * @note  CPU 主频 / 1000 = 每毫秒的时钟周期数
 *        例如 80MHz → 80,000 cycles/ms
 */
void delay_ms(uint32_t ms)
{
    uint32_t cycles = (CPUCLK_FREQ / 1000) * ms;    // ms → 总周期数
    delay_cycles(cycles);
}

/**
 * @brief 微秒级阻塞延时
 * @note  CPU 主频 / 1,000,000 = 每微秒的时钟周期数
 *        例如 80MHz → 80 cycles/us, 50us = 4000 cycles
 */
void delay_us(uint32_t us)
{
    uint32_t cycles = (CPUCLK_FREQ / 1000000) * us;  // us → 总周期数
    delay_cycles(cycles);
}

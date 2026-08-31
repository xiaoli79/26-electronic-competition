#ifndef NTB_TIME_H
#define NTB_TIME_H

#include "ti_msp_dl_config.h"

/**
 * @brief  获取系统运行毫秒时间戳
 * @return 从开机到当前的毫秒数 (int64_t)
 * @note   基于 TIMER2 (TIMG12) + MFCLK 4MHz ÷ 8 = 500kHz
 *         每 500 tick = 1ms, 周期 6000s 后自动回绕
 */
int64_t get_time_stamp_ms();

#endif

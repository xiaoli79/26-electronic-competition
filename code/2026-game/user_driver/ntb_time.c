#include "ti_msp_dl_config.h"

/**
 * @brief  获取系统运行毫秒时间戳
 * @note   NTB_INST: TIMG12, 时钟 MFCLK(4MHz) ÷ 8 = 500kHz
 *         每 500 个 tick = 1ms, 计数值 ÷ 500 = 毫秒数
 *         定时器周期 6000s, 溢出后自动回绕继续计数
 */
int64_t get_time_stamp_ms()
{
    int64_t counter_ = DL_Timer_getTimerCount(NTB_INST);  // 读硬件计数值
    return counter_ / 500;                                 // tick → 毫秒
}

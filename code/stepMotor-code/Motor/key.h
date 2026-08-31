#ifndef KEY_H
#define KEY_H

#include "ti_msp_dl_config.h"
#include <stdbool.h>

/* 按键另一端接 3.3 V，SysConfig 配置为内部下拉，所以按下为高电平。 */
typedef struct {
    GPIO_Regs *port;
    uint32_t pin;
    bool stable_level;
    uint8_t debounce_count;
    uint16_t hold_ticks;
    uint16_t next_repeat_tick;
    bool pressed_event;
    bool repeat_event;
    bool long_pressed_event;
    bool long_press_latched;
} KeyState;

void Key_Init(KeyState *key, GPIO_Regs *port, uint32_t pin);
/* 每个 APP_TICK 10 ms 节拍调用一次。 */
void Key_Scan(KeyState *key);
bool Key_TakePressed(KeyState *key);
bool Key_TakeRepeat(KeyState *key);
/* 稳定按下 1 秒只产生一次事件，松开后才允许下一次触发。 */
bool Key_TakeLongPressed(KeyState *key);

#endif

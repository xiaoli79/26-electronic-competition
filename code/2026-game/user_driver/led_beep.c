#include "led_beep.h"

// LED灯
void led_on()
{
    DL_GPIO_setPins(LED_BEEP_PORT, LED_BEEP_LED_PIN);
}
void led_off()
{
    DL_GPIO_clearPins(LED_BEEP_PORT, LED_BEEP_LED_PIN);
}


// 蜂鸣器
void beep_on()
{
    //低电平触发
    /* Buzzer hardware removed: compatibility no-op. */

}
void beep_off()
{
    //高电平关闭
    /* Buzzer hardware removed: compatibility no-op. */
}

#include "key.h"

#define KEY_DEBOUNCE_TICKS       3U
#define KEY_REPEAT_START_TICKS  50U
#define KEY_REPEAT_TICKS        10U
#define KEY_LONG_PRESS_TICKS    50U /* 50个10 ms节拍，即稳定按下0.5秒。 */

static bool Key_Read(const KeyState *key)
{
    return (DL_GPIO_readPins(key->port, key->pin) & key->pin) != 0U;
}

void Key_Init(KeyState *key, GPIO_Regs *port, uint32_t pin)
{
    key->port = port;
    key->pin = pin;
    key->stable_level = Key_Read(key);
    key->debounce_count = 0U;
    key->hold_ticks = 0U;
    key->next_repeat_tick = KEY_REPEAT_START_TICKS;
    key->pressed_event = false;
    key->repeat_event = false;
    key->long_pressed_event = false;
    key->long_press_latched = false;
}

void Key_Scan(KeyState *key)
{
    bool raw_level = Key_Read(key);

    if (raw_level == key->stable_level) {
        key->debounce_count = 0U;
    } else {
        key->debounce_count++;
        if (key->debounce_count >= KEY_DEBOUNCE_TICKS) {
            key->stable_level = raw_level;
            key->debounce_count = 0U;
            key->hold_ticks = 0U;
            key->next_repeat_tick = KEY_REPEAT_START_TICKS;
            if (raw_level) {
                key->pressed_event = true;
                key->long_press_latched = false;
            } else {
                key->long_press_latched = false;
            }
        }
    }

    if (!key->stable_level) {
        return;
    }

    if (key->hold_ticks < UINT16_MAX) {
        key->hold_ticks++;
    }

    if (key->hold_ticks >= key->next_repeat_tick) {
        key->repeat_event = true;
        if (key->next_repeat_tick <= (UINT16_MAX - KEY_REPEAT_TICKS)) {
            key->next_repeat_tick += KEY_REPEAT_TICKS;
        }
    }

    if (!key->long_press_latched &&
        (key->hold_ticks >= KEY_LONG_PRESS_TICKS)) {
        key->long_pressed_event = true;
        key->long_press_latched = true;
    }
}

bool Key_TakePressed(KeyState *key)
{
    bool event = key->pressed_event;
    key->pressed_event = false;
    return event;
}

bool Key_TakeRepeat(KeyState *key)
{
    bool event = key->repeat_event;
    key->repeat_event = false;
    return event;
}

bool Key_TakeLongPressed(KeyState *key)
{
    bool event = key->long_pressed_event;
    key->long_pressed_event = false;
    return event;
}

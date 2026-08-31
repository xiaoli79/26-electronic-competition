#include "key.h"
extern int status;

uint8_t get_key_state(uint32_t key) {
    uint32_t pins = DL_GPIO_readPins(KEY_PORT, key);

    /* Stable version: input pull-down, pressed key connects to 3.3 V. */
    return ((pins & key) != 0U) ? 1U : 0U;
}

/* Compatibility reader for the stable active-high key wiring. */
uint8_t get_key_states(void)
{
    uint32_t pins = DL_GPIO_readPins(KEY_PORT,
        KEY_KEY1_PIN | KEY_KEY2_PIN);

    if ((pins & KEY_KEY2_PIN) != 0U)
    {
        return KEY_STATE_KEY2;
    }
    if ((pins & KEY_KEY1_PIN) != 0U)
    {
        return KEY_STATE_KEY1;
    }
    return KEY_STATE_NONE;
}

volatile int32_t counter_1_A = 0;
volatile int32_t counter_2_A = 0;
volatile int32_t encoder_total_count_1 = 0;
volatile int32_t encoder_total_count_2 = 0;

static uint8_t encoder_last_state_1 = 0U;
static uint8_t encoder_last_state_2 = 0U;

/*
 * AB 正交状态查表：索引 = 上一次 AB 状态 << 2 | 当前 AB 状态。
 * 合法的相邻状态变化返回 +1 或 -1；两位同时跳变属于干扰或漏边沿，返回 0。
 */
static const int8_t encoder_transition_table[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0
};

static uint8_t encoder_read_state_1(void)
{
    uint8_t state = 0U;

    if ((DL_GPIO_readPins(DC_MOTOR_AA_PORT, DC_MOTOR_AA_PIN) &
         DC_MOTOR_AA_PIN) != 0U)
        state |= 0x02U;
    if ((DL_GPIO_readPins(DC_MOTOR_AB_PORT, DC_MOTOR_AB_PIN) &
         DC_MOTOR_AB_PIN) != 0U)
        state |= 0x01U;

    return state;
}

static uint8_t encoder_read_state_2(void)
{
    uint8_t state = 0U;

    if ((DL_GPIO_readPins(DC_MOTOR_BA_PORT, DC_MOTOR_BA_PIN) &
         DC_MOTOR_BA_PIN) != 0U)
        state |= 0x02U;
    if ((DL_GPIO_readPins(DC_MOTOR_BB_PORT, DC_MOTOR_BB_PIN) &
         DC_MOTOR_BB_PIN) != 0U)
        state |= 0x01U;

    return state;
}

static void encoder_update_1(void)
{
    uint8_t current_state = encoder_read_state_1();
    int8_t delta = encoder_transition_table[(encoder_last_state_1 << 2) |
                                             current_state];

    encoder_last_state_1 = current_state;
    counter_1_A += delta;
    encoder_total_count_1 += delta;
}

static void encoder_update_2(void)
{
    uint8_t current_state = encoder_read_state_2();
    int8_t delta = encoder_transition_table[(encoder_last_state_2 << 2) |
                                             current_state];

    encoder_last_state_2 = current_state;
    counter_2_A += delta;
    encoder_total_count_2 += delta;
}

void encoder_quadrature_init(void)
{
    encoder_last_state_1 = encoder_read_state_1();
    encoder_last_state_2 = encoder_read_state_2();
    counter_1_A = 0;
    counter_2_A = 0;
    encoder_total_count_1 = 0;
    encoder_total_count_2 = 0;

    DL_GPIO_clearInterruptStatus(DC_MOTOR_AA_PORT,
                                 DC_MOTOR_AA_PIN | DC_MOTOR_AB_PIN);
    DL_GPIO_clearInterruptStatus(DC_MOTOR_BA_PORT,
                                 DC_MOTOR_BA_PIN | DC_MOTOR_BB_PIN);
}

void GROUP1_IRQHandler()
{
    switch (DL_GPIO_getPendingInterrupt(GPIOB))
    {
    // case KEY_KEY9_IIDX:
    //     /* code */
    //     status = (status + 1) % 3;
    //     break;
    // case KEY_KEY10_IIDX:
    //     status = (status + 3 -1) % 3;
    //     /* code */
    //     break;
    case DC_MOTOR_BA_IIDX:
    case DC_MOTOR_BB_IIDX:
        encoder_update_2();
        break;
    
    default:
        break;
    }

    switch (DL_GPIO_getPendingInterrupt(GPIOA))
    {
    case DC_MOTOR_AA_IIDX:
    case DC_MOTOR_AB_IIDX:
        encoder_update_1();
        break;
    
    default:
        break;
    }

}




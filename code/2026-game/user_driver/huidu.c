#include "huidu.h"
#include "delay.h"
#include "uart.h"
#include <stdio.h>

uint8_t  huidu_value[8] = {0, 0, 0, 0, 0, 0, 0, 0};
float    target_speed_8[] = {100, 160, 220, 350, 350, 220, 160, 100};

extern float target_speed_1;
extern float target_speed_2;

#define EIGHT_ROAD_I2C_ADDRESS           0x09U
#define EIGHT_ROAD_I2C_TIMEOUT_LOOPS     20000U
#define EIGHT_ROAD_I2C_TRANSFER_EVENTS   \
    (DL_I2C_INTERRUPT_CONTROLLER_RX_DONE | \
     DL_I2C_INTERRUPT_CONTROLLER_NACK | \
     DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST)

static volatile uint8_t eight_road_ready = 0U;
static volatile uint8_t eight_road_raw_value = 0U;
static volatile uint8_t eight_road_value_bits = 0U;
static volatile uint32_t eight_road_error_count = 0U;

/* ============================================================
 * 原 CD4051 驱动（8 路灰度）
 * 已由板载 MCU 的 I2C 八路红外模块替代，代码仅保留作参考。
 * ============================================================ */
#if 0
static void _select_channel(uint8_t ch)
{
    uint32_t mask_b = 0;
    if (ch & 0x01) mask_b |= HUIDU_A0_PIN;
    if (ch & 0x02) mask_b |= HUIDU_A1_PIN;
    DL_GPIO_clearPins(HUIDU_A0_PORT, HUIDU_A0_PIN | HUIDU_A1_PIN);
    DL_GPIO_setPins(HUIDU_A0_PORT, mask_b);
    if (ch & 0x04) DL_GPIO_setPins(HUIDU_A2_PORT, HUIDU_A2_PIN);
    else           DL_GPIO_clearPins(HUIDU_A2_PORT, HUIDU_A2_PIN);
}

static uint8_t _read_OUT(void)
{
    return (DL_GPIO_readPins(HUIDU_OUT_PORT, HUIDU_OUT_PIN) & HUIDU_OUT_PIN) ? 1 : 0;
}

uint8_t get_gpio_state(GPIO_Regs *gpio_port, uint32_t gpio)
{
    uint32_t high_bits = DL_GPIO_readPins(gpio_port, gpio);
    if ((high_bits & gpio) != 0) return 1;
    else                         return 0;
}

void Grayscale_Sensor_Init(void)
{
    DL_GPIO_initDigitalOutput(HUIDU_A0_IOMUX);
    DL_GPIO_initDigitalOutput(HUIDU_A1_IOMUX);
    DL_GPIO_clearPins(HUIDU_A0_PORT, HUIDU_A0_PIN | HUIDU_A1_PIN);
    DL_GPIO_enableOutput(HUIDU_A0_PORT, HUIDU_A0_PIN | HUIDU_A1_PIN);

    DL_GPIO_initDigitalOutput(HUIDU_A2_IOMUX);
    DL_GPIO_clearPins(HUIDU_A2_PORT, HUIDU_A2_PIN);
    DL_GPIO_enableOutput(HUIDU_A2_PORT, HUIDU_A2_PIN);
}

//轮询八路，进而得出其值
void Grayscale_Sensor_Read_All(uint8_t *sv)
{
    for (uint8_t i = 0; i < 8; i++) {
        _select_channel(i);
        delay_us(50);
        sv[i] = _read_OUT();
    }
}
#endif

static void Eight_Road_I2C_ResetTransfer(void)
{
    DL_I2C_resetControllerTransfer(eight_road_INST);
    DL_I2C_flushControllerRXFIFO(eight_road_INST);
    DL_I2C_clearInterruptStatus(eight_road_INST,
        EIGHT_ROAD_I2C_TRANSFER_EVENTS);
}

static uint8_t Eight_Road_I2C_WaitIdle(void)
{
    uint32_t timeout = EIGHT_ROAD_I2C_TIMEOUT_LOOPS;

    while (timeout-- > 0U)
    {
        uint32_t status = DL_I2C_getControllerStatus(eight_road_INST);

        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U)
        {
            return 0U;
        }
        if ((status & DL_I2C_CONTROLLER_STATUS_IDLE) != 0U)
        {
            return 1U;
        }
    }
    return 0U;
}

/*
 * 从八路红外模块固定的 7 位地址 0x09 直接读取 1 字节。
 * 所有等待均有超时，不会无限阻塞。
 */
static uint8_t Eight_Road_I2C_ReadByte(uint8_t address, uint8_t *value)
{
    uint32_t timeout;

    if ((value == NULL) || !Eight_Road_I2C_WaitIdle())
    {
        Eight_Road_I2C_ResetTransfer();
        return 0U;
    }

    DL_I2C_flushControllerRXFIFO(eight_road_INST);
    DL_I2C_clearInterruptStatus(eight_road_INST,
        EIGHT_ROAD_I2C_TRANSFER_EVENTS);
    DL_I2C_startControllerTransfer(eight_road_INST, address,
        DL_I2C_CONTROLLER_DIRECTION_RX, 1U);

    /* MSPM0 I2C_ERR_13：传输启动后等待至少 3 个 I2C 功能时钟。 */
    delay_cycles(8U);

    timeout = EIGHT_ROAD_I2C_TIMEOUT_LOOPS;
    while (timeout-- > 0U)
    {
        uint32_t events = DL_I2C_getRawInterruptStatus(eight_road_INST,
            EIGHT_ROAD_I2C_TRANSFER_EVENTS);
        uint32_t status = DL_I2C_getControllerStatus(eight_road_INST);

        if ((events & (DL_I2C_INTERRUPT_CONTROLLER_NACK |
                       DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST)) != 0U ||
            (status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U)
        {
            Eight_Road_I2C_ResetTransfer();
            return 0U;
        }

        if ((events & DL_I2C_INTERRUPT_CONTROLLER_RX_DONE) != 0U)
        {
            if (DL_I2C_isControllerRXFIFOEmpty(eight_road_INST))
            {
                Eight_Road_I2C_ResetTransfer();
                return 0U;
            }

            *value = DL_I2C_receiveControllerData(eight_road_INST);
            DL_I2C_clearInterruptStatus(eight_road_INST,
                DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);
            return 1U;
        }
    }

    Eight_Road_I2C_ResetTransfer();
    return 0U;
}

static void Eight_Road_IR_ApplyRawValue(uint8_t raw_value)
{
    uint8_t normalized = raw_value;
    uint8_t value_bits = 0U;
    uint8_t i;

#if EIGHT_ROAD_IR_ACTIVE_LOW
    normalized = (uint8_t)~normalized;
#endif

    for (i = 0U; i < 8U; i++)
    {
#if EIGHT_ROAD_IR_REVERSE_BITS
        uint8_t bit_index = (uint8_t)(7U - i);
#else
        uint8_t bit_index = i;
#endif
        huidu_value[i] = (uint8_t)((normalized >> bit_index) & 0x01U);
        value_bits |= (uint8_t)(huidu_value[i] << i);
    }

    /* A single-byte snapshot keeps OLED reads consistent with the ISR update. */
    eight_road_value_bits = value_bits;
}

uint8_t Eight_Road_IR_Init(void)
{
    uint8_t raw_value;

    eight_road_ready = 0U;
    eight_road_raw_value = 0U;
    eight_road_value_bits = 0U;
    eight_road_error_count = 0U;

    Eight_Road_I2C_ResetTransfer();

    if (Eight_Road_I2C_ReadByte(EIGHT_ROAD_I2C_ADDRESS, &raw_value) != 0U)
    {
        eight_road_raw_value = raw_value;
        Eight_Road_IR_ApplyRawValue(raw_value);
        eight_road_ready = 1U;
    }
    else
    {
        eight_road_error_count++;
    }

#if EIGHT_ROAD_IR_STARTUP_DEBUG
    {
        char debug_string[72];

        if (eight_road_ready != 0U)
        {
            snprintf(debug_string, sizeof(debug_string),
                     "IR8: ready raw=0x%02X\r\n",
                     (unsigned int)eight_road_raw_value);
        }
        else
        {
            snprintf(debug_string, sizeof(debug_string),
                     "IR8: init failed\r\n");
        }
        UART_send_string(DEBUG_INST, debug_string);
    }
#endif

    return eight_road_ready;
}

uint8_t Eight_Road_IR_IsReady(void)
{
    return eight_road_ready;
}

uint8_t Eight_Road_IR_GetRawValue(void)
{
    return eight_road_raw_value;
}

uint8_t Eight_Road_IR_GetValueBits(void)
{
    return eight_road_value_bits;
}

uint32_t Eight_Road_IR_GetErrorCount(void)
{
    return eight_road_error_count;
}

/* 保留原接口，motor.c 中的巡线、横线检测和停车逻辑无需修改。 */
void huidu_get_value(void)
{
    uint8_t raw_value;

    if (eight_road_ready == 0U)
    {
        return;
    }

    if (Eight_Road_I2C_ReadByte(EIGHT_ROAD_I2C_ADDRESS, &raw_value) != 0U)
    {
        eight_road_raw_value = raw_value;
        Eight_Road_IR_ApplyRawValue(raw_value);
    }
    else
    {
        /* 瞬时 NACK/超时时保留上一次有效的 8 路结果。 */
        eight_road_error_count++;
    }
}

/*
 * target_speed_8[8] 对称差速表 (两边大, 中间小):
 *
 *   传感器位置: [0]最左 [1]左 [2]中左 [3]中 [4]中 [5]中右 [6]右 [7]最右
 *
 *   [0]=100 ← 最边缘, 慢轮最慢 = 差速最大 (大)
 *   [1]=160 ← 较边缘, 慢轮较慢
 *   [2]=220 ← 微偏,   慢轮微慢
 *   [3]=350 ← 中间,   直行 / 快轮基准 (小)
 *   [4]=350 ← 中间,   直行 / 快轮基准 (小)
 *   [5]=220 ← 微偏,   慢轮微慢
 *   [6]=160 ← 较边缘, 慢轮较慢
 *   [7]=100 ← 最边缘, 慢轮最慢 = 差速最大 (大)
 *
 *   线偏左 → 右转(左慢右快)  |  线偏右 → 左转(左快右慢)
 *
 *   快轮固定用 target_speed_8[3] = 350
 */


 //速度巡线，未被使用
// void adjust_motor(void)
// {
//     huidu_get_value();

//     // 全白: 丢线, 保持最低速直行
//     if (huidu_value[0] == 0 && huidu_value[1] == 0 && huidu_value[2] == 0 && huidu_value[3] == 0 &&
//         huidu_value[4] == 0 && huidu_value[5] == 0 && huidu_value[6] == 0 && huidu_value[7] == 0)
//     {
//         motor_set_direction(1, 1);
//         motor_set_direction(2, 1);
//         float min_speed = target_speed_1 < target_speed_2 ? target_speed_1 : target_speed_2;
//         target_speed_1 = min_speed;
//         target_speed_2 = min_speed;
//     }
//     // 全黑: 十字/终点, 停车
//     else if (huidu_value[0] == 1 && huidu_value[1] == 1 && huidu_value[2] == 1 && huidu_value[3] == 1 &&
//              huidu_value[4] == 1 && huidu_value[5] == 1 && huidu_value[6] == 1 && huidu_value[7] == 1)
//     {
//         target_speed_1 = 0;
//         target_speed_2 = 0;
//     }
//     // 仅中间 [3][4] 见线: 直行
//     else if (huidu_value[0] == 0 && huidu_value[1] == 0 && huidu_value[2] == 0 &&
//              huidu_value[3] == 1 && huidu_value[4] == 1 &&
//              huidu_value[5] == 0 && huidu_value[6] == 0 && huidu_value[7] == 0)
//     {
//         motor_set_direction(1, 1);
//         motor_set_direction(2, 1);
//         target_speed_1 = target_speed_8[3];   // 350
//         target_speed_2 = target_speed_8[3];   // 350
//     }
//     // ---- 线偏左 → 右转 (左慢, 右快) ----
//     // 最左 [0]=1: 最急右转
//     else if (huidu_value[0] == 1)
//     {
//         target_speed_1 = target_speed_8[0];   // 100 (左最慢)
//         target_speed_2 = target_speed_8[3];   // 350 (右快)
//     }
//     // 左 [1]=1: 较急右转
//     else if (huidu_value[1] == 1)
//     {
//         target_speed_1 = target_speed_8[1];   // 160 (左较慢)
//         target_speed_2 = target_speed_8[3];   // 350 (右快)
//     }
//     // 中左 [2]=1: 缓右转
//     else if (huidu_value[2] == 1)
//     {
//         target_speed_1 = target_speed_8[2];   // 220 (左微慢)
//         target_speed_2 = target_speed_8[3];   // 350 (右快)
//     }
//     // ---- 线偏右 → 左转 (左快, 右慢) ----
//     // 最右 [7]=1: 最急左转
//     else if (huidu_value[7] == 1)
//     {
//         target_speed_1 = target_speed_8[3];   // 350 (左快)
//         target_speed_2 = target_speed_8[7];   // 100 (右最慢)
//     }
//     // 右 [6]=1: 较急左转
//     else if (huidu_value[6] == 1)
//     {
//         target_speed_1 = target_speed_8[3];   // 350 (左快)
//         target_speed_2 = target_speed_8[6];   // 160 (右较慢)
//     }
//     // 中右 [5]=1: 缓左转
//     else if (huidu_value[5] == 1)
//     {
//         target_speed_1 = target_speed_8[3];   // 350 (左快)
//         target_speed_2 = target_speed_8[5];   // 220 (右微慢)
//     }
// }

/* ============================================================
 * adjust_motor_pwm — 8路对称PID差速调节 (motor.c 使用)
 *
 *   传感器: [0]最左 [1]左 [2]中左 [3]中 [4]中 [5]中右 [6]右 [7]最右
 *
 *   左右对称:
 *     [0] 极左 → pwm_diff 直接拉到 +STEP_0_7
 *     [1] 较左 → pwm_diff 累加 +STEP_1_6
 *     [2] 微左 → pwm_diff 累加 +STEP_2_5
 *     [5] 微右 → pwm_diff 累减 -STEP_2_5
 *     [6] 较右 → pwm_diff 累减 -STEP_1_6
 *     [7] 极右 → pwm_diff 直接拉到 -STEP_0_7
 *     [3]或[4] 中间 → pwm_diff 复位归零
 * ============================================================ */
int pwm_huidu_base = 1500;
int pwm_huidu_diff_half = 0;

#define STEP_0_7 850 // 极左/极右: 直接设最大差速
#define STEP_1_6 120   // 较左/较右: 快速累加
#define STEP_2_5 50   // 微左/微右: 慢速累加

// 8路灰度巡线差速调节函数(PWM增量式)
void adjust_motor_pwm()
{
    huidu_get_value();

    // 极左 [0]: 直接拉到最大
    if (huidu_value[0] == 1)
        pwm_huidu_diff_half = STEP_0_7;
    // 较左 [1]: 累加
    else if (huidu_value[1] == 1)
        pwm_huidu_diff_half += STEP_1_6;
    // 微左 [2]: 累加
    else if (huidu_value[2] == 1)
        pwm_huidu_diff_half += STEP_2_5;
    // 微右 [5]: 累减
    else if (huidu_value[5] == 1)
        pwm_huidu_diff_half -= STEP_2_5;
    // 较右 [6]: 累减
    else if (huidu_value[6] == 1)
        pwm_huidu_diff_half -= STEP_1_6;
    // 极右 [7]: 直接拉到最大
    else if (huidu_value[7] == 1)
        pwm_huidu_diff_half = -STEP_0_7;
    // 中间 [3] 或 [4] 见线: 复位归零
    else if (huidu_value[3] == 1 || huidu_value[4] == 1)
        pwm_huidu_diff_half = 0;
    // 全白: 保持上次差速不变
    // ==========================================
    // 这是之前的全白，直行逻辑
    else
        pwm_huidu_diff_half = 0;

    // 限幅
    if (pwm_huidu_diff_half > STEP_0_7)  pwm_huidu_diff_half = STEP_0_7;
    if (pwm_huidu_diff_half < -STEP_0_7) pwm_huidu_diff_half = -STEP_0_7;

    //电机设置占空比
    motor_set_duty(1, limit_duty(pwm_huidu_base - pwm_huidu_diff_half));
    motor_set_duty(2, limit_duty(pwm_huidu_base + pwm_huidu_diff_half));
}

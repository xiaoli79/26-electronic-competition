#include "motor.h"
#include "control_config.h"

/* 电机细分、固定速度和脉冲保护统一在control_config.h中调节。 */

static volatile uint32_t g_remaining_steps = 0U;
static volatile bool g_is_running = false;
static volatile int32_t g_position_steps = 0;
static volatile MotorDirection g_direction = MOTOR_DIR_FORWARD;

static int32_t Motor_Angle0p1DegToSteps(int16_t angle_0p1deg)
{
    int32_t scaled = (int32_t) angle_0p1deg * (int32_t) MOTOR_STEP_PER_REV;

    /* 对正负角度均执行对称的四舍五入。 */
    if (scaled >= 0) {
        return (scaled + 1800) / 3600;
    }
    return (scaled - 1800) / 3600;
}

static int16_t Motor_StepsToAngle0p1Deg(int32_t steps)
{
    int32_t scaled = steps * 3600;
    int32_t angle;

    if (scaled >= 0) {
        angle = (scaled + ((int32_t) MOTOR_STEP_PER_REV / 2)) /
                (int32_t) MOTOR_STEP_PER_REV;
    } else {
        angle = (scaled - ((int32_t) MOTOR_STEP_PER_REV / 2)) /
                (int32_t) MOTOR_STEP_PER_REV;
    }

    if (angle > INT16_MAX) {
        angle = INT16_MAX;
    } else if (angle < INT16_MIN) {
        angle = INT16_MIN;
    }
    return (int16_t) angle;
}

static void Motor_ConfigureFixedSpeed(void)
{
    uint32_t period;

    /* period = timer_clk / (固定角速度对应的STEP频率)。 */
    period = ((uint32_t) MOTOR_PWM_INST_CLK_FREQ * MOTOR_DEGREES_PER_REV) /
             ((uint32_t) MOTOR_FIXED_SPEED_DEG_S * MOTOR_STEP_PER_REV);
    if (period > 65535U) {
        period = 65535U;
    }
    if (period < MOTOR_MIN_TIMER_PERIOD) {
        period = MOTOR_MIN_TIMER_PERIOD;
    }

    DL_Timer_setLoadValue(MOTOR_PWM_INST, period);
    DL_Timer_setCaptureCompareValue(MOTOR_PWM_INST, period / 2U,
                                    GPIO_MOTOR_PWM_C0_IDX);
}

void Motor_Init(void)
{
    DL_GPIO_setPins(MOTOR_PORT, MOTOR_RST_PIN);
    DL_GPIO_setPins(MOTOR_PORT, MOTOR_SLP_PIN);
    DL_GPIO_setPins(MOTOR_PORT, MOTOR_DCY_PIN);
    Motor_SetDirection(MOTOR_DIR_FORWARD);
    Motor_ConfigureFixedSpeed();
    NVIC_EnableIRQ(MOTOR_PWM_INST_INT_IRQN);
}

void Motor_SetDirection(MotorDirection direction)
{
    g_direction = direction;

    if (direction == MOTOR_DIR_FORWARD) {
        DL_GPIO_clearPins(MOTOR_PORT, MOTOR_DIR_PIN);
    } else {
        DL_GPIO_setPins(MOTOR_PORT, MOTOR_DIR_PIN);
    }
}

void Motor_MoveAngle(uint32_t angle_deg)
{
    NVIC_DisableIRQ(MOTOR_PWM_INST_INT_IRQN);
    DL_Timer_stopCounter(MOTOR_PWM_INST);
    DL_Timer_clearInterruptStatus(MOTOR_PWM_INST,
                                  DL_TIMER_INTERRUPT_LOAD_EVENT);
    NVIC_ClearPendingIRQ(MOTOR_PWM_INST_INT_IRQN);
    g_remaining_steps = (angle_deg * MOTOR_STEP_PER_REV) / MOTOR_DEGREES_PER_REV;

    if (g_remaining_steps == 0U) {
        g_is_running = false;
        NVIC_EnableIRQ(MOTOR_PWM_INST_INT_IRQN);
        return;
    }

    g_is_running = true;
    NVIC_EnableIRQ(MOTOR_PWM_INST_INT_IRQN);
    DL_Timer_startCounter(MOTOR_PWM_INST);
}

void Motor_Stop(void)
{
    DL_Timer_stopCounter(MOTOR_PWM_INST);
    g_remaining_steps = 0U;
    g_is_running = false;
}

void Motor_SetLogicalZero(void)
{
    NVIC_DisableIRQ(MOTOR_PWM_INST_INT_IRQN);
    DL_Timer_stopCounter(MOTOR_PWM_INST);
    DL_Timer_clearInterruptStatus(MOTOR_PWM_INST,
                                  DL_TIMER_INTERRUPT_LOAD_EVENT);
    NVIC_ClearPendingIRQ(MOTOR_PWM_INST_INT_IRQN);
    g_remaining_steps = 0U;
    g_is_running = false;
    g_position_steps = 0;
    NVIC_EnableIRQ(MOTOR_PWM_INST_INT_IRQN);
}

void Motor_MoveToAngle0p1Deg(int16_t target_0p1deg)
{
    int32_t targetSteps = Motor_Angle0p1DegToSteps(target_0p1deg);
    int32_t deltaSteps;

    /* 中途更新目标时先屏蔽旧LOAD事件，再从实际STEP位置重新规划。 */
    NVIC_DisableIRQ(MOTOR_PWM_INST_INT_IRQN);
    DL_Timer_stopCounter(MOTOR_PWM_INST);
    DL_Timer_clearInterruptStatus(MOTOR_PWM_INST,
                                  DL_TIMER_INTERRUPT_LOAD_EVENT);
    NVIC_ClearPendingIRQ(MOTOR_PWM_INST_INT_IRQN);
    g_remaining_steps = 0U;
    g_is_running = false;
    deltaSteps = targetSteps - g_position_steps;
    if (deltaSteps == 0) {
        NVIC_EnableIRQ(MOTOR_PWM_INST_INT_IRQN);
        return;
    }

    if (deltaSteps > 0) {
        Motor_SetDirection(MOTOR_DIR_FORWARD);
        g_remaining_steps = (uint32_t) deltaSteps;
    } else {
        Motor_SetDirection(MOTOR_DIR_REVERSE);
        g_remaining_steps = (uint32_t) (-deltaSteps);
    }

    g_is_running = true;
    NVIC_EnableIRQ(MOTOR_PWM_INST_INT_IRQN);
    DL_Timer_startCounter(MOTOR_PWM_INST);
}

int16_t Motor_GetAngle0p1Deg(void)
{
    return Motor_StepsToAngle0p1Deg(g_position_steps);
}

MotorDirection Motor_GetDirection(void)
{
    return g_direction;
}

bool Motor_IsRunning(void)
{
    return g_is_running;
}

void MOTOR_PWM_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(MOTOR_PWM_INST)) {
    case DL_TIMER_IIDX_LOAD:
        /*
         * 每个 LOAD 中断对应一个 STEP 周期。原代码在剩余步数减到 0 后，
         * 还要多等一个周期才停止，导致命令角度多走一步。最后一步到达时
         * 立即停止计时器，确保脉冲数量与 g_remaining_steps 一一对应。
         */
        if (g_remaining_steps != 0U) {
            if (g_direction == MOTOR_DIR_FORWARD) {
                g_position_steps++;
            } else {
                g_position_steps--;
            }

            g_remaining_steps--;
            if (g_remaining_steps == 0U) {
                Motor_Stop();
            }
        }
        break;

    default:
        break;
    }
}

#include "ti_msp_dl_config.h"
#include "oled.h"
#include "motor.h"
#include "key.h"
#include "K230.H"
#include "control_config.h"

/* 所有实机调试参数统一放在 control_config.h，本文件只实现状态机。 */

typedef enum {
    TASK_1 = 1,
    TASK_2 = 2,
    TASK_3 = 3
} TaskId;

typedef enum {
    TASK2_IDLE = 0,
    TASK2_WAIT_CENTER,
    TASK2_MOVE_TO_POS5,
    TASK2_MOVE_TO_NEG5,
    TASK2_FINISHED,
    TASK2_VISION_LOST
} Task2State;

typedef enum {
    TASK3_IDLE = 0,
    TASK3_RUNNING,
    TASK3_VISION_LOST
} Task3State;

static KeyState g_task_key;
static KeyState g_is_start_key;
static KeyState g_motor_start_key;

static TaskId g_current_task = TASK_1;
static bool g_is_start = false;
static MotorDirection g_next_debug_direction = MOTOR_DIR_FORWARD;
static bool g_debug_move_active = false;
static bool g_last_motor_running = false;

static K230_Target_t g_k230_target = {false, 0};
static bool g_k230_new_target = false;
static uint16_t g_k230_sample_age_ticks = UINT16_MAX;
static bool g_header_display_pending = true;
static bool g_pos_display_pending = true;
static bool g_motor_display_pending = true;
static uint8_t g_pos_refresh_ticks = POS_REFRESH_TICKS;

static Task2State g_task2_state = TASK2_IDLE;
static int16_t g_task2_command_angle_0p1deg = 0;
static int16_t g_task2_last_position_0p1cm = 0;
static int32_t g_task2_velocity_0p1cm_s = 0;
static bool g_task2_have_last_position = false;
static uint16_t g_task2_sample_elapsed_ticks = 0U;
static uint16_t g_task2_control_ticks = 0U;
static uint16_t g_task2_vision_lost_ticks = 0U;
static bool g_task2_zero_initialized = false;
static int32_t g_task2_pid_integral_scaled = 0;
static uint16_t g_task2_stuck_ticks = 0U;
static uint16_t g_task2_breakaway_ticks = 0U;
static bool g_task2_breakaway_active = false;
static uint16_t g_task2_finish_stable_ticks = 0U;
static bool g_task2_settle_phase = false;

static Task3State g_task3_state = TASK3_IDLE;
static int16_t g_task3_command_angle_0p1deg = 0;
static int16_t g_task3_last_position_0p1cm = 0;
static int32_t g_task3_velocity_0p1cm_s = 0;
static bool g_task3_have_last_position = false;
static uint16_t g_task3_sample_elapsed_ticks = 0U;
static uint16_t g_task3_control_ticks = 0U;
static uint16_t g_task3_vision_lost_ticks = 0U;
static int32_t g_task3_pid_integral_scaled = 0;
static uint16_t g_task3_stuck_ticks = 0U;
static uint16_t g_task3_breakaway_ticks = 0U;
static bool g_task3_breakaway_active = false;
static uint16_t g_task3_hold_stable_ticks = 0U;
static bool g_task3_holding_center = false;

static void Task2_Start(void);
static void Task2_StopAndLevel(Task2State finalState);
static void Task2_Process(void);
static void Task3_Start(void);
static void Task3_Stop(Task3State finalState);
static void Task3_Process(void);

/* TIMG8 ISR 只累计节拍，GPIO 扫描、状态机和 OLED 刷新都留在主循环。 */
static volatile uint16_t g_app_tick_pending = 0U;

/* UART3 TX 中断正在从 K230 转发队列取数据时为 true。 */
static volatile bool g_pc_tx_active = false;

static void PcForward_Init(void)
{
    /* TX 中断只在有待发数据时动态使能，避免空 FIFO 产生无意义中断。 */
    DL_UART_disableInterrupt(DEBUG_INST, DL_UART_MAIN_INTERRUPT_TX);
    DL_UART_clearInterruptStatus(DEBUG_INST, DL_UART_MAIN_INTERRUPT_TX);
    NVIC_ClearPendingIRQ(DEBUG_INST_INT_IRQN);
    NVIC_EnableIRQ(DEBUG_INST_INT_IRQN);
}

/* 将已经排队的 K230 可读文本尽可能写入 UART3 发送 FIFO，不等待硬件。 */
static uint8_t PcForward_FillTxFifo(void)
{
    uint8_t data;
    uint8_t count = 0U;

    while (!DL_UART_isTXFIFOFull(DEBUG_INST) &&
           K230_ReadForwardByte(&data)) {
        DL_UART_transmitData(DEBUG_INST, data);
        count++;
    }

    return count;
}

static void PcForward_Start(void)
{
    uint8_t sent;

    if (g_pc_tx_active) {
        return;
    }

    /*
     * empty.syscfg 将 UART3 TX 阈值设为 EMPTY：硬件完整发出当前 FIFO
     * 内容后才请求下一次 TX 中断。先清除上一次完成留下的状态，再装入
     * 本轮原始 K230 数据，整个过程不等待串口硬件。
     */
    DL_UART_clearInterruptStatus(DEBUG_INST, DL_UART_MAIN_INTERRUPT_TX);
    sent = PcForward_FillTxFifo();
    if (sent == 0U) {
        return;
    }

    g_pc_tx_active = true;
    DL_UART_enableInterrupt(DEBUG_INST, DL_UART_MAIN_INTERRUPT_TX);
}

void DEBUG_INST_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(DEBUG_INST) != DL_UART_IIDX_TX) {
        return;
    }

    /* FIFO 已空：一次中断尽可能填满 FIFO，继续无阻塞地转发。 */
    if (PcForward_FillTxFifo() == 0U) {
        /* 队列清空后关掉 TX 中断，避免空 FIFO 重复进入中断。 */
        DL_UART_disableInterrupt(DEBUG_INST, DL_UART_MAIN_INTERRUPT_TX);
        g_pc_tx_active = false;
    }
}

void APP_TICK_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(APP_TICK_INST) == DL_TIMER_IIDX_ZERO) {
        if (g_app_tick_pending < UINT16_MAX) {
            g_app_tick_pending++;
        }
    }
}

static bool App_TakeTick(void)
{
    bool available = false;

    __disable_irq();
    if (g_app_tick_pending != 0U) {
        g_app_tick_pending--;
        available = true;
    }
    __enable_irq();

    return available;
}

static uint8_t DecimalDigits(uint16_t value)
{
    if (value >= 1000U) {
        return 4U;
    }
    if (value >= 100U) {
        return 3U;
    }
    if (value >= 10U) {
        return 2U;
    }
    return 1U;
}

static int32_t Abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t Clamp32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int16_t Task2_QuantizeAngle(int32_t angle_0p1deg)
{
    if (angle_0p1deg >= 0) {
        angle_0p1deg =
            ((angle_0p1deg + (TASK2_COMMAND_STEP_0P1DEG / 2)) /
             TASK2_COMMAND_STEP_0P1DEG) * TASK2_COMMAND_STEP_0P1DEG;
    } else {
        angle_0p1deg =
            -(((-angle_0p1deg + (TASK2_COMMAND_STEP_0P1DEG / 2)) /
               TASK2_COMMAND_STEP_0P1DEG) * TASK2_COMMAND_STEP_0P1DEG);
    }
    return (int16_t) angle_0p1deg;
}

static void ShowHeader(void)
{
    OLED_ClearArea(0U, 0U, 128U, 16U);
    OLED_ShowString(0U, 0U, (u8 *) "T:", 16U);
    OLED_ShowNum(16U, 0U, (uint32_t) g_current_task, 1U, 16U);
    OLED_ShowString(24U, 0U, (u8 *) ", is_start:", 16U);
    OLED_ShowNum(112U, 0U, g_is_start ? 1U : 0U, 1U, 16U);
    OLED_RefreshArea(0U, 0U, 128U, 16U);
}

static void ShowPosition(void)
{
    uint8_t x = 32U;

    OLED_ClearArea(0U, 16U, 128U, 32U);
    OLED_ShowString(0U, 24U, (u8 *) "Pos:", 16U);

    if (!g_k230_target.found) {
        OLED_ShowString(x, 24U, (u8 *) "LOST", 16U);
    } else {
        int32_t position = g_k230_target.position_0p1cm;
        uint16_t magnitude;
        uint16_t wholeCm;
        uint8_t digits;

        if (position < 0) {
            OLED_ShowString(x, 24U, (u8 *) "-", 16U);
            x += 8U;
            position = -position;
        }

        magnitude = (uint16_t) position;
        wholeCm = magnitude / 10U;
        digits = DecimalDigits(wholeCm);
        OLED_ShowNum(x, 24U, wholeCm, digits, 16U);
        x = (uint8_t) (x + (digits * 8U));
        OLED_ShowString(x, 24U, (u8 *) ".", 16U);
        x += 8U;
        OLED_ShowNum(x, 24U, magnitude % 10U, 1U, 16U);
        x += 8U;
        OLED_ShowString(x, 24U, (u8 *) "cm", 16U);
    }

    /* 文字只占 y=24..39，局部刷新两页，避免 50 ms 更新占满 I2C。 */
    OLED_RefreshArea(0U, 24U, 128U, 16U);
}

static void ShowMotorParameters(void)
{
    uint32_t displayedAngle = DEBUG_MOTOR_ANGLE_DEG;

    if (g_current_task == TASK_2) {
        int32_t magnitude = Abs32(g_task2_command_angle_0p1deg);
        displayedAngle = (uint32_t) ((magnitude + 5) / 10);
    } else if (g_current_task == TASK_3) {
        int32_t magnitude = Abs32(g_task3_command_angle_0p1deg);
        displayedAngle = (uint32_t) ((magnitude + 5) / 10);
    }

    OLED_ClearArea(0U, 48U, 128U, 16U);
    OLED_ShowString(0U, 48U, (u8 *) "dir:", 12U);
    OLED_ShowString(24U, 48U,
                    Motor_GetDirection() == MOTOR_DIR_FORWARD ?
                    (u8 *) "FWD" : (u8 *) "REV", 12U);
    OLED_ShowString(42U, 48U, (u8 *) " v:", 12U);
    OLED_ShowNum(60U, 48U, MOTOR_FIXED_SPEED_DEG_S, 2U, 12U);
    OLED_ShowString(72U, 48U, (u8 *) " angle:", 12U);
    OLED_ShowNum(114U, 48U, displayedAngle, 2U, 12U);
    OLED_RefreshArea(0U, 48U, 128U, 16U);
}

static void ShowInitialScreen(void)
{
    ShowHeader();
    ShowPosition();
    ShowMotorParameters();
    g_header_display_pending = false;
    g_pos_display_pending = false;
    g_motor_display_pending = false;
    g_pos_refresh_ticks = 0U;
}

static void Task2_ResetControlState(void)
{
    g_task2_command_angle_0p1deg = 0;
    g_task2_last_position_0p1cm = 0;
    g_task2_velocity_0p1cm_s = 0;
    g_task2_have_last_position = false;
    g_task2_sample_elapsed_ticks = 0U;
    g_task2_control_ticks = 0U;
    g_task2_vision_lost_ticks = 0U;
    g_task2_zero_initialized = false;
    g_task2_pid_integral_scaled = 0;
    g_task2_stuck_ticks = 0U;
    g_task2_breakaway_ticks = 0U;
    g_task2_breakaway_active = false;
    g_task2_finish_stable_ticks = 0U;
    g_task2_settle_phase = false;
    g_k230_new_target = false;
}

static void Task2_Start(void)
{
    /*
     * 按下is_start只进入等待状态，不能提前建立电机零点。
     * 只有K230位置进入中心点±1.0 cm后，才允许初始化并执行PID任务。
     */
    Motor_Stop();
    Task2_ResetControlState();
    /* 给K230最多3000 ms送来第一帧新数据，旧位置不会被用于控制。 */
    g_k230_sample_age_ticks = 0U;
    g_task2_state = TASK2_WAIT_CENTER;
    g_is_start = true;
    g_header_display_pending = true;
    g_motor_display_pending = true;
}

static void Task2_StopAndLevel(Task2State finalState)
{
    /* 只有本次任务已建立逻辑零点，才允许按软件位置返回0度。 */
    g_task2_state = finalState;
    g_task2_command_angle_0p1deg = 0;
    g_task2_pid_integral_scaled = 0;
    g_task2_stuck_ticks = 0U;
    g_task2_breakaway_ticks = 0U;
    g_task2_breakaway_active = false;
    g_task2_finish_stable_ticks = 0U;
    g_task2_settle_phase = false;
    g_k230_new_target = false;
    if (g_task2_zero_initialized) {
        Motor_MoveToAngle0p1Deg(0);
    } else {
        Motor_Stop();
    }
    g_task2_zero_initialized = false;
    g_is_start = false;
    g_header_display_pending = true;
    g_motor_display_pending = true;
}

static void Task2_ResetPid(void)
{
    g_task2_pid_integral_scaled = 0;
}

static void Task2_ResetBreakaway(void)
{
    g_task2_stuck_ticks = 0U;
    g_task2_breakaway_ticks = 0U;
    g_task2_breakaway_active = false;
}

static void Task2_UpdateVelocity(uint16_t elapsedTicks)
{
    int16_t position = g_k230_target.position_0p1cm;

    if (!g_task2_have_last_position) {
        g_task2_velocity_0p1cm_s = 0;
        g_task2_have_last_position = true;
    } else {
        int32_t delta = (int32_t) position - g_task2_last_position_0p1cm;
        int32_t rawVelocity;

        if (elapsedTicks == 0U) {
            elapsedTicks = 1U;
        }
        rawVelocity = (delta * 100) / (int32_t) elapsedTicks;
        rawVelocity = Clamp32(rawVelocity, -1000, 1000);
        /* 50%旧速度+50%新速度，让D项更快感知小球速度并提前制动。 */
        g_task2_velocity_0p1cm_s =
            (g_task2_velocity_0p1cm_s + rawVelocity) / 2;
    }

    g_task2_last_position_0p1cm = position;
}

static int16_t Task2_ShapeCommand(int32_t command, int32_t limit)
{
    int32_t minimumSlew;
    int32_t maximumSlew;

    command = Clamp32(command, -limit, limit);

    /* 单次目标变化上限由control_config.h统一配置。 */
    minimumSlew = (int32_t) g_task2_command_angle_0p1deg -
                  TASK2_COMMAND_SLEW_0P1DEG;
    maximumSlew = (int32_t) g_task2_command_angle_0p1deg +
                  TASK2_COMMAND_SLEW_0P1DEG;
    command = Clamp32(command, minimumSlew, maximumSlew);
    command = Task2_QuantizeAngle(command);
    command = Clamp32(command, -limit, limit);
    return (int16_t) command;
}

static int16_t Task2_CalculatePidCommand(int16_t targetPosition_0p1cm,
                                         uint16_t elapsedTicks,
                                         int32_t kp_x100,
                                         int32_t kd_x100)
{
    int32_t error =
        (int32_t) targetPosition_0p1cm - g_k230_target.position_0p1cm;
    int32_t absoluteError = Abs32(error);
    int32_t absoluteVelocity = Abs32(g_task2_velocity_0p1cm_s);
    int32_t proportionalTerm;
    int32_t integralTerm;
    int32_t derivativeTerm;
    int32_t command;
    int32_t candidateIntegralTerm;
    int32_t candidateCommand;
    int64_t integralLimitScaled =
        (int64_t) TASK2_PID_INTEGRAL_LIMIT_0P1DEG * 10000;
    int64_t candidateIntegral;

    if (elapsedTicks == 0U) {
        elapsedTicks = 1U;
    }

    /*
     * 静摩擦补偿只在“误差仍大且连续近似静止”时触发。补偿期间不累积
     * 积分；小球开始移动、误差变小或达到最大补偿时间后立即回到PID。
     */
    if (g_task2_breakaway_active) {
        uint32_t accumulated =
            (uint32_t) g_task2_breakaway_ticks + elapsedTicks;
        g_task2_breakaway_ticks =
            (accumulated > UINT16_MAX) ? UINT16_MAX :
                                         (uint16_t) accumulated;

        if ((absoluteVelocity >= TASK2_MOVING_SPEED_0P1CM_S) ||
            (absoluteError < TASK2_STUCK_ERROR_0P1CM) ||
            (g_task2_breakaway_ticks >= TASK2_BREAKAWAY_MAX_TICKS)) {
            Task2_ResetBreakaway();
        } else {
            command = (error >= 0) ? TASK2_BREAKAWAY_TILT_0P1DEG :
                                     -TASK2_BREAKAWAY_TILT_0P1DEG;
            return Task2_ShapeCommand(
                command, TASK2_BREAKAWAY_TILT_0P1DEG);
        }
    }

    if ((absoluteError >= TASK2_STUCK_ERROR_0P1CM) &&
        (absoluteVelocity <= TASK2_STUCK_SPEED_0P1CM_S)) {
        uint32_t accumulated =
            (uint32_t) g_task2_stuck_ticks + elapsedTicks;
        g_task2_stuck_ticks =
            (accumulated >= TASK2_STUCK_TIME_TICKS) ?
            TASK2_STUCK_TIME_TICKS : (uint16_t) accumulated;

        if (g_task2_stuck_ticks >= TASK2_STUCK_TIME_TICKS) {
            g_task2_breakaway_active = true;
            g_task2_breakaway_ticks = 0U;
            g_task2_stuck_ticks = 0U;
            Task2_ResetPid();
            command = (error >= 0) ? TASK2_BREAKAWAY_TILT_0P1DEG :
                                     -TASK2_BREAKAWAY_TILT_0P1DEG;
            return Task2_ShapeCommand(
                command, TASK2_BREAKAWAY_TILT_0P1DEG);
        }
    } else {
        g_task2_stuck_ticks = 0U;
    }

    proportionalTerm = (kp_x100 * error) / 100;
    derivativeTerm =
        (kd_x100 * g_task2_velocity_0p1cm_s) / 100;

    /*
     * 条件积分抗饱和：如果加入本次积分后会让已经饱和的输出沿误差方向
     * 继续增大，就拒绝这次积分；反方向积分仍允许，用于快速解除饱和。
     */
    candidateIntegral =
        (int64_t) g_task2_pid_integral_scaled +
        ((int64_t) TASK2_PID_KI_X100 * error * elapsedTicks);
    if (candidateIntegral > integralLimitScaled) {
        candidateIntegral = integralLimitScaled;
    } else if (candidateIntegral < -integralLimitScaled) {
        candidateIntegral = -integralLimitScaled;
    }

    candidateIntegralTerm = (int32_t) candidateIntegral / 10000;
    candidateCommand =
        proportionalTerm + candidateIntegralTerm - derivativeTerm;
    if (!(((candidateCommand >= TASK2_PID_TILT_LIMIT_0P1DEG) &&
           (error > 0)) ||
          ((candidateCommand <= -TASK2_PID_TILT_LIMIT_0P1DEG) &&
           (error < 0)))) {
        g_task2_pid_integral_scaled = (int32_t) candidateIntegral;
    }

    integralTerm = g_task2_pid_integral_scaled / 10000;
    command = proportionalTerm + integralTerm - derivativeTerm;

    return Task2_ShapeCommand(command, TASK2_PID_TILT_LIMIT_0P1DEG);
}

static int16_t Task2_EnforceNegativeMinimumDrive(int16_t command_0p1deg)
{
    /*
     * +5 cm折返后、第一次到达-4.5 cm之前，如果小球负向速度已经很低，
     * 不允许D项把REV推进角抵消得小于1.5度。仍通过ShapeCommand处理，
     * 因而不会绕过目标角变化率和最大倾角保护。
     */
    if (!g_task2_settle_phase &&
        (g_k230_target.position_0p1cm >
         TASK2_SETTLE_ENTRY_POSITION_0P1CM) &&
        (Abs32(g_task2_velocity_0p1cm_s) <=
         TASK2_NEG_MIN_SPEED_0P1CM_S) &&
        (command_0p1deg > -TASK2_NEG_MIN_DRIVE_0P1DEG)) {
        return Task2_ShapeCommand(-TASK2_NEG_MIN_DRIVE_0P1DEG,
                                  TASK2_PID_TILT_LIMIT_0P1DEG);
    }

    return command_0p1deg;
}

static int16_t Task2_EnforcePositiveMinimumDrive(int16_t position_0p1cm,
                                                  int16_t command_0p1deg)
{
    /*
     * 任务2第一段是“通过+5 cm后立即折返”，而非在+5 cm处稳住。
     * 因此在尚未达到计划折返点前，不允许小误差、量化或速度项把命令
     * 缩小为0或REV；保持最低FWD推进量直到切入REV阶段。
     */
    if ((position_0p1cm <
         (TASK2_TARGET_POS5_0P1CM - TASK2_POS5_TURN_LEAD_0P1CM)) &&
        (command_0p1deg < TASK2_POS_MIN_DRIVE_0P1DEG)) {
        return Task2_ShapeCommand(TASK2_POS_MIN_DRIVE_0P1DEG,
                                  TASK2_PID_TILT_LIMIT_0P1DEG);
    }

    return command_0p1deg;
}

static void Task2_ApplyCommand(int16_t command_0p1deg)
{
    if (command_0p1deg == g_task2_command_angle_0p1deg) {
        return;
    }

    g_task2_command_angle_0p1deg = command_0p1deg;
    Motor_MoveToAngle0p1Deg(command_0p1deg);
    g_motor_display_pending = true;
}

static void Task2_ProcessMeasurement(uint16_t elapsedTicks)
{
    int32_t position = g_k230_target.position_0p1cm;
    int32_t error;
    int16_t command;

    Task2_UpdateVelocity(elapsedTicks);

    switch (g_task2_state) {
    case TASK2_WAIT_CENTER:
        if (Abs32(position - TASK2_START_POSITION_0P1CM) <=
            TASK2_START_ERROR_0P1CM) {
            /*
             * 小球已进入中心点-1.0～+1.0 cm：此刻把当前电机位置
             * 定义为0度，不再要求位置必须精确等于0.0 cm。
             */
            Motor_SetLogicalZero();
            g_task2_zero_initialized = true;
            g_task2_command_angle_0p1deg = 0;
            g_task2_velocity_0p1cm_s = 0;
            g_task2_last_position_0p1cm = (int16_t) position;
            g_task2_have_last_position = true;
            Task2_ResetPid();
            Task2_ResetBreakaway();
            g_task2_state = TASK2_MOVE_TO_POS5;

            command = Task2_CalculatePidCommand(
                TASK2_TARGET_POS5_0P1CM, elapsedTicks,
                TASK2_POS_TRANSFER_KP_X100,
                TASK2_POS_TRANSFER_KD_X100);
            Task2_ApplyCommand(command);
        }
        break;

    case TASK2_MOVE_TO_POS5:
        if (position >=
            (TASK2_TARGET_POS5_0P1CM - TASK2_POS5_TURN_LEAD_0P1CM)) {
            /*
             * 第一段不在+5 cm附近稳定。达到“+5 cm减去提前折返量”
             * 后立即进入去-5 cm的负向PID阶段，剩余距离由小球当前的
             * 正向动能走过，以减小实际越过+5 cm后的惯性超调。
             */
            Task2_ResetPid();
            Task2_ResetBreakaway();
            g_task2_finish_stable_ticks = 0U;
            g_task2_state = TASK2_MOVE_TO_NEG5;
            command = Task2_CalculatePidCommand(
                TASK2_TARGET_NEG5_0P1CM, elapsedTicks,
                TASK2_NEG_TRANSFER_KP_X100,
                TASK2_NEG_TRANSFER_KD_X100);
            command = Task2_EnforceNegativeMinimumDrive(command);
        } else {
            command = Task2_CalculatePidCommand(
                TASK2_TARGET_POS5_0P1CM, elapsedTicks,
                TASK2_POS_TRANSFER_KP_X100,
                TASK2_POS_TRANSFER_KD_X100);
            command = Task2_EnforcePositiveMinimumDrive(
                (int16_t) position, command);
        }
        Task2_ApplyCommand(command);
        break;

    case TASK2_MOVE_TO_NEG5:
        error = (int32_t) TASK2_TARGET_NEG5_0P1CM - position;
        if (!g_task2_settle_phase &&
            (position <= TASK2_SETTLE_ENTRY_POSITION_0P1CM)) {
            /*
             * 第一次到达-4.5 cm即锁定终点稳定阶段。即使之后短暂回到
             * -4.5 cm右侧，也不再切回较激进的负向参数，避免边界抖动。
             * 保留负向传输阶段最多1度的积分偏置，避免切换瞬间失去推进力。
             */
            g_task2_settle_phase = true;
            Task2_ResetBreakaway();
        }

        if ((Abs32(error) <= TASK2_FINISH_ERROR_0P1CM) &&
            (Abs32(g_task2_velocity_0p1cm_s) <=
             TASK2_FINISH_SPEED_0P1CM_S)) {
            uint32_t accumulated =
                (uint32_t) g_task2_finish_stable_ticks + elapsedTicks;
            g_task2_finish_stable_ticks =
                (accumulated >= TASK2_FINISH_STABLE_TICKS) ?
                TASK2_FINISH_STABLE_TICKS : (uint16_t) accumulated;
        } else {
            g_task2_finish_stable_ticks = 0U;
        }

        if (g_task2_finish_stable_ticks >= TASK2_FINISH_STABLE_TICKS) {
            /*
             * 小球已在-5±0.5 cm内低速稳定200 ms。立即停止STEP并记录
             * 此刻的实际电机角度，让驱动器保持该平衡角，不返回0度。
             */
            Motor_Stop();
            g_task2_command_angle_0p1deg = Motor_GetAngle0p1Deg();
            g_task2_state = TASK2_FINISHED;
            Task2_ResetPid();
            Task2_ResetBreakaway();
            g_is_start = false;
            g_header_display_pending = true;
            g_motor_display_pending = true;
            break;
        }
        if (g_task2_settle_phase) {
            command = Task2_CalculatePidCommand(
                TASK2_TARGET_NEG5_0P1CM, elapsedTicks,
                TASK2_SETTLE_KP_X100,
                TASK2_SETTLE_KD_X100);
        } else {
            command = Task2_CalculatePidCommand(
                TASK2_TARGET_NEG5_0P1CM, elapsedTicks,
                TASK2_NEG_TRANSFER_KP_X100,
                TASK2_NEG_TRANSFER_KD_X100);
            command = Task2_EnforceNegativeMinimumDrive(command);
        }
        Task2_ApplyCommand(command);
        break;

    case TASK2_FINISHED:
        /* 任务已达标，步进电机保持最后的平衡角，不再执行位置PID。 */
        break;

    case TASK2_IDLE:
    case TASK2_VISION_LOST:
    default:
        break;
    }
}

static void SwitchTask(void)
{
    if (g_current_task == TASK_2) {
        Task2_StopAndLevel(TASK2_IDLE);
    } else if (g_current_task == TASK_3) {
        Task3_Stop(TASK3_IDLE);
    } else {
        Motor_Stop();
    }
    g_debug_move_active = false;
    g_last_motor_running = false;
    g_next_debug_direction = MOTOR_DIR_FORWARD;
    g_is_start = false;

    if (g_current_task == TASK_3) {
        g_current_task = TASK_1;
    } else {
        g_current_task = (TaskId) ((uint8_t) g_current_task + 1U);
    }

    g_header_display_pending = true;
    g_motor_display_pending = true;
}

static void HandleIsStartKey(void)
{
    if (g_current_task == TASK_2) {
        if (g_is_start) {
            Task2_StopAndLevel(TASK2_IDLE);
        } else {
            Task2_Start();
        }
    } else if (g_current_task == TASK_3) {
        if (g_is_start) {
            Task3_Stop(TASK3_IDLE);
        } else {
            Task3_Start();
        }
    } else {
        g_is_start = !g_is_start;
        g_header_display_pending = true;
    }
}

static void HandleMotorStartKey(void)
{
    /* 任务2、任务3由K230视觉PID独占电机；按键3只控制任务1。 */
    if (g_current_task != TASK_1) {
        return;
    }

    /* 运行中再次按下按键3：打断剩余动作并立即反向运行一整段。 */
    if (Motor_IsRunning()) {
        MotorDirection reverseDirection =
            (Motor_GetDirection() == MOTOR_DIR_FORWARD) ?
            MOTOR_DIR_REVERSE : MOTOR_DIR_FORWARD;

        Motor_Stop();
        Motor_SetDirection(reverseDirection);
        g_next_debug_direction = reverseDirection;
        Motor_MoveAngle(DEBUG_MOTOR_ANGLE_DEG);

        g_debug_move_active = Motor_IsRunning();
        g_last_motor_running = g_debug_move_active;
        g_motor_display_pending = true;
        return;
    }

    Motor_SetDirection(g_next_debug_direction);
    Motor_MoveAngle(DEBUG_MOTOR_ANGLE_DEG);
    if (Motor_IsRunning()) {
        g_debug_move_active = true;
        g_motor_display_pending = true;
    }
}

static void ProcessMotorCompletion(void)
{
    bool running = Motor_IsRunning();

    if (g_debug_move_active && g_last_motor_running && !running) {
        g_debug_move_active = false;
        g_next_debug_direction =
            (g_next_debug_direction == MOTOR_DIR_FORWARD) ?
            MOTOR_DIR_REVERSE : MOTOR_DIR_FORWARD;
    }

    g_last_motor_running = running;
}

static void Task1_Process(void)
{
    /* 调试运动由按键3逐段触发，is_start 仅作为独立任务状态标志。 */
}

static void Task2_Process(void)
{
    uint16_t measurementTicks;
    bool freshTarget;

    if (g_task2_sample_elapsed_ticks < UINT16_MAX) {
        g_task2_sample_elapsed_ticks++;
    }
    if (g_task2_control_ticks < TASK2_CONTROL_PERIOD_TICKS) {
        g_task2_control_ticks++;
    }

    /*
     * 运动过程中，尚未达到终点稳定条件就持续PID修正；视觉丢失会
     * 安全退出。终点完成时保留PID找到的平衡角并停止发STEP。
     */
    if (g_k230_target.found) {
        g_task2_vision_lost_ticks = 0U;
        if (g_k230_sample_age_ticks >= TASK2_VISION_TIMEOUT_TICKS) {
            /* 旧坐标已经失效，明确显示LOST，避免误认为终点提前完成。 */
            g_k230_target.found = false;
            g_pos_display_pending = true;
            Task2_StopAndLevel(TASK2_VISION_LOST);
            return;
        }
    } else {
        if (g_task2_vision_lost_ticks < TASK2_VISION_TIMEOUT_TICKS) {
            g_task2_vision_lost_ticks++;
        }
        if (g_task2_vision_lost_ticks >= TASK2_VISION_TIMEOUT_TICKS) {
            Task2_StopAndLevel(TASK2_VISION_LOST);
            return;
        }
    }

    freshTarget = g_k230_target.found &&
                  (g_k230_sample_age_ticks < TASK2_VISION_TIMEOUT_TICKS);
    if (g_k230_new_target &&
        (g_task2_control_ticks >= TASK2_CONTROL_PERIOD_TICKS)) {
        g_k230_new_target = false;
        if (freshTarget) {
            measurementTicks = g_task2_sample_elapsed_ticks;
            if (measurementTicks == 0U) {
                measurementTicks = 1U;
            } else if (measurementTicks > TASK2_VISION_TIMEOUT_TICKS) {
                measurementTicks = TASK2_VISION_TIMEOUT_TICKS;
            }

            g_task2_sample_elapsed_ticks = 0U;
            g_task2_control_ticks = 0U;
            Task2_ProcessMeasurement(measurementTicks);
        }
    }
}

static void Task3_ResetPid(void)
{
    g_task3_pid_integral_scaled = 0;
}

static void Task3_SaveBalanceBias(int16_t balanceAngle_0p1deg)
{
    int32_t limitedAngle = Clamp32(
        balanceAngle_0p1deg,
        -TASK3_PID_INTEGRAL_LIMIT_0P1DEG,
        TASK3_PID_INTEGRAL_LIMIT_0P1DEG);

    /*
     * 位置接近0、速度接近0时，当前实际角度就是已找到的平衡偏置。
     * 把它保存为积分项，恢复PID时不会错误地重新追向启动姿态0度。
     */
    g_task3_pid_integral_scaled = limitedAngle * 10000;
}

static void Task3_ResetBreakaway(void)
{
    g_task3_stuck_ticks = 0U;
    g_task3_breakaway_ticks = 0U;
    g_task3_breakaway_active = false;
}

static void Task3_ResetControlState(void)
{
    g_task3_command_angle_0p1deg = 0;
    g_task3_last_position_0p1cm = 0;
    g_task3_velocity_0p1cm_s = 0;
    g_task3_have_last_position = false;
    g_task3_sample_elapsed_ticks = 0U;
    g_task3_control_ticks = 0U;
    g_task3_vision_lost_ticks = 0U;
    g_task3_pid_integral_scaled = 0;
    g_task3_hold_stable_ticks = 0U;
    g_task3_holding_center = false;
    Task3_ResetBreakaway();
    g_k230_new_target = false;
}

static void Task3_Start(void)
{
    /*
     * 不要求小球先位于0 cm，也不把当前姿态当作物理水平。
     * 这里只把当前电机位置设为“相对角度计数参考点”；下一帧K230坐标
     * 到来后，PID会从小球的任意当前位置向0 cm纠偏。
     */
    Motor_SetLogicalZero();
    Task3_ResetControlState();
    g_k230_sample_age_ticks = 0U;
    g_task3_state = TASK3_RUNNING;
    g_is_start = true;
    g_header_display_pending = true;
    g_motor_display_pending = true;
}

static void Task3_Stop(Task3State finalState)
{
    /*
     * 任务3没有机械绝对零位。停止或视觉丢失时保持当前实际角度，
     * 不盲目返回启动姿态，避免把已经居中的小球再次推走。
     */
    Motor_Stop();
    g_task3_command_angle_0p1deg = Motor_GetAngle0p1Deg();
    g_task3_state = finalState;
    g_task3_hold_stable_ticks = 0U;
    g_task3_holding_center = false;
    Task3_ResetPid();
    Task3_ResetBreakaway();
    g_k230_new_target = false;
    g_is_start = false;
    g_header_display_pending = true;
    g_motor_display_pending = true;
}

static void Task3_UpdateVelocity(uint16_t elapsedTicks)
{
    int16_t position = g_k230_target.position_0p1cm;

    if (!g_task3_have_last_position) {
        g_task3_velocity_0p1cm_s = 0;
        g_task3_have_last_position = true;
    } else {
        int32_t delta = (int32_t) position - g_task3_last_position_0p1cm;
        int32_t rawVelocity;

        if (elapsedTicks == 0U) {
            elapsedTicks = 1U;
        }
        rawVelocity = (delta * 100) / (int32_t) elapsedTicks;
        rawVelocity = Clamp32(rawVelocity, -1000, 1000);
        g_task3_velocity_0p1cm_s =
            (g_task3_velocity_0p1cm_s + rawVelocity) / 2;
    }

    g_task3_last_position_0p1cm = position;
}

static int16_t Task3_QuantizeAngle(int32_t angle_0p1deg)
{
    if (angle_0p1deg >= 0) {
        angle_0p1deg =
            ((angle_0p1deg + (TASK3_COMMAND_STEP_0P1DEG / 2)) /
             TASK3_COMMAND_STEP_0P1DEG) * TASK3_COMMAND_STEP_0P1DEG;
    } else {
        angle_0p1deg =
            -(((-angle_0p1deg + (TASK3_COMMAND_STEP_0P1DEG / 2)) /
               TASK3_COMMAND_STEP_0P1DEG) * TASK3_COMMAND_STEP_0P1DEG);
    }
    return (int16_t) angle_0p1deg;
}

static int16_t Task3_ShapeCommand(int32_t command, int32_t limit)
{
    int32_t minimumSlew;
    int32_t maximumSlew;

    command = Clamp32(command, -limit, limit);
    minimumSlew = (int32_t) g_task3_command_angle_0p1deg -
                  TASK3_COMMAND_SLEW_0P1DEG;
    maximumSlew = (int32_t) g_task3_command_angle_0p1deg +
                  TASK3_COMMAND_SLEW_0P1DEG;
    command = Clamp32(command, minimumSlew, maximumSlew);
    command = Task3_QuantizeAngle(command);
    command = Clamp32(command, -limit, limit);
    return (int16_t) command;
}

static int16_t Task3_CalculatePidCommand(uint16_t elapsedTicks)
{
    int32_t error =
        (int32_t) TASK3_TARGET_POSITION_0P1CM -
        g_k230_target.position_0p1cm;
    int32_t absoluteError = Abs32(error);
    int32_t absoluteVelocity = Abs32(g_task3_velocity_0p1cm_s);
    int32_t proportionalTerm;
    int32_t integralTerm;
    int32_t derivativeTerm;
    int32_t command;
    int32_t candidateIntegralTerm;
    int32_t candidateCommand;
    int64_t integralLimitScaled =
        (int64_t) TASK3_PID_INTEGRAL_LIMIT_0P1DEG * 10000;
    int64_t candidateIntegral;

    if (elapsedTicks == 0U) {
        elapsedTicks = 1U;
    }

    if (g_task3_breakaway_active) {
        uint32_t accumulated =
            (uint32_t) g_task3_breakaway_ticks + elapsedTicks;
        g_task3_breakaway_ticks =
            (accumulated > UINT16_MAX) ? UINT16_MAX :
                                         (uint16_t) accumulated;

        if ((absoluteVelocity >= TASK3_MOVING_SPEED_0P1CM_S) ||
            (absoluteError < TASK3_STUCK_ERROR_0P1CM) ||
            (g_task3_breakaway_ticks >= TASK3_BREAKAWAY_MAX_TICKS)) {
            Task3_ResetBreakaway();
        } else {
            command = (error >= 0) ? TASK3_BREAKAWAY_TILT_0P1DEG :
                                     -TASK3_BREAKAWAY_TILT_0P1DEG;
            return Task3_ShapeCommand(command,
                                      TASK3_BREAKAWAY_TILT_0P1DEG);
        }
    }

    if ((absoluteError >= TASK3_STUCK_ERROR_0P1CM) &&
        (absoluteVelocity <= TASK3_STUCK_SPEED_0P1CM_S)) {
        uint32_t accumulated =
            (uint32_t) g_task3_stuck_ticks + elapsedTicks;
        g_task3_stuck_ticks =
            (accumulated >= TASK3_STUCK_TIME_TICKS) ?
            TASK3_STUCK_TIME_TICKS : (uint16_t) accumulated;

        if (g_task3_stuck_ticks >= TASK3_STUCK_TIME_TICKS) {
            g_task3_breakaway_active = true;
            g_task3_breakaway_ticks = 0U;
            g_task3_stuck_ticks = 0U;
            Task3_ResetPid();
            command = (error >= 0) ? TASK3_BREAKAWAY_TILT_0P1DEG :
                                     -TASK3_BREAKAWAY_TILT_0P1DEG;
            return Task3_ShapeCommand(command,
                                      TASK3_BREAKAWAY_TILT_0P1DEG);
        }
    } else {
        g_task3_stuck_ticks = 0U;
    }

    proportionalTerm = (TASK3_PID_KP_X100 * error) / 100;
    derivativeTerm =
        (TASK3_PID_KD_X100 * g_task3_velocity_0p1cm_s) / 100;

    candidateIntegral =
        (int64_t) g_task3_pid_integral_scaled +
        ((int64_t) TASK3_PID_KI_X100 * error * elapsedTicks);
    if (candidateIntegral > integralLimitScaled) {
        candidateIntegral = integralLimitScaled;
    } else if (candidateIntegral < -integralLimitScaled) {
        candidateIntegral = -integralLimitScaled;
    }

    candidateIntegralTerm = (int32_t) candidateIntegral / 10000;
    candidateCommand =
        proportionalTerm + candidateIntegralTerm - derivativeTerm;
    if (!(((candidateCommand >= TASK3_PID_TILT_LIMIT_0P1DEG) &&
           (error > 0)) ||
          ((candidateCommand <= -TASK3_PID_TILT_LIMIT_0P1DEG) &&
           (error < 0)))) {
        g_task3_pid_integral_scaled = (int32_t) candidateIntegral;
    }

    integralTerm = g_task3_pid_integral_scaled / 10000;
    command = proportionalTerm + integralTerm - derivativeTerm;
    return Task3_ShapeCommand(command, TASK3_PID_TILT_LIMIT_0P1DEG);
}

static void Task3_ApplyCommand(int16_t command_0p1deg)
{
    if (command_0p1deg == g_task3_command_angle_0p1deg) {
        return;
    }

    g_task3_command_angle_0p1deg = command_0p1deg;
    Motor_MoveToAngle0p1Deg(command_0p1deg);
    g_motor_display_pending = true;
}

static void Task3_ProcessMeasurement(uint16_t elapsedTicks)
{
    int32_t error;
    int32_t absoluteError;
    int32_t absoluteVelocity;
    int16_t command;

    Task3_UpdateVelocity(elapsedTicks);
    error = (int32_t) TASK3_TARGET_POSITION_0P1CM -
            g_k230_target.position_0p1cm;
    absoluteError = Abs32(error);
    absoluteVelocity = Abs32(g_task3_velocity_0p1cm_s);

    if (g_task3_holding_center) {
        /*
         * 在允许范围内保持PID找到的实际平衡角；一旦位置越界，或仍有
         * 明显速度可能即将越界，就提前恢复闭环制动和位置修正。
         */
        if ((absoluteError <= TASK3_RESTART_ERROR_0P1CM) &&
            (absoluteVelocity <= TASK3_RESTART_SPEED_0P1CM_S)) {
            return;
        }

        g_task3_holding_center = false;
        g_task3_hold_stable_ticks = 0U;
        g_task3_command_angle_0p1deg = Motor_GetAngle0p1Deg();
        Task3_ResetBreakaway();
    }

    if ((absoluteError <= TASK3_HOLD_ERROR_0P1CM) &&
        (absoluteVelocity <= TASK3_HOLD_SPEED_0P1CM_S)) {
        uint32_t accumulated =
            (uint32_t) g_task3_hold_stable_ticks + elapsedTicks;
        g_task3_hold_stable_ticks =
            (accumulated >= TASK3_HOLD_STABLE_TICKS) ?
            TASK3_HOLD_STABLE_TICKS : (uint16_t) accumulated;
    } else {
        g_task3_hold_stable_ticks = 0U;
    }

    if (g_task3_hold_stable_ticks >= TASK3_HOLD_STABLE_TICKS) {
        Motor_Stop();
        g_task3_command_angle_0p1deg = Motor_GetAngle0p1Deg();
        Task3_SaveBalanceBias(g_task3_command_angle_0p1deg);
        g_task3_holding_center = true;
        Task3_ResetBreakaway();
        g_motor_display_pending = true;
        return;
    }

    command = Task3_CalculatePidCommand(elapsedTicks);
    Task3_ApplyCommand(command);
}

static void Task3_Process(void)
{
    uint16_t measurementTicks;
    bool freshTarget;

    if (g_task3_state != TASK3_RUNNING) {
        return;
    }

    if (g_task3_sample_elapsed_ticks < UINT16_MAX) {
        g_task3_sample_elapsed_ticks++;
    }
    if (g_task3_control_ticks < TASK3_CONTROL_PERIOD_TICKS) {
        g_task3_control_ticks++;
    }

    if (g_k230_target.found) {
        g_task3_vision_lost_ticks = 0U;
        if (g_k230_sample_age_ticks >= TASK3_VISION_TIMEOUT_TICKS) {
            g_k230_target.found = false;
            g_pos_display_pending = true;
            Task3_Stop(TASK3_VISION_LOST);
            return;
        }
    } else {
        if (g_task3_vision_lost_ticks < TASK3_VISION_TIMEOUT_TICKS) {
            g_task3_vision_lost_ticks++;
        }
        if (g_task3_vision_lost_ticks >= TASK3_VISION_TIMEOUT_TICKS) {
            Task3_Stop(TASK3_VISION_LOST);
            return;
        }
    }

    freshTarget = g_k230_target.found &&
                  (g_k230_sample_age_ticks < TASK3_VISION_TIMEOUT_TICKS);
    if (g_k230_new_target &&
        (g_task3_control_ticks >= TASK3_CONTROL_PERIOD_TICKS)) {
        g_k230_new_target = false;
        if (freshTarget) {
            measurementTicks = g_task3_sample_elapsed_ticks;
            if (measurementTicks == 0U) {
                measurementTicks = 1U;
            } else if (measurementTicks > TASK3_VISION_TIMEOUT_TICKS) {
                measurementTicks = TASK3_VISION_TIMEOUT_TICKS;
            }

            g_task3_sample_elapsed_ticks = 0U;
            g_task3_control_ticks = 0U;
            Task3_ProcessMeasurement(measurementTicks);
        }
    }
}

static void ProcessSelectedTask(void)
{
    if (!g_is_start) {
        return;
    }

    switch (g_current_task) {
    case TASK_1:
        Task1_Process();
        break;
    case TASK_2:
        Task2_Process();
        break;
    case TASK_3:
        Task3_Process();
        break;
    default:
        break;
    }
}

static void ProcessKeys(void)
{
    Key_Scan(&g_task_key);
    Key_Scan(&g_is_start_key);
    Key_Scan(&g_motor_start_key);

    /* 按键1短按和重复事件不用；只消费一次性的0.5秒长按事件。 */
    (void) Key_TakePressed(&g_task_key);
    (void) Key_TakeRepeat(&g_task_key);
    if (Key_TakeLongPressed(&g_task_key)) {
        SwitchTask();
    }

    if (Key_TakePressed(&g_is_start_key)) {
        HandleIsStartKey();
    }
    if (Key_TakePressed(&g_motor_start_key)) {
        HandleMotorStartKey();
    }

    /* 这些按键只需要短按事件，丢弃未使用的长按/重复事件。 */
    (void) Key_TakeRepeat(&g_is_start_key);
    (void) Key_TakeLongPressed(&g_is_start_key);
    (void) Key_TakeRepeat(&g_motor_start_key);
    (void) Key_TakeLongPressed(&g_motor_start_key);
}

static void ProcessDisplays(void)
{
    if (g_header_display_pending) {
        ShowHeader();
        g_header_display_pending = false;
    }

    if (g_pos_display_pending &&
        (g_pos_refresh_ticks >= POS_REFRESH_TICKS)) {
        ShowPosition();
        g_pos_display_pending = false;
        g_pos_refresh_ticks = 0U;
    }

    if (g_motor_display_pending) {
        ShowMotorParameters();
        g_motor_display_pending = false;
    }
}

static void App_Process10ms(void)
{
    if (g_k230_sample_age_ticks < UINT16_MAX) {
        g_k230_sample_age_ticks++;
    }

    if (g_pos_refresh_ticks < POS_REFRESH_TICKS) {
        g_pos_refresh_ticks++;
    }

    ProcessKeys();
    ProcessSelectedTask();
    ProcessMotorCompletion();
    ProcessDisplays();
}

int main(void)
{
    SYSCFG_DL_init();

    OLED_Init();
    OLED_ColorTurn(0U);
    OLED_DisplayTurn(0U);

    Motor_Init();
    K230_Init();
    PcForward_Init();

    /* PB20/PB19/PB18 均为内部下拉输入，按下接 3.3 V。 */
    Key_Init(&g_task_key, KEY_PORT, KEY_TASK_PIN);
    Key_Init(&g_is_start_key, KEY_PORT, KEY_IS_START_PIN);
    Key_Init(&g_motor_start_key, KEY_PORT, KEY_MOTOR_START_PIN);

    g_app_tick_pending = 0U;
    NVIC_ClearPendingIRQ(APP_TICK_INST_INT_IRQN);
    NVIC_EnableIRQ(APP_TICK_INST_INT_IRQN);

    ShowInitialScreen();

    while (1) {
        K230_Target_t receivedTarget;

        /* K230 解析、电脑转发连续运行，不等待10 ms应用节拍。 */
        K230_Process();
        PcForward_Start();

        if (K230_GetLatestTarget(&receivedTarget)) {
            g_k230_target = receivedTarget;
            g_k230_new_target = true;
            g_k230_sample_age_ticks = 0U;
            g_pos_display_pending = true;
        }

        while (App_TakeTick()) {
            App_Process10ms();
        }
    }
}

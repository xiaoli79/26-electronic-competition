#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

/*
 * ======================== 实机调试参数总表 ========================
 *
 * 实机调试时优先只修改本文件，empty.c 和 motor.c 不再保存重复参数。
 *
 * 单位约定：
 *   _0P1DEG      0.1度，例如 60=6.0度，-80=-8.0度；
 *   _0P1CM       0.1 cm，例如 47=+4.7 cm，-50=-5.0 cm；
 *   _0P1CM_S     0.1 cm/s，例如 5=0.5 cm/s；
 *   _TICKS       10 ms应用节拍，例如 20=200 ms，500=5.00 s。
 */

/* -------------------- 应用节拍与OLED刷新 -------------------- */
#define APP_TICK_PERIOD_MS                  10U /* TIMG8应用节拍：10 ms。 */
#define POS_REFRESH_PERIOD_MS               500U /* OLED位置区域最快每500 ms刷新一次。 */
#define POS_REFRESH_TICKS \
    (POS_REFRESH_PERIOD_MS / APP_TICK_PERIOD_MS)

/* -------------------- 步进电机硬件与固定速度 -------------------- */
#define MOTOR_STEP_PER_REV                 6400U /* 驱动器细分后的每圈STEP脉冲数。 */
#define MOTOR_DEGREES_PER_REV               360U /* 电机轴一圈的机械角度。 */
#define MOTOR_FIXED_SPEED_DEG_S              60U /* 任务1、任务2、任务3共用的电机轴固定转速。 */
#define MOTOR_MIN_TIMER_PERIOD              800U /* TIMG0最小周期，即最高脉冲频率保护。 */

/* -------------------- 任务1：手动电机调试 -------------------- */
#define DEBUG_MOTOR_ANGLE_DEG                 15U /* 按键3每次相对转动15度。 */

/* -------------------- 任务2：控制周期与视觉保护 -------------------- */
#define TASK2_CONTROL_PERIOD_TICKS              5U /* 每50 ms使用一次最新K230位置。 */
#define TASK2_VISION_TIMEOUT_TICKS            300U /* 视觉连续丢失3000 ms后安全退出。 */

/* -------------------- 任务2：0 -> +5 cm -> -5 cm --------------------
 * K230位置进入中心点±1.0 cm范围时建立电机逻辑零点并启动PID。
 * K230位置达到“+5.0 cm减去提前折返量”后立即开始REV，让小球利用
 * 剩余动能通过+5.0 cm；最终按-5±0.5 cm内部条件停止，为赛题要求的
 * -5±1 cm保留0.5 cm余量。
 */
#define TASK2_START_POSITION_0P1CM              0  /* 启动中心位置：0.0 cm。 */
#define TASK2_START_ERROR_0P1CM                 10  /* 启动允许范围：-1～+1 cm。 */
#define TASK2_TARGET_POS5_0P1CM                50  /* 第一目标：+5.0 cm。 */
#define TASK2_TARGET_NEG5_0P1CM              (-50) /* 最终目标：-5.0 cm。 */
#define TASK2_ENDPOINT_ERROR_0P1CM             10  /* 赛题+5 cm允许误差记录：±1.0 cm。 */
#define TASK2_POS5_TURN_LEAD_0P1CM             20  /* 提前折返量：2.0 cm；设0则到+5.0 cm才折返。 */
#define TASK2_FINISH_ERROR_0P1CM                8  /* 最终停止内部误差：-5±0.8 cm。 */
#define TASK2_FINISH_SPEED_0P1CM_S               5  /* 最终速度不超过0.5 cm/s。 */
#define TASK2_FINISH_STABLE_TICKS              10U /* 终点连续稳定100 ms才完成。 */

/* -------------------- 任务2：分阶段PID增益 --------------------
 * PID形式：目标角 = Kp*位置误差 + Ki*误差积分 - Kd*小球速度。
 * Kp/Ki/Kd均放大100倍保存，例如200表示2.00。
 * 0到+5 cm采用单向通过控制：位置PID只提供正向大小，不使用D项提前
 * 制动；+5到-4.5 cm降低Kd并增大Kp，让负向传输更积极；
 * 第一次到达-4.5 cm后锁定最终稳定参数，不再切回传输参数。
 */
#define TASK2_POS_TRANSFER_KP_X100            180 /* 0到+5 cm：Kp=1.80度/cm。 */
#define TASK2_POS_TRANSFER_KD_X100              0 /* 单向通过+5 cm：禁用D项提前REV制动。 */
#define TASK2_NEG_TRANSFER_KP_X100            300 /* +5到-4 cm前：Kp=3.00度/cm。 */
#define TASK2_NEG_TRANSFER_KD_X100            100 /* +5到-4 cm前：Kd=1.00度/(cm/s)。 */
#define TASK2_SETTLE_KP_X100                  220 /* 进入终点区后：Kp=2.20度/cm。 */
#define TASK2_SETTLE_KD_X100                  120 /* 进入终点区后：Kd=1.20度/(cm/s)。 */

/* Ki为任务2公共参数；+5 cm折返时清零，负向切入稳定阶段时保留积分。 */
#define TASK2_PID_KI_X100                      15 /* Ki=0.15度/(cm*s)。 */
#define TASK2_PID_INTEGRAL_LIMIT_0P1DEG        10 /* 积分项最大贡献：±1.0度。 */
#define TASK2_PID_TILT_LIMIT_0P1DEG            60 /* 电机逻辑目标角最大值：±6.0度。 */
#define TASK2_COMMAND_STEP_0P1DEG               5 /* 角度命令量化步长：0.5度。 */
#define TASK2_COMMAND_SLEW_0P1DEG              40 /* 每50 ms目标最多改变4.0度。 */

/* 0到+5 cm期间，位置尚未通过+5.0 cm时保持至少+1.5度FWD推进，
 * 防止误差变小后PID输出落入量化死区或因视觉速度项提前反向。 */
#define TASK2_POS_MIN_DRIVE_0P1DEG              15 /* FWD最低推进角：+1.5度。 */

/* -------------------- 任务2：负向传输最低推进量 --------------------
 * 在尚未到达-4.5 cm、且小球速度低于0.8 cm/s时，如果PID给出的REV
 * 角度不足1.5度，则至少保持REV 1.5度，防止过早犹豫停住。
 */
#define TASK2_SETTLE_ENTRY_POSITION_0P1CM    (-45) /* 到达-4.5 cm才切入最终稳定。 */
#define TASK2_NEG_MIN_DRIVE_0P1DEG             15 /* 最小REV推进角：1.5度。 */
#define TASK2_NEG_MIN_SPEED_0P1CM_S             8 /* 低速阈值：0.8 cm/s。 */

/* -------------------- 任务2：静摩擦启动补偿 --------------------
 * 当位置误差仍较大、小球连续150 ms几乎不动时，临时使用±8.0度启动角。
 * 补偿最多保持250 ms；检测到小球速度达到0.5 cm/s后立即恢复正常PID。
 * 启动补偿期间暂停积分，避免克服静摩擦后由历史积分继续推动小球超调。
 */
#define TASK2_BREAKAWAY_TILT_0P1DEG            80 /* 静摩擦启动补偿角：±8.0度。 */
#define TASK2_STUCK_ERROR_0P1CM                15 /* 误差至少1.5 cm才检查卡住。 */
#define TASK2_STUCK_SPEED_0P1CM_S               2 /* 速度不超过0.2 cm/s视为近似静止。 */
#define TASK2_STUCK_TIME_TICKS                 20U /* 近似静止持续200 ms后触发。 */
#define TASK2_BREAKAWAY_MAX_TICKS              25U /* 单次启动补偿最长250 ms。 */
#define TASK2_MOVING_SPEED_0P1CM_S              5 /* 速度达到0.5 cm/s视为已经启动。 */

/* -------------------- 任务3：任意位置 -> 0 cm持续稳定 --------------------
 * 按下is_start后不检查小球初始位置，收到新的K230坐标便立即开始闭环。
 * 进入±0.3 cm且速度较低后保持当前电机平衡角；位置超过±0.6 cm或
 * 小球速度再次增大时自动恢复PID，因此任务3不会像任务2那样自动结束。
 */
#define TASK3_CONTROL_PERIOD_TICKS              9U /* 每90 ms使用一次最新K230位置。 */
#define TASK3_VISION_TIMEOUT_TICKS              300U /* 视觉连续丢失3000 ms后停止控制。 */
#define TASK3_TARGET_POSITION_0P1CM               0 /* 固定目标：0.0 cm。 */
#define TASK3_HOLD_ERROR_0P1CM                     3 /* 进入保持的误差：±0.3 cm。 */
#define TASK3_RESTART_ERROR_0P1CM                 6 /* 超过±0.6 cm立即恢复PID。 */
#define TASK3_HOLD_SPEED_0P1CM_S                   2 /* 进入保持时速度不超过0.2 cm/s。 */
#define TASK3_RESTART_SPEED_0P1CM_S                4 /* 速度超过0.4 cm/s提前恢复PID。 */
#define TASK3_HOLD_STABLE_TICKS                  10U /* 连续稳定100 ms后保持平衡角。 */

/* -------------------- 任务3：PID与电机角度保护 --------------------
 * PID形式和单位与任务2相同，但参数完全独立，便于只整定中心稳定任务。
 * KP越大，电机角度越大，越容易克服静摩擦，但也更容易过冲；KD越大，电机角度越小，响应更平稳，但也更容易被静摩擦卡住。
 */
#define TASK3_PID_KP_X100                       180 /* Kp=1.80度/cm。 */
#define TASK3_PID_KI_X100                        5 /* Ki=0.05度/(cm*s)。 */
#define TASK3_PID_KD_X100                       120 /* Kd=1.20度/(cm/s)。 */
#define TASK3_PID_INTEGRAL_LIMIT_0P1DEG          60 /* 积分项可学习启动姿态偏差：±6.0度。 */
#define TASK3_PID_TILT_LIMIT_0P1DEG              150 /* 相对启动姿态最大调整：±15.0度。 */
#define TASK3_COMMAND_STEP_0P1DEG                 2 /* 角度命令量化步长：0.2度。 */
#define TASK3_COMMAND_SLEW_0P1DEG                25 /* 每90 ms目标最多改变2.5度。 */

/* 任务3也保留静摩擦启动补偿，保证小球离中心较远且静止时能够起步。 */
#define TASK3_BREAKAWAY_TILT_0P1DEG               80 /* 静摩擦启动补偿角：±8.0度。 */
#define TASK3_STUCK_ERROR_0P1CM                   10 /* 误差至少1.0 cm才检查卡住。 */
#define TASK3_STUCK_SPEED_0P1CM_S                 2 /* 速度不超过0.2 cm/s视为近似静止。 */
#define TASK3_STUCK_TIME_TICKS                   20U /* 近似静止持续200 ms后触发。 */
#define TASK3_BREAKAWAY_MAX_TICKS                25U /* 单次启动补偿最长250 ms。 */
#define TASK3_MOVING_SPEED_0P1CM_S                5 /* 速度达到0.5 cm/s视为已经启动。 */

/* 基础配置关系检查，避免调参时产生除零或无效刷新周期。 */
#if (APP_TICK_PERIOD_MS == 0U) || \
    ((POS_REFRESH_PERIOD_MS % APP_TICK_PERIOD_MS) != 0U)
#error "POS_REFRESH_PERIOD_MS必须是APP_TICK_PERIOD_MS的整数倍"
#endif

#if (TASK2_POS_TRANSFER_KP_X100 < 0) || \
    (TASK2_POS_TRANSFER_KD_X100 < 0) || \
    (TASK2_NEG_TRANSFER_KP_X100 < 0) || \
    (TASK2_NEG_TRANSFER_KD_X100 < 0) || \
    (TASK2_SETTLE_KP_X100 < 0) || (TASK2_SETTLE_KD_X100 < 0) || \
    (TASK2_PID_KI_X100 < 0) || (TASK2_PID_TILT_LIMIT_0P1DEG <= 0) || \
    (TASK2_START_ERROR_0P1CM < 0) || \
    (TASK2_POS5_TURN_LEAD_0P1CM < 0) || \
    (TASK2_POS5_TURN_LEAD_0P1CM >= TASK2_TARGET_POS5_0P1CM) || \
    (TASK2_FINISH_ERROR_0P1CM <= 0) || \
    (TASK2_FINISH_ERROR_0P1CM > TASK2_ENDPOINT_ERROR_0P1CM) || \
    (TASK2_FINISH_SPEED_0P1CM_S < 0) || \
    (TASK2_FINISH_STABLE_TICKS == 0U) || \
    (TASK2_POS_MIN_DRIVE_0P1DEG <= 0) || \
    (TASK2_POS_MIN_DRIVE_0P1DEG > TASK2_PID_TILT_LIMIT_0P1DEG) || \
    (TASK2_NEG_MIN_DRIVE_0P1DEG <= 0) || \
    (TASK2_NEG_MIN_DRIVE_0P1DEG > TASK2_PID_TILT_LIMIT_0P1DEG) || \
    (TASK2_NEG_MIN_SPEED_0P1CM_S < 0) || \
    (TASK2_COMMAND_STEP_0P1DEG <= 0) || \
    (TASK2_COMMAND_SLEW_0P1DEG <= 0)
#error "任务2位置、PID增益和角度保护参数必须为有效非负值"

#endif

#if (TASK2_BREAKAWAY_TILT_0P1DEG < TASK2_PID_TILT_LIMIT_0P1DEG) || \
    (TASK2_STUCK_ERROR_0P1CM <= 0) || \
    (TASK2_STUCK_SPEED_0P1CM_S < 0) || \
    (TASK2_STUCK_TIME_TICKS == 0U) || \
    (TASK2_BREAKAWAY_MAX_TICKS == 0U) || \
    (TASK2_MOVING_SPEED_0P1CM_S <= TASK2_STUCK_SPEED_0P1CM_S)
#error "任务2静摩擦补偿参数关系无效"
#endif

#if (TASK3_PID_KP_X100 < 0) || (TASK3_PID_KI_X100 < 0) || \
    (TASK3_PID_KD_X100 < 0) || (TASK3_PID_TILT_LIMIT_0P1DEG <= 0) || \
    (TASK3_HOLD_ERROR_0P1CM < 0) || \
    (TASK3_RESTART_ERROR_0P1CM < TASK3_HOLD_ERROR_0P1CM) || \
    (TASK3_HOLD_SPEED_0P1CM_S < 0) || \
    (TASK3_RESTART_SPEED_0P1CM_S < TASK3_HOLD_SPEED_0P1CM_S) || \
    (TASK3_HOLD_STABLE_TICKS == 0U) || \
    (TASK3_COMMAND_STEP_0P1DEG <= 0) || \
    (TASK3_COMMAND_SLEW_0P1DEG <= 0)
#endif

#if (TASK3_BREAKAWAY_TILT_0P1DEG <= 0) || \
    (TASK3_BREAKAWAY_TILT_0P1DEG > TASK3_PID_TILT_LIMIT_0P1DEG) || \
    (TASK3_STUCK_ERROR_0P1CM <= TASK3_RESTART_ERROR_0P1CM) || \
    (TASK3_STUCK_SPEED_0P1CM_S < 0) || \
    (TASK3_STUCK_TIME_TICKS == 0U) || \
    (TASK3_BREAKAWAY_MAX_TICKS == 0U) || \
    (TASK3_MOVING_SPEED_0P1CM_S <= TASK3_STUCK_SPEED_0P1CM_S)
#error "任务3静摩擦补偿参数关系无效"
#endif

#endif

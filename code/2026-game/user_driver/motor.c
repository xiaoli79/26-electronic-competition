#include "motor.h"

//PID参数
float kp = 0.5; // 比例系数
float ki = 0.4; // 积分系数

int PWM_1_duty = 0; //当前PWM占空比
float target_speed_1 = 0; // 目标速度 mm/s
float last_error_1 = 0; //上一次误差(PID微分用)
float current_error_1 = 0; //当前误差

int PWM_2_duty = 0; //当前PWM占空比
float target_speed_2 = 0; // 目标速度 mm/s
float last_error_2 = 0; //上一次误差(PID微分用)
float current_error_2 = 0;

extern volatile int32_t counter_1_A;
float speed_1 = 0; //实际速度 mm/s

extern volatile int32_t counter_2_A;
float speed_2 = 0; //实际速度 mm/s
extern volatile int32_t encoder_total_count_1;
extern volatile int32_t encoder_total_count_2;
extern Attitude_t current_attitude;
extern float yaw_start;

extern volatile uint8_t task;
extern volatile uint8_t is_start;
extern volatile int64_t last_change_time;

/* =====================================================================
 * 任务 1：自动直行 1500 mm 编码器测距测试
 *
 * KEY2 启动后，左右轮使用相同的目标速度和各自的速度 PI 闭环直行。
 * 总距离取左右轮相对起点绝对位移的平均值；达到 1500 mm 后短刹，
 * 最终时间、平均距离、左右轮距离和原始计数会一直保留在 OLED 上。
 * ===================================================================== */
#if 0
#define TASK1_ENCODER_MM_PER_COUNT MOTOR_ENCODER_MM_PER_COUNT
#define TASK1_CONTROL_PERIOD_S              (0.01f)
#define TASK1_SPEED_SAMPLE_TICKS            (2U)     /* 2 x 10 ms = 20 ms */
#define TASK1_SPEED_FILTER_ALPHA             (0.35f)
#define TASK1_TARGET_DISTANCE_MM             (1500.0f)
#define TASK1_PRE_DECEL_DISTANCE_MM          (1300.0f)
#define TASK1_CRUISE_SPEED_MM_S              (200.0f)
#define TASK1_APPROACH_SPEED_MM_S            (100.0f)
#define TASK1_ACCEL_MM_S2                    (150.0f)
#define TASK1_DECEL_MM_S2                    (200.0f)
#define TASK1_SPEED_STATIC_PWM               (180.0f)
#define TASK1_SPEED_KFF                      (2.0f)
#define TASK1_SPEED_KP                       (1.0f)
#define TASK1_SPEED_KI                       (2.0f)
#define TASK1_SPEED_INTEGRAL_LIMIT           (300.0f)
#define TASK1_MAX_DRIVE_PWM                  (1800)
#define TASK1_BRAKE_DUTY                     (2500U)
#define TASK1_BRAKE_HOLD_MS                  (80U)
#define TASK1_TIMEOUT_MS                     (15000U)

volatile int32_t task1_encoder_count_1 = 0;
volatile int32_t task1_encoder_count_2 = 0;
volatile float task1_encoder_distance_1_mm = 0.0f;
volatile float task1_encoder_distance_2_mm = 0.0f;
volatile uint32_t task1_total_time_ms = 0U;
volatile float task1_distance_mm = 0.0f;
volatile Task1_State_t task1_state = TASK1_ACCEL;
volatile uint8_t task1_finished = 0U;

static int32_t task1_encoder_zero_1 = 0;
static int32_t task1_encoder_zero_2 = 0;
static int32_t task1_speed_last_encoder_1 = 0;
static int32_t task1_speed_last_encoder_2 = 0;
static volatile uint8_t task1_encoder_reset_requested = 1U;
static uint8_t task1_initialized = 0U;
static uint8_t task1_speed_sample_tick = 0U;
static float task1_profile_speed_mm_s = 0.0f;
static float task1_speed_integral_1 = 0.0f;
static float task1_speed_integral_2 = 0.0f;
static int64_t task1_start_time_ms = 0;
static int64_t task1_brake_start_time_ms = 0;

void task1_encoder_reset(void)
{
    /* 由 10 ms 电机中断统一读取并建立零点，避免主循环和中断同时修改数据。 */
    task1_encoder_reset_requested = 1U;
}
#endif

/* Task 1: low-speed eight-channel infrared line-following test. */
#define TASK1_LINE_TEST_BASE_PWM          (800)
#define TASK1_LINE_TEST_KP                (65)
#define TASK1_LINE_TEST_KD                (25)
#define TASK1_LINE_TEST_DIFF_LIMIT        (550)
#define TASK1_LINE_TEST_REVERSE_STEERING  (0U)

volatile uint8_t task1_line_count = 0U;
volatile int16_t task1_line_error_x10 = 0;
volatile int16_t task1_line_diff_pwm = 0;
volatile uint16_t task1_line_pwm_1 = 0U;
volatile uint16_t task1_line_pwm_2 = 0U;
volatile uint8_t task1_line_sensor_ok = 0U;

static const int8_t task1_line_weight[8] = {-8, -5, -3, -1, 1, 3, 5, 8};
static int16_t task1_line_last_error_x10 = 0;
static uint8_t task1_line_controller_active = 0U;

/* KEY2 clears the task-1 infrared debug data before starting the test. */
void task1_encoder_reset(void)
{
    task1_line_count = 0U;
    task1_line_error_x10 = 0;
    task1_line_diff_pwm = 0;
    task1_line_pwm_1 = 0U;
    task1_line_pwm_2 = 0U;
    task1_line_sensor_ok = 0U;
    task1_line_last_error_x10 = 0;
    task1_line_controller_active = 0U;
}

void motor_init(uint8_t motor_id)
{
    DL_GPIO_setPins(DC_MOTOR_STBY_PORT, DC_MOTOR_STBY_PIN);
    if(motor_id == 1){
        // 初始为刹车状态
        DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
        DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        //设置占空比
        DL_Timer_setCaptureCompareValue(PWMAB_INST, 0, GPIO_PWMAB_C0_IDX);
    }
    else if(motor_id == 2){
        // 初始为刹车状态
        DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
        DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWMAB_INST, 0, GPIO_PWMAB_C1_IDX);
    }
    //PWM输出定时器
    DL_Timer_startCounter(PWMAB_INST);
    //PID中断定时器(10ms)
    DL_Timer_startCounter(MOTOR_PID_INST);
    // 打开中断
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);
}

// PWM限幅函数
// 通过更改小车的占空比进而提高电机的转速
int limit_duty(int duty)
{
    if(duty > 2500){
        duty = 2500;
    }
    if(duty < 0){
        duty = 0;
    }
    return duty;
}

// 设置电机占空比 PWM越大 -> 电机电压越高 -> 电机转速越快
void motor_set_duty(uint8_t motor_id, uint32_t duty)
{
    duty = limit_duty(duty);
    if(motor_id == 1){
        DL_Timer_setCaptureCompareValue(PWMAB_INST, duty, GPIO_PWMAB_C0_IDX);
    }
    else if(motor_id == 2){
        DL_Timer_setCaptureCompareValue(PWMAB_INST, duty, GPIO_PWMAB_C1_IDX);
    }
}

// direction: 0 停止，1 正转，2 反转
void motor_set_direction(uint8_t motor_id, uint8_t direction)
{
    if(motor_id == 1){
        if(direction == 0){
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
        else if(direction == 1){                          // 正转 (接线反了, 1/2 对调)
            DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
        else if(direction == 2){                          // 反转 (接线反了, 1/2 对调)
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
    }
    else if(motor_id == 2){
        if(direction == 0){
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
        else if(direction == 1){
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
        else if(direction == 2){
            DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
    }
}

// 编码器算速度
void calculate_speed(uint8_t motor_id)
{   
    //counter编码器脉冲数 
    //MOTOR_BIANMAQI 编码器分辨率 
    //PI 圆周率 
    //MOTOR_WHEEL_D 轮子直径
    if (motor_id == 1) {
        int32_t pulse_count = counter_1_A;
        if (pulse_count < 0) pulse_count = -pulse_count;
        speed_1 = (float)pulse_count * MOTOR_ENCODER_MM_PER_COUNT * 100.0f; // 10 ms 内脉冲换算为 mm/s
        counter_1_A = 0; // 计算完速度后清零计数器
    }
    if (motor_id == 2) {
        int32_t pulse_count = counter_2_A;
        if (pulse_count < 0) pulse_count = -pulse_count;
        speed_2 = (float)pulse_count * MOTOR_ENCODER_MM_PER_COUNT * 100.0f; // 10 ms 内脉冲换算为 mm/s
        counter_2_A = 0; // 计算完速度后清零计数器
    }
}

// PID控制函数 速度闭环(增量是PI)
void DC_MOTOR_PID(uint8_t motor_id)
{
    float error;
    if (motor_id == 1) {
        error = target_speed_1 - speed_1;// 目标速度 - 实际速度 = 误差
        current_error_1 = error;
        // PID增量式公式: PWM增量 = Kp * (当前误差 - 上次误差) + Ki * 当前误差
        PWM_1_duty += (int)(kp * (current_error_1-last_error_1) + ki *(current_error_1));
        last_error_1 = current_error_1;
        PWM_1_duty = limit_duty(PWM_1_duty);
        motor_set_duty(motor_id, PWM_1_duty);
    }
    if (motor_id == 2) {
        error = target_speed_2 - speed_2;
        current_error_2 = error;
        // PID增量式公式: PWM增量 = Kp * (当前误差 - 上次误差) + Ki * 当前误差
        PWM_2_duty += (int)(kp * (current_error_2-last_error_2) + ki *(current_error_2));
        last_error_2 = current_error_2;
        PWM_2_duty = limit_duty(PWM_2_duty);
        motor_set_duty(motor_id, PWM_2_duty);
    }
}

float yaw_k_p = 9.0;  //P比例系数: 偏1度 -> 差速加9
int yaw_pwm_base = 0; 
float yaw_k_i = 0.03; //I积分系数: 偏1度 -> 差速加0.03
float yaw_total_error = 0;
extern uint8_t huidu_value[];
extern int pwm_huidu_base;
extern int pwm_huidu_diff_half;

#define status_change_counter_init 50
int status = 0; //0陀螺仪控制车头，1是巡线状态
int status_change_times = 0; // 状态切换了多少次
int status_change_counter = status_change_counter_init; // 状态切换到陀螺仪模式倒计时


int led_beep_off_counter = 100; // 关闭声光提示倒计时
extern float last_head_for; // 上次车头朝向
extern float yaw_start_init; // 最初车头朝向

// Task 5 矩形巡线相关变量
float yaw_line_entry = 0;      // 进入巡线模式时的yaw角度
float yaw_line_exit = 0;       // 丢线时的yaw角度
int last_line_sensor = 0;      // 最后见线的传感器编号(0~7, 0=最左 7=最右)
int corner_count = 0;          // 已完成的拐角数
int turn_direction = 0;        // 1=右转, -1=左转, 0=未确定

#define MAX_PWM_DIFF 700        // 差速上限，防止转弯过猛
#define TASK5_DEBOUNCE_CNT 15    // 丢线消抖次数(20×10ms=0.2s, 比默认0.5s快)
#define TASK5_YAW_TOLERANCE 3.0f  // 原地转弯到位精度(度)

#define TASK5_DEFAULT_TURN_DIRECTION  1
#define TASK5_FIND_LINE_DEBOUNCE_CNT  3
#define TASK5_FIND_LINE_PWM           450
#define TASK5_TURN_TIMEOUT_MS         1800
#define TASK5_FIND_LINE_TIMEOUT_MS    1200

static uint8_t task5_initialized = 0U;
static uint8_t task5_find_line_count = 0U;

/* =====================================================================
 * 任务 2：环形赛道一圈后在 A 点精确停车
 *
 * 终点横线由8路红外阵列识别：至少4路同时检测到黑色，并连续确认20 ms。
 * 确认A点横线后不再按编码器继续前进，程序会在同一个控制周期清空速度PI，
 * 随即执行TB6612短刹；最终里程和横线后的制动前移量会冻结供OLED显示。
 * ===================================================================== */

/* 任务 2 的速度、距离和控制参数；先用默认值完成架空测试，再按实车标定。 */
#define TASK2_CONTROL_PERIOD_S              (0.01f)
#define TASK2_SPEED_SAMPLE_TICKS            (2U)     /* 2 × 10 ms = 20 ms */
#define TASK2_ENCODER_MM_PER_PULSE           MOTOR_ENCODER_MM_PER_COUNT
#define TASK2_SPEED_FILTER_ALPHA             (0.35f) /* 编码器速度滤波：增大则响应快但抖动大，减小则更平滑 */
#define TASK2_CRUISE_SPEED_MM_S              (480.0f)/* 直线巡航速度；由460小幅提高到480以缩短整圈时间，弯道仍由偏差自动降速 */
#define TASK2_CURVE_MIN_SPEED_MM_S           (330.0f)/* 严重偏线时的最低基础速度；兼顾过弯速度和转弯半径，过高容易冲出弯道 */
#define TASK2_APPROACH_SPEED_MM_S            (120.0f)/* 接近终点、寻找 A 点横线时的速度 */
#define TASK2_ACCEL_MM_S2                    (250.0f)/* 加速度：增大起步更快，过大容易打滑或冲线 */
#define TASK2_DECEL_MM_S2                    (300.0f)/* 减速度：增大降速更快，过大容易顿挫 */
#define TASK2_PRE_DECEL_DISTANCE_MM          (5800.0f) /* 实测一圈约 6030 mm，在 A 点前约 200mm 开始减速 */
#define TASK2_FINISH_ARM_DISTANCE_MM         (4400.0f) /* 超过 4400 mm 后才允许“灰度 >= 4 路”识别 A 点 */
#define TASK2_LINE_KP                        (26.0f) /* 循迹比例：由22提高到26，使第7/8路持续亮起时也能保持较大的回线差速 */
#define TASK2_LINE_KD                        (8.0f)  /* 循迹微分：根据误差变化提前修正并抑制摆动；过大会放大传感器跳变 */
#define TASK2_LINE_DIFF_LIMIT_MM_S           (230.0f)/* 单侧最大速度修正量；允许外侧灯触发时建立更大差速，过大可能出现左右摆动 */
#define TASK2_CURVE_SLOWDOWN_MM_S            (12.0f) /* 每偏离一个权重单位降低的基础速度；由15降至12，避免全程弯道降速过多而超时 */
#define TASK2_WHEEL_TARGET_SLEW_MM_S         (25.0f) /* 每10 ms允许目标轮速变化的最大值；由20提高到25，更快建立新的左右轮目标速度 */
#define TASK2_SPEED_STATIC_PWM               (180.0f)/* 克服减速箱静摩擦；电机低速不转时可适当增大 */
#define TASK2_SPEED_KFF                      (2.2f)  /* 速度前馈；由2.0提高到2.2，使实际轮速更接近提高后的目标速度 */
#define TASK2_SPEED_KP                       (1.2f)  /* 轮速 PI 比例；提高后加速和转弯时的轮速跟随更快，过大可能抖动 */
#define TASK2_SPEED_KI                       (2.0f)  /* 轮速 PI 积分；消除长期轮速误差，过大容易振荡 */
#define TASK2_SPEED_INTEGRAL_LIMIT           (300.0f)/* 积分限幅，防止停车或堵转后突然冲车 */
#define TASK2_MAX_DRIVE_PWM                  (2000)  /* 最大驱动PWM；给高速和外轮加速留余量，堵转时仍应立即断电 */
#define TASK2_MARKER_MIN_BLACK_SENSORS       (4U)   /* 至少 4 路为黑，判定为 A 点横线 */
#define TASK2_BRAKE_DUTY                   (2500U)  /* IN1=IN2=1 且 PWM 有效时为 TB6612 短刹 */
#define TASK2_BRAKE_HOLD_MS                (80U)    /* 横线确认后立即强刹，并保持80 ms */
#define TASK2_MARKER_CONFIRM_TICKS         (2U)     /* 连续检测20 ms才确认横线，防止传感器瞬时误判 */
#define TASK2_LOST_LINE_LIMIT_TICKS        (30U)   /* 300 ms 持续丢线后安全停车 */
#define TASK2_TIMEOUT_MS                   (20000U)

volatile Task2_State_t task2_state = TASK2_LEAVE_START;
static uint8_t task2_initialized = 0U;
static uint8_t task2_marker_confirm_count = 0U;
static uint8_t task2_lost_line_count = 0U;
static uint8_t task2_speed_sample_tick = 0U;
static uint8_t task2_finish_line_left = 0U;
static uint8_t task2_finish_detected = 0U;
static int32_t task2_start_encoder_1 = 0;
static int32_t task2_start_encoder_2 = 0;
static int32_t task2_speed_last_encoder_1 = 0;
static int32_t task2_speed_last_encoder_2 = 0;
static int32_t task2_finish_encoder_1 = 0;
static int32_t task2_finish_encoder_2 = 0;
static float task2_profile_speed_mm_s = 0.0f;
static float task2_last_line_error = 0.0f;
static float task2_speed_integral_1 = 0.0f;
static float task2_speed_integral_2 = 0.0f;
static int64_t task2_start_time_ms = 0;
static int64_t task2_brake_start_time_ms = 0;

volatile uint32_t task2_total_time_ms = 0U;
volatile float task2_distance_mm = 0.0f;
volatile float task2_stop_error_mm = 0.0f;
volatile float task2_target_speed_mm_s = 0.0f;
volatile float task2_actual_speed_mm_s = 0.0f;
volatile uint8_t task2_finished = 0U;

/* =====================================================================
 * 任务 3：对应赛题第 4 问的“小车 A 点出发并通过 B 点”部分
 *
 * AB 的中心线距离为1500 mm。通过B点后继续匀减速到0，确认左右轮
 * 实际速度接近0后再关闭驱动，尽量减小停车冲击。
 * ===================================================================== */
#define TASK3_CRUISE_SPEED_MM_S            (330.0f) 
#define TASK3_FINAL_COAST_SPEED_MM_S         (0.0f) /* 正常停车目标速度：匀减速到0，不保留末段爬行 */
#define TASK3_ACCEL_MM_S2                  (100.0f) /* 更缓慢地起步，减小瞬时加速度  原来150*/ 
#define TASK3_DECEL_MM_S2                   (60.0f) /* 恒定减速度：每10 ms目标速度降低0.6 mm/s */
#define TASK3_PRE_DECEL_DISTANCE_MM       (1450.0f) /* 到此里程开始把目标速度由330 mm/s匀减到0 */
#define TASK3_B_POINT_DISTANCE_MM         (1500.0f) /* A到B的比赛距离；到此锁存A-B用时 */
#define TASK3_STOP_SPEED_THRESHOLD_MM_S     (12.0f) /* 左右实际轮速均低于此值，视为车辆基本停稳 */
#define TASK3_STOP_CONFIRM_TICKS              (3U)  /* 连续确认30 ms，避免低速编码器波动误判 */
#define TASK3_SOFT_STOP_DIFF_RATIO           (0.70f) /* 减速末段最大差速=当前基础速度的70%，防止单轮猛转 */
#define TASK3_COAST_HOLD_MS                 (250U)  /* 确认近零速后关闭驱动，等待250 ms再冻结OLED距离 */
#define TASK3_TIMEOUT_MS                   (8000U)  /* 未通过B点时的安全超时；超时仍使用强刹 */

volatile Task3_State_t task3_state = TASK3_ACCEL;
volatile uint32_t task3_total_time_ms = 0U;
volatile float task3_distance_mm = 0.0f;
volatile uint8_t task3_passed_b = 0U;
volatile uint8_t task3_finished = 0U;

static uint8_t task3_initialized = 0U;
static uint8_t task3_speed_sample_tick = 0U;
static uint8_t task3_lost_line_count = 0U;
static int32_t task3_start_encoder_1 = 0;
static int32_t task3_start_encoder_2 = 0;
static int32_t task3_speed_last_encoder_1 = 0;
static int32_t task3_speed_last_encoder_2 = 0;
static float task3_profile_speed_mm_s = 0.0f;
static float task3_last_line_error = 0.0f;
static float task3_speed_integral_1 = 0.0f;
static float task3_speed_integral_2 = 0.0f;
static int64_t task3_start_time_ms = 0;
static int64_t task3_brake_start_time_ms = 0;
static uint8_t task3_normal_coast_stop = 0U;
static uint8_t task3_zero_speed_confirm_count = 0U;

/* =====================================================================
 * 任务 4：合并赛题第 5、6 问的小车循迹部分
 *
 * 小车从A点出发，顺时针循迹一圈并再次通过A点。A点通过时锁存比赛
 * 时间，随后匀减速到0，确认实际轮速接近0后再关闭驱动。
 * ===================================================================== */
#define TASK4_CRUISE_SPEED_MM_S             (300.0f)
#define TASK4_CURVE_MIN_SPEED_MM_S          (240.0f) //
#define TASK4_FINAL_COAST_SPEED_MM_S          (0.0f) /* A点后正常停车目标速度：匀减速到0 */
#define TASK4_ACCEL_MM_S2                   (120.0f)  //加速120.0f
#define TASK4_DECEL_MM_S2                    (60.0f) /* A点后恒定减速度：每10 ms目标速度降低0.6 mm/s */
#define TASK4_CURVE_SLOWDOWN_MM_S             (8.0f)
#define TASK4_FINISH_ARM_DISTANCE_MM       (5000.0f)
#define TASK4_MARKER_MIN_BLACK_SENSORS        (4U) /* A点横线至少3路黑色即判定候选 */
#define TASK4_STOP_SPEED_THRESHOLD_MM_S      (12.0f) /* 左右实际轮速均低于此值后才关闭驱动 */
#define TASK4_STOP_CONFIRM_TICKS               (3U)  /* 连续确认30 ms，避免低速测速波动误判 */
#define TASK4_SOFT_STOP_DIFF_RATIO            (0.70f) /* A点后最大差速=当前基础速度的70%，避免停车前猛转 */
#define TASK4_COAST_HOLD_MS                  (250U)  /* 确认近零速后关闭驱动，等待250 ms再冻结OLED距离 */
#define TASK4_TIMEOUT_MS                  (30000U)  /* 未再次通过A点时的安全超时；超时仍使用强刹 */

volatile Task4_State_t task4_state = TASK4_LEAVE_START;
volatile uint32_t task4_total_time_ms = 0U;
volatile float task4_distance_mm = 0.0f;
volatile uint8_t task4_passed_a = 0U;
volatile uint8_t task4_finished = 0U;

static uint8_t task4_initialized = 0U;
static uint8_t task4_marker_confirm_count = 0U;
static uint8_t task4_lost_line_count = 0U;
static uint8_t task4_speed_sample_tick = 0U;
static uint8_t task4_start_line_left = 0U;
static int32_t task4_start_encoder_1 = 0;
static int32_t task4_start_encoder_2 = 0;
static int32_t task4_speed_last_encoder_1 = 0;
static int32_t task4_speed_last_encoder_2 = 0;
static float task4_profile_speed_mm_s = 0.0f;
static float task4_last_line_error = 0.0f;
static float task4_speed_integral_1 = 0.0f;
static float task4_speed_integral_2 = 0.0f;
static int64_t task4_start_time_ms = 0;
static int64_t task4_brake_start_time_ms = 0;
static uint8_t task4_normal_coast_stop = 0U;
static uint8_t task4_zero_speed_confirm_count = 0U;

// =====================================================================
// adjust_head() — 陀螺仪PI航向闭环控制
//
// 原理:
//   用陀螺仪yaw角作为反馈，PI控制器输出左右轮差速，
//   驱动小车转向，使车头始终对准 yaw_start 这个目标角度。
//
// 控制流程:
//   ① 计算 yaw_error = 当前yaw - 目标yaw_start
//      yaw_error > 0 → 车头偏右 → 需要左转(yaw减小)
//      yaw_error < 0 → 车头偏左 → 需要右转(yaw增大)
//
//   ② 360°跳变处理:
//      例: 当前355°, 目标5° → 误差=350°, 实际只需转10°
//      处理: 误差>180 → 误差-=360 → 350→-10 ✓
//
//   ③ P项(比例): pwm_diff_half = (yaw_error + 积分) * yaw_k_p
//      偏角越大 → 差速越大 → 回正越快
//      yaw_k_p=9.0: 偏1° → 差速9, 偏50° → 差速450(限幅400)
//
//   ④ I项(积分): yaw_total_error += yaw_k_i * yaw_error (每10ms)
//      持续小偏差累积积分 → 慢慢加力消除稳态误差
//      yaw_k_i=0.03: 偏1°持续1秒 → 累积差速=0.03*1*100=3
//      死区: |yaw_error|<0.5° → 积分清零，防止小偏差震荡
//
//   ⑤ 差速限幅: ±MAX_PWM_DIFF=550, 防止转弯过猛
//
//   ⑥ 输出到电机:
//      左轮 = yaw_pwm_base - pwm_diff_half
//      右轮 = yaw_pwm_base + pwm_diff_half
//      yaw_pwm_base=800, diff=400 → 左400(慢) 右1200(快) → 右转
//      yaw_pwm_base=0,   diff=400 → 左0(停)   右400(转) → 原地右转
//
// 例: yaw_start=0°, 当前yaw=-30°(330°)
//     yaw_error = 330-0=330 → >180 → 330-360=-30°
//     pwm_diff = -30 * 9.0 = -270
//     左轮 = 800 - (-270) = 1070 (快), 右轮 = 800 + (-270) = 530 (慢)
//     左快右慢 → 车往左转(yaw增大) → 从330°转回0°
// =====================================================================
//P:偏得多 -> 大理拉回来(快)
//I:偏得少但一直偏 -> 慢慢加力，直到对准(准)
void adjust_head()
{
    float yaw_error = current_attitude.yaw - yaw_start;
    //处理360°/0°的跳变
    if(yaw_error > 180) yaw_error -= 360;
    else if(yaw_error < -180) yaw_error += 360;

    int pwm_diff_half = 0;
    //P计算
    if(yaw_error > 0.5 || yaw_error < -0.5) pwm_diff_half = (yaw_error + yaw_total_error) * yaw_k_p;
    //偏差小于0.5度时，积分清零，防止积分过大导致震荡
    else yaw_total_error = 0;
    //限幅
    if(pwm_diff_half < -MAX_PWM_DIFF) pwm_diff_half = -MAX_PWM_DIFF;
    else if(pwm_diff_half > MAX_PWM_DIFF) pwm_diff_half = MAX_PWM_DIFF;

    //输出到电机上，让电机根据陀螺仪根据相应的转动
    //电机1转速 = 基准转速 - 差速的一半
    motor_set_duty(1, limit_duty(yaw_pwm_base - pwm_diff_half));
    //电机2转速 = 基准转速 + 差速的一半
    motor_set_duty(2, limit_duty(yaw_pwm_base + pwm_diff_half));


    //积分计算 每10ms计算一次积分，积分系数是0.03，偏差1度 -> 差速加0.03
    yaw_total_error += yaw_k_i * yaw_error;
}


//定时器中断服务函数，10ms一次
void MOTOR_PID_INST_IRQHandler()
{
    switch (DL_Timer_getPendingInterrupt(MOTOR_PID_INST))
    {
    case DL_TIMER_IIDX_LOAD:

        // 执行任务1，遇到黑线，停止，有蜂鸣器叫
        // ===== 电机测试模式: 两轮直行, 检查哪个电机接反 =====
        // if(task == 1 && is_start == 1)
        // {
        //     motor_set_direction(1, 1);   // 电机1 正转
        //     motor_set_direction(2, 1);   // 电机2 正转
        //     motor_set_duty(1, 500);      // 左轮 低速
        //     motor_set_duty(2, 500);      // 右轮 低速
        // }

        // ===== 陀螺仪测试模式: 晃动车头, 观察轮子反应 =====
        // 车架起来(轮子悬空) → KEY2启动 → 手动转车身 → 看轮子
        //   顺时针转车 → 左轮快、右轮慢 (车往左回正)
        //   逆时针转车 → 左轮慢、右轮快 (车往右回正)
        //   不动       → 两轮同速
        // if(task == 1 && is_start == 1)
        // {
        //     // motor_set_direction(1, 1);    // 两轮正转
        //     // motor_set_direction(2, 1);
        //     // yaw_pwm_base = 500;           // 基准速度(低速, 方便观察差速)
        //     // adjust_head();                // 陀螺仪闭环, 自动维持 yaw_start 方向
        // }

        /* ============================================================
         * 任务 1：自动直行 1500 mm 编码器测距测试。
         *
         * 使用方法：
         * 1. KEY1 切换到任务 1，把车放在直线起点；
         * 2. KEY2 启动，车辆缓慢加速到 200 mm/s；
         * 3. 1300 mm 后减速，编码器平均距离达到 1500 mm 时短刹；
         * 4. 停车后 OLED 保留最终时间、距离和左右轮原始计数。
         *
         * 本任务只测试编码器测距和两轮速度闭环，不读取循迹传感器。
         * ============================================================ */
#if 0
        if (task == 1U)
        {
            int32_t task1_delta_1;
            int32_t task1_delta_2;
            int32_t task1_abs_delta_1;
            int32_t task1_abs_delta_2;
            int32_t task1_speed_delta_1;
            int32_t task1_speed_delta_2;
            uint8_t task1_speed_updated = 0U;
            uint8_t task1_drive_enabled = 1U;
            float task1_raw_speed_1;
            float task1_raw_speed_2;
            float task1_desired_speed;
            float task1_speed_error_1;
            float task1_speed_error_2;
            float task1_pwm_float_1;
            float task1_pwm_float_2;
            int task1_pwm_1;
            int task1_pwm_2;
            int64_t task1_now_ms = get_time_stamp_ms();

            /* 进入任务 1 或再次按 KEY2 时，以当前编码器位置建立新零点。 */
            if (task1_encoder_reset_requested != 0U)
            {
                task1_encoder_zero_1 = encoder_total_count_1;
                task1_encoder_zero_2 = encoder_total_count_2;
                task1_speed_last_encoder_1 = encoder_total_count_1;
                task1_speed_last_encoder_2 = encoder_total_count_2;
                task1_encoder_count_1 = 0;
                task1_encoder_count_2 = 0;
                task1_encoder_distance_1_mm = 0.0f;
                task1_encoder_distance_2_mm = 0.0f;
                task1_distance_mm = 0.0f;
                task1_total_time_ms = 0U;
                task1_finished = 0U;
                task1_state = TASK1_ACCEL;
                task1_profile_speed_mm_s = 0.0f;
                task1_speed_integral_1 = 0.0f;
                task1_speed_integral_2 = 0.0f;
                task1_speed_sample_tick = 0U;
                task1_initialized = 0U;
                task1_encoder_reset_requested = 0U;
            }

            if (is_start == 1U)
            {
                if (task1_initialized == 0U)
                {
                    task1_start_time_ms = last_change_time;
                    task1_brake_start_time_ms = 0;
                    speed_1 = 0.0f;
                    speed_2 = 0.0f;
                    target_speed_1 = 0.0f;
                    target_speed_2 = 0.0f;
                    motor_set_direction(1U, 1U);
                    motor_set_direction(2U, 1U);
                    motor_set_duty(1U, 0U);
                    motor_set_duty(2U, 0U);
                    task1_initialized = 1U;
                }

                task1_delta_1 = encoder_total_count_1 - task1_encoder_zero_1;
                task1_delta_2 = encoder_total_count_2 - task1_encoder_zero_2;
                task1_encoder_count_1 = task1_delta_1;
                task1_encoder_count_2 = task1_delta_2;

                /* 电机镜像安装时计数符号可能相反，测距分别取绝对值再求平均。 */
                task1_abs_delta_1 = (task1_delta_1 < 0) ? -task1_delta_1 : task1_delta_1;
                task1_abs_delta_2 = (task1_delta_2 < 0) ? -task1_delta_2 : task1_delta_2;
                task1_encoder_distance_1_mm =
                    (float)task1_abs_delta_1 * TASK1_ENCODER_MM_PER_COUNT;
                task1_encoder_distance_2_mm =
                    (float)task1_abs_delta_2 * TASK1_ENCODER_MM_PER_COUNT;
                task1_distance_mm = (task1_encoder_distance_1_mm +
                                     task1_encoder_distance_2_mm) * 0.5f;
                task1_total_time_ms =
                    (uint32_t)(task1_now_ms - task1_start_time_ms);

                /* 每 20 ms 用编码器增量测速，并低通滤波后送入速度 PI。 */
                task1_speed_sample_tick++;
                if (task1_speed_sample_tick >= TASK1_SPEED_SAMPLE_TICKS)
                {
                    task1_speed_sample_tick = 0U;
                    task1_speed_delta_1 =
                        encoder_total_count_1 - task1_speed_last_encoder_1;
                    task1_speed_delta_2 =
                        encoder_total_count_2 - task1_speed_last_encoder_2;
                    task1_speed_last_encoder_1 = encoder_total_count_1;
                    task1_speed_last_encoder_2 = encoder_total_count_2;
                    if (task1_speed_delta_1 < 0) task1_speed_delta_1 = -task1_speed_delta_1;
                    if (task1_speed_delta_2 < 0) task1_speed_delta_2 = -task1_speed_delta_2;

                    task1_raw_speed_1 = (float)task1_speed_delta_1 *
                                        TASK1_ENCODER_MM_PER_COUNT / 0.02f;
                    task1_raw_speed_2 = (float)task1_speed_delta_2 *
                                        TASK1_ENCODER_MM_PER_COUNT / 0.02f;
                    speed_1 = TASK1_SPEED_FILTER_ALPHA * task1_raw_speed_1 +
                              (1.0f - TASK1_SPEED_FILTER_ALPHA) * speed_1;
                    speed_2 = TASK1_SPEED_FILTER_ALPHA * task1_raw_speed_2 +
                              (1.0f - TASK1_SPEED_FILTER_ALPHA) * speed_2;
                    task1_speed_updated = 1U;
                }

                if ((task1_state == TASK1_ACCEL) &&
                    (task1_profile_speed_mm_s >= TASK1_CRUISE_SPEED_MM_S - 1.0f))
                {
                    task1_state = TASK1_CRUISE;
                }
                if ((task1_state != TASK1_BRAKING) &&
                    (task1_distance_mm >= TASK1_PRE_DECEL_DISTANCE_MM))
                {
                    task1_state = TASK1_PRE_DECEL;
                }

                /* 到达 1500 mm 或超过 15 s 均立即短刹，防止车辆继续运行。 */
                if (((task1_distance_mm >= TASK1_TARGET_DISTANCE_MM) ||
                     (task1_total_time_ms >= TASK1_TIMEOUT_MS)) &&
                    (task1_state != TASK1_BRAKING))
                {
                    target_speed_1 = 0.0f;
                    target_speed_2 = 0.0f;
                    task1_speed_integral_1 = 0.0f;
                    task1_speed_integral_2 = 0.0f;
                    motor_set_direction(1U, 0U);
                    motor_set_direction(2U, 0U);
                    motor_set_duty(1U, TASK1_BRAKE_DUTY);
                    motor_set_duty(2U, TASK1_BRAKE_DUTY);
                    task1_brake_start_time_ms = task1_now_ms;
                    task1_state = TASK1_BRAKING;
                    task1_drive_enabled = 0U;
                }

                /* 保持 80 ms 短刹；期间仍更新编码器距离，最终值包含停车惯性。 */
                if (task1_state == TASK1_BRAKING)
                {
                    task1_drive_enabled = 0U;
                    if ((uint32_t)(task1_now_ms - task1_brake_start_time_ms) >=
                        TASK1_BRAKE_HOLD_MS)
                    {
                        motor_set_duty(1U, 0U);
                        motor_set_duty(2U, 0U);
                        task1_state = TASK1_FINISHED;
                        task1_finished = 1U;
                        task1_initialized = 0U;
                        is_start = 0U;
                    }
                }

                if (task1_drive_enabled != 0U)
                {
                    task1_desired_speed =
                        (task1_state == TASK1_PRE_DECEL) ?
                        TASK1_APPROACH_SPEED_MM_S : TASK1_CRUISE_SPEED_MM_S;

                    /* 梯形速度轨迹：缓加速、匀速、提前减速，避免突然冲车。 */
                    if (task1_profile_speed_mm_s < task1_desired_speed)
                    {
                        task1_profile_speed_mm_s +=
                            TASK1_ACCEL_MM_S2 * TASK1_CONTROL_PERIOD_S;
                        if (task1_profile_speed_mm_s > task1_desired_speed)
                            task1_profile_speed_mm_s = task1_desired_speed;
                    }
                    else if (task1_profile_speed_mm_s > task1_desired_speed)
                    {
                        task1_profile_speed_mm_s -=
                            TASK1_DECEL_MM_S2 * TASK1_CONTROL_PERIOD_S;
                        if (task1_profile_speed_mm_s < task1_desired_speed)
                            task1_profile_speed_mm_s = task1_desired_speed;
                    }

                    target_speed_1 = task1_profile_speed_mm_s;
                    target_speed_2 = task1_profile_speed_mm_s;

                    /* 两个电机各自闭环到相同轮速，使车辆尽量保持直行。 */
                    if (task1_speed_updated != 0U)
                    {
                        task1_speed_error_1 = target_speed_1 - speed_1;
                        task1_speed_error_2 = target_speed_2 - speed_2;
                        task1_speed_integral_1 += task1_speed_error_1 * 0.02f;
                        task1_speed_integral_2 += task1_speed_error_2 * 0.02f;

                        if (task1_speed_integral_1 > TASK1_SPEED_INTEGRAL_LIMIT)
                            task1_speed_integral_1 = TASK1_SPEED_INTEGRAL_LIMIT;
                        if (task1_speed_integral_1 < -TASK1_SPEED_INTEGRAL_LIMIT)
                            task1_speed_integral_1 = -TASK1_SPEED_INTEGRAL_LIMIT;
                        if (task1_speed_integral_2 > TASK1_SPEED_INTEGRAL_LIMIT)
                            task1_speed_integral_2 = TASK1_SPEED_INTEGRAL_LIMIT;
                        if (task1_speed_integral_2 < -TASK1_SPEED_INTEGRAL_LIMIT)
                            task1_speed_integral_2 = -TASK1_SPEED_INTEGRAL_LIMIT;

                        task1_pwm_float_1 = TASK1_SPEED_STATIC_PWM +
                                            TASK1_SPEED_KFF * target_speed_1 +
                                            TASK1_SPEED_KP * task1_speed_error_1 +
                                            TASK1_SPEED_KI * task1_speed_integral_1;
                        task1_pwm_float_2 = TASK1_SPEED_STATIC_PWM +
                                            TASK1_SPEED_KFF * target_speed_2 +
                                            TASK1_SPEED_KP * task1_speed_error_2 +
                                            TASK1_SPEED_KI * task1_speed_integral_2;
                        task1_pwm_1 = (int)task1_pwm_float_1;
                        task1_pwm_2 = (int)task1_pwm_float_2;
                        if (task1_pwm_1 > TASK1_MAX_DRIVE_PWM) task1_pwm_1 = TASK1_MAX_DRIVE_PWM;
                        if (task1_pwm_2 > TASK1_MAX_DRIVE_PWM) task1_pwm_2 = TASK1_MAX_DRIVE_PWM;
                        if (task1_pwm_1 < 0) task1_pwm_1 = 0;
                        if (task1_pwm_2 < 0) task1_pwm_2 = 0;
                        motor_set_duty(1U, (uint32_t)task1_pwm_1);
                        motor_set_duty(2U, (uint32_t)task1_pwm_2);
                    }
                }
            }
            else
            {
                /* 未启动或已经完成时保持停车，同时保留最终显示数据。 */
                motor_set_duty(1U, 0U);
                motor_set_duty(2U, 0U);
                task1_initialized = 0U;
            }
        }
#endif

        /* Task 1 reads the sensors every 10 ms, even while the car is stopped. */
        if (task == 1U)
        {
            uint32_t error_count_before;
            uint32_t error_count_after;
            int32_t weighted_sum = 0;
            int32_t diff_pwm = 0;
            int16_t error_x10 = 0;
            int16_t derivative_x10 = 0;
            uint8_t line_count = 0U;
            uint8_t sensor_ok;
            uint8_t i;

            error_count_before = Eight_Road_IR_GetErrorCount();
            huidu_get_value();
            error_count_after = Eight_Road_IR_GetErrorCount();
            sensor_ok = (uint8_t)((Eight_Road_IR_IsReady() != 0U) &&
                                  (error_count_after == error_count_before));

            if (sensor_ok != 0U)
            {
                for (i = 0U; i < 8U; i++)
                {
                    if (huidu_value[i] != 0U)
                    {
                        weighted_sum += task1_line_weight[i];
                        line_count++;
                    }
                }

                if (line_count != 0U)
                {
                    error_x10 = (int16_t)((weighted_sum * 10) / line_count);
                }
            }

            task1_line_sensor_ok = sensor_ok;
            task1_line_count = line_count;
            task1_line_error_x10 = error_x10;

            if ((is_start != 0U) && (sensor_ok != 0U) && (line_count != 0U))
            {
                int pwm_1;
                int pwm_2;

                if (task1_line_controller_active != 0U)
                {
                    derivative_x10 = (int16_t)(error_x10 - task1_line_last_error_x10);
                }
                else
                {
                    task1_line_controller_active = 1U;
                }
                task1_line_last_error_x10 = error_x10;

                diff_pwm = ((TASK1_LINE_TEST_KP * (int32_t)error_x10) +
                            (TASK1_LINE_TEST_KD * (int32_t)derivative_x10)) / 10;

#if TASK1_LINE_TEST_REVERSE_STEERING
                diff_pwm = -diff_pwm;
#endif
                if (diff_pwm > TASK1_LINE_TEST_DIFF_LIMIT)
                    diff_pwm = TASK1_LINE_TEST_DIFF_LIMIT;
                else if (diff_pwm < -TASK1_LINE_TEST_DIFF_LIMIT)
                    diff_pwm = -TASK1_LINE_TEST_DIFF_LIMIT;

                pwm_1 = limit_duty(TASK1_LINE_TEST_BASE_PWM + (int)diff_pwm);
                pwm_2 = limit_duty(TASK1_LINE_TEST_BASE_PWM - (int)diff_pwm);

                /* 若上一个任务以STBY滑停，任务1启动时重新使能电机驱动。 */
                DL_GPIO_setPins(DC_MOTOR_STBY_PORT, DC_MOTOR_STBY_PIN);
                motor_set_direction(1U, 1U);
                motor_set_direction(2U, 1U);
                motor_set_duty(1U, (uint32_t)pwm_1);
                motor_set_duty(2U, (uint32_t)pwm_2);

                task1_line_diff_pwm = (int16_t)diff_pwm;
                task1_line_pwm_1 = (uint16_t)pwm_1;
                task1_line_pwm_2 = (uint16_t)pwm_2;
            }
            else
            {
                motor_set_duty(1U, 0U);
                motor_set_duty(2U, 0U);
                task1_line_diff_pwm = 0;
                task1_line_pwm_1 = 0U;
                task1_line_pwm_2 = 0U;
                task1_line_last_error_x10 = 0;
                task1_line_controller_active = 0U;
            }
        }
        else
        {
            task1_line_last_error_x10 = 0;
            task1_line_controller_active = 0U;
        }



        

        // // ===== 原 Task1: A→B 基础循迹 =====
        // if(task == 1 && is_start == 1)
        // {
        //     yaw_pwm_base = 800;
        //     adjust_head();
        //     huidu_get_value();
        //     if(huidu_value[0] == 1 || huidu_value[1] == 1 || huidu_value[2] == 1 || huidu_value[3] == 1 || huidu_value[4] == 1 || huidu_value[5] == 1 || huidu_value[6] == 1 || huidu_value[7] == 1)
        //     {
        //         motor_set_duty(1, 0);
        //         motor_set_duty(2, 0);
        //         is_start = 0;
        //         led_on();
        //         beep_on();
        //         led_beep_off_counter = 30;
        //     }
        // }
        
        /* ============================================================
         * 任务 2：速度规划 + 红外循迹 PD + 左右轮编码器 PI + A 点精停。
         * 控制中断每 10 ms 执行；编码器速度每 20 ms 更新一次。
         * ============================================================ */
        if (task == 2U && is_start == 1U)
        {
            static const int8_t task2_line_weight[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
            int32_t task2_encoder_now_1;
            int32_t task2_encoder_now_2;
            int32_t task2_encoder_delta_1;
            int32_t task2_encoder_delta_2;
            uint8_t task2_line_count = 0U;
            uint8_t task2_marker_black;
            uint8_t task2_speed_updated = 0U;
            uint8_t task2_drive_enabled = 1U;
            uint8_t task2_i;
            int16_t task2_line_weighted_sum = 0;
            int task2_pwm_1;
            int task2_pwm_2;
            float task2_raw_speed_1;
            float task2_raw_speed_2;
            float task2_line_error;
            float task2_line_derivative;
            float task2_line_differential;
            float task2_raw_target_speed_1;
            float task2_raw_target_speed_2;
            float task2_desired_profile_speed;
            float task2_effective_base_speed;
            float task2_curve_slowdown;
            float task2_distance_after_finish_mm;
            float task2_speed_error_1;
            float task2_speed_error_2;
            float task2_pwm_float_1;
            float task2_pwm_float_2;
            int64_t task2_now_ms;

            task2_now_ms = get_time_stamp_ms();
            task2_encoder_now_1 = encoder_total_count_1;
            task2_encoder_now_2 = encoder_total_count_2;

            /* 第一次进入任务：保存编码器起点并清空全部控制状态。 */
            if (task2_initialized == 0U)
            {
                task2_state = TASK2_LEAVE_START;
                task2_marker_confirm_count = 0U;
                task2_lost_line_count = 0U;
                task2_speed_sample_tick = 0U;
                task2_finish_line_left = 0U;
                task2_finish_detected = 0U;
                task2_start_encoder_1 = task2_encoder_now_1;
                task2_start_encoder_2 = task2_encoder_now_2;
                task2_speed_last_encoder_1 = task2_encoder_now_1;
                task2_speed_last_encoder_2 = task2_encoder_now_2;
                task2_finish_encoder_1 = task2_encoder_now_1;
                task2_finish_encoder_2 = task2_encoder_now_2;
                task2_profile_speed_mm_s = 0.0f;
                task2_last_line_error = 0.0f;
                task2_speed_integral_1 = 0.0f;
                task2_speed_integral_2 = 0.0f;
                speed_1 = 0.0f;
                speed_2 = 0.0f;
                target_speed_1 = 0.0f;
                target_speed_2 = 0.0f;
                task2_distance_mm = 0.0f;
                task2_stop_error_mm = 0.0f;
                task2_target_speed_mm_s = 0.0f;
                task2_actual_speed_mm_s = 0.0f;
                task2_total_time_ms = 0U;
                task2_finished = 0U;
                task2_start_time_ms = last_change_time;
                task2_brake_start_time_ms = 0;
                /* 若上一个任务以STBY滑停，任务2启动时重新使能电机驱动。 */
                DL_GPIO_setPins(DC_MOTOR_STBY_PORT, DC_MOTOR_STBY_PIN);
                motor_set_direction(1U, 1U);
                motor_set_direction(2U, 1U);
                motor_set_duty(1U, 0U);
                motor_set_duty(2U, 0U);
                task2_initialized = 1U;
            }

            /*
             * 总里程由正交有符号位置计算。左右电机可能镜像安装，因此分别取
             * 相对起点位移的绝对值，再求平均作为车辆中心行驶距离。
             */
            task2_encoder_delta_1 = task2_encoder_now_1 - task2_start_encoder_1;
            task2_encoder_delta_2 = task2_encoder_now_2 - task2_start_encoder_2;
            if (task2_encoder_delta_1 < 0) task2_encoder_delta_1 = -task2_encoder_delta_1;
            if (task2_encoder_delta_2 < 0) task2_encoder_delta_2 = -task2_encoder_delta_2;
            task2_distance_mm = ((float)task2_encoder_delta_1 +
                                 (float)task2_encoder_delta_2) *
                                0.5f * TASK2_ENCODER_MM_PER_PULSE;
            task2_total_time_ms = (uint32_t)(task2_now_ms - task2_start_time_ms);

            /* 每 20 ms 计算一次轮速，再做一阶低通，减少编码器量化抖动。 */
            task2_speed_sample_tick++;
            if (task2_speed_sample_tick >= TASK2_SPEED_SAMPLE_TICKS)
            {
                task2_speed_sample_tick = 0U;
                task2_encoder_delta_1 = task2_encoder_now_1 - task2_speed_last_encoder_1;
                task2_encoder_delta_2 = task2_encoder_now_2 - task2_speed_last_encoder_2;
                task2_speed_last_encoder_1 = task2_encoder_now_1;
                task2_speed_last_encoder_2 = task2_encoder_now_2;
                if (task2_encoder_delta_1 < 0) task2_encoder_delta_1 = -task2_encoder_delta_1;
                if (task2_encoder_delta_2 < 0) task2_encoder_delta_2 = -task2_encoder_delta_2;

                task2_raw_speed_1 = (float)task2_encoder_delta_1 *
                                    TASK2_ENCODER_MM_PER_PULSE / 0.02f;
                task2_raw_speed_2 = (float)task2_encoder_delta_2 *
                                    TASK2_ENCODER_MM_PER_PULSE / 0.02f;
                speed_1 = TASK2_SPEED_FILTER_ALPHA * task2_raw_speed_1 +
                          (1.0f - TASK2_SPEED_FILTER_ALPHA) * speed_1;
                speed_2 = TASK2_SPEED_FILTER_ALPHA * task2_raw_speed_2 +
                          (1.0f - TASK2_SPEED_FILTER_ALPHA) * speed_2;
                task2_actual_speed_mm_s = (speed_1 + speed_2) * 0.5f;
                task2_speed_updated = 1U;
            }

            /* 读取 8 路循迹传感器并计算加权位置误差。 */
            huidu_get_value();
            for (task2_i = 0U; task2_i < 8U; task2_i++)
            {
                if (huidu_value[task2_i] != 0U)
                {
                    task2_line_weighted_sum += task2_line_weight[task2_i];
                    task2_line_count++;
                }
            }

            if (task2_line_count == 0U)
            {
                task2_lost_line_count++;
                task2_line_error = task2_last_line_error;
            }
            else
            {
                task2_lost_line_count = 0U;
                task2_line_error = (float)task2_line_weighted_sum / (float)task2_line_count;
            }
            task2_line_derivative = task2_line_error - task2_last_line_error;
            task2_last_line_error = task2_line_error;

            /*
             * A 点横线会同时覆盖多路灰度；正常循迹线通常只覆盖 1～3 路。
             * 以后换成 8 路红外时，只要仍把 8 路黑白结果写入 huidu_value[]，
             * 后面的状态机和停车逻辑都不需要改动。
             */
            task2_marker_black =
                (task2_line_count >= TASK2_MARKER_MIN_BLACK_SENSORS) ? 1U : 0U;

            /* 先驶离起点横线；驶离后才允许下一次黑线作为终点。 */
            if ((task2_state == TASK2_LEAVE_START) && (task2_marker_black == 0U))
            {
                task2_finish_line_left = 1U;
                task2_state = TASK2_ACCEL;
            }

            if ((task2_state == TASK2_ACCEL) &&
                (task2_profile_speed_mm_s >= TASK2_CRUISE_SPEED_MM_S - 1.0f))
            {
                task2_state = TASK2_CRUISE;
            }
            if ((task2_state == TASK2_CRUISE) &&
                (task2_distance_mm >= TASK2_PRE_DECEL_DISTANCE_MM))
            {
                task2_state = TASK2_PRE_DECEL;
            }
            if ((task2_state == TASK2_PRE_DECEL) &&
                (task2_profile_speed_mm_s <= TASK2_APPROACH_SPEED_MM_S + 1.0f))
            {
                task2_state = TASK2_FIND_FINISH;
            }

            /* 行驶超过布防里程后才检测终点；横线连续确认20 ms后立即短刹。 */
            if ((task2_finish_line_left != 0U) &&
                (task2_distance_mm >= TASK2_FINISH_ARM_DISTANCE_MM) &&
                (task2_state != TASK2_BRAKING))
            {
                if (task2_marker_black != 0U)
                {
                    if (task2_marker_confirm_count == 0U)
                    {
                        task2_finish_encoder_1 = task2_encoder_now_1;
                        task2_finish_encoder_2 = task2_encoder_now_2;
                    }
                    if (task2_marker_confirm_count < TASK2_MARKER_CONFIRM_TICKS)
                    {
                        task2_marker_confirm_count++;
                    }
                    if (task2_marker_confirm_count >= TASK2_MARKER_CONFIRM_TICKS)
                    {
                        task2_finish_detected = 1U;
                        target_speed_1 = 0.0f;
                        target_speed_2 = 0.0f;
                        task2_target_speed_mm_s = 0.0f;
                        task2_speed_integral_1 = 0.0f;
                        task2_speed_integral_2 = 0.0f;
                        motor_set_direction(1U, 0U);
                        motor_set_direction(2U, 0U);
                        motor_set_duty(1U, TASK2_BRAKE_DUTY);
                        motor_set_duty(2U, TASK2_BRAKE_DUTY);
                        task2_brake_start_time_ms = task2_now_ms;
                        task2_state = TASK2_BRAKING;
                        task2_drive_enabled = 0U;
                    }
                }
                else
                {
                    task2_marker_confirm_count = 0U;
                }
            }

            /*
             * 当前目标就是“传感器确认横线的位置”，不再继续前进补偿距离。
             * 短刹期间继续统计横线后的惯性前移量；stop_error为负表示越过检测点。
             */
            if ((task2_finish_detected != 0U) &&
                (task2_state == TASK2_BRAKING))
            {
                task2_encoder_delta_1 = task2_encoder_now_1 - task2_finish_encoder_1;
                task2_encoder_delta_2 = task2_encoder_now_2 - task2_finish_encoder_2;
                if (task2_encoder_delta_1 < 0) task2_encoder_delta_1 = -task2_encoder_delta_1;
                if (task2_encoder_delta_2 < 0) task2_encoder_delta_2 = -task2_encoder_delta_2;
                task2_distance_after_finish_mm =
                    ((float)task2_encoder_delta_1 +
                     (float)task2_encoder_delta_2) *
                    0.5f * TASK2_ENCODER_MM_PER_PULSE;
                task2_stop_error_mm = -task2_distance_after_finish_mm;
            }

            /* 超时或持续丢线也进入短刹，防止车辆继续盲跑。 */
            if (((task2_total_time_ms > TASK2_TIMEOUT_MS) ||
                 (task2_lost_line_count > TASK2_LOST_LINE_LIMIT_TICKS)) &&
                (task2_state != TASK2_BRAKING))
            {
                target_speed_1 = 0.0f;
                target_speed_2 = 0.0f;
                task2_target_speed_mm_s = 0.0f;
                task2_speed_integral_1 = 0.0f;
                task2_speed_integral_2 = 0.0f;
                motor_set_direction(1U, 0U);
                motor_set_direction(2U, 0U);
                motor_set_duty(1U, TASK2_BRAKE_DUTY);
                motor_set_duty(2U, TASK2_BRAKE_DUTY);
                task2_brake_start_time_ms = task2_now_ms;
                task2_state = TASK2_BRAKING;
                task2_drive_enabled = 0U;
            }

            /* 短刹保持 100 ms，然后停止计时并保存最终显示数据。 */
            if (task2_state == TASK2_BRAKING)
            {
                task2_drive_enabled = 0U;
                if ((uint32_t)(task2_now_ms - task2_brake_start_time_ms) >=
                    TASK2_BRAKE_HOLD_MS)
                {
                    motor_set_duty(1U, 0U);
                    motor_set_duty(2U, 0U);
                    task2_total_time_ms = (uint32_t)(task2_now_ms - task2_start_time_ms);
                    task2_state = TASK2_FINISHED;
                    task2_finished = 1U;
                    task2_initialized = 0U;
                    is_start = 0U;
                }
            }

            if (task2_drive_enabled != 0U)
            {
                /* 根据阶段选择基础速度，再通过加减速度限幅形成梯形速度轨迹。 */
                if ((task2_state == TASK2_PRE_DECEL) ||
                    (task2_state == TASK2_FIND_FINISH))
                {
                    task2_desired_profile_speed = TASK2_APPROACH_SPEED_MM_S;
                }
                else
                {
                    task2_desired_profile_speed = TASK2_CRUISE_SPEED_MM_S;
                }

                if (task2_profile_speed_mm_s < task2_desired_profile_speed)
                {
                    task2_profile_speed_mm_s +=
                        TASK2_ACCEL_MM_S2 * TASK2_CONTROL_PERIOD_S;
                    if (task2_profile_speed_mm_s > task2_desired_profile_speed)
                    {
                        task2_profile_speed_mm_s = task2_desired_profile_speed;
                    }
                }
                else if (task2_profile_speed_mm_s > task2_desired_profile_speed)
                {
                    task2_profile_speed_mm_s -=
                        TASK2_DECEL_MM_S2 * TASK2_CONTROL_PERIOD_S;
                    if (task2_profile_speed_mm_s < task2_desired_profile_speed)
                    {
                        task2_profile_speed_mm_s = task2_desired_profile_speed;
                    }
                }

                /* 偏离中心越多，弯道基础速度越低，但不会影响启动/终点低速段。 */
                task2_curve_slowdown = TASK2_CURVE_SLOWDOWN_MM_S *
                                       ((task2_line_error >= 0.0f) ?
                                        task2_line_error : -task2_line_error);
                task2_effective_base_speed = task2_profile_speed_mm_s;
                if (task2_effective_base_speed > TASK2_CURVE_MIN_SPEED_MM_S)
                {
                    task2_effective_base_speed -= task2_curve_slowdown;
                    if (task2_effective_base_speed < TASK2_CURVE_MIN_SPEED_MM_S)
                    {
                        task2_effective_base_speed = TASK2_CURVE_MIN_SPEED_MM_S;
                    }
                }

                /* 循迹 PD 只生成左右目标速度差，真正的 PWM 由速度 PI 计算。 */
                task2_line_differential = TASK2_LINE_KP * task2_line_error +
                                          TASK2_LINE_KD * task2_line_derivative;
                if (task2_line_differential > TASK2_LINE_DIFF_LIMIT_MM_S)
                    task2_line_differential = TASK2_LINE_DIFF_LIMIT_MM_S;
                if (task2_line_differential < -TASK2_LINE_DIFF_LIMIT_MM_S)
                    task2_line_differential = -TASK2_LINE_DIFF_LIMIT_MM_S;

                task2_raw_target_speed_1 =
                    task2_effective_base_speed + task2_line_differential;
                task2_raw_target_speed_2 =
                    task2_effective_base_speed - task2_line_differential;
                if (task2_raw_target_speed_1 < 0.0f) task2_raw_target_speed_1 = 0.0f;
                if (task2_raw_target_speed_2 < 0.0f) task2_raw_target_speed_2 = 0.0f;
                if (task2_raw_target_speed_1 >
                    TASK2_CRUISE_SPEED_MM_S + TASK2_LINE_DIFF_LIMIT_MM_S)
                    task2_raw_target_speed_1 =
                        TASK2_CRUISE_SPEED_MM_S + TASK2_LINE_DIFF_LIMIT_MM_S;
                if (task2_raw_target_speed_2 >
                    TASK2_CRUISE_SPEED_MM_S + TASK2_LINE_DIFF_LIMIT_MM_S)
                    task2_raw_target_speed_2 =
                        TASK2_CRUISE_SPEED_MM_S + TASK2_LINE_DIFF_LIMIT_MM_S;

                /* 限制循迹修正造成的目标轮速跳变，避免左右电机来回猛拉。 */
                if (task2_raw_target_speed_1 >
                    target_speed_1 + TASK2_WHEEL_TARGET_SLEW_MM_S)
                    target_speed_1 += TASK2_WHEEL_TARGET_SLEW_MM_S;
                else if (task2_raw_target_speed_1 <
                         target_speed_1 - TASK2_WHEEL_TARGET_SLEW_MM_S)
                    target_speed_1 -= TASK2_WHEEL_TARGET_SLEW_MM_S;
                else
                    target_speed_1 = task2_raw_target_speed_1;

                if (task2_raw_target_speed_2 >
                    target_speed_2 + TASK2_WHEEL_TARGET_SLEW_MM_S)
                    target_speed_2 += TASK2_WHEEL_TARGET_SLEW_MM_S;
                else if (task2_raw_target_speed_2 <
                         target_speed_2 - TASK2_WHEEL_TARGET_SLEW_MM_S)
                    target_speed_2 -= TASK2_WHEEL_TARGET_SLEW_MM_S;
                else
                    target_speed_2 = task2_raw_target_speed_2;

                if (target_speed_1 < 0.0f) target_speed_1 = 0.0f;
                if (target_speed_2 < 0.0f) target_speed_2 = 0.0f;
                task2_target_speed_mm_s = (target_speed_1 + target_speed_2) * 0.5f;

                /*
                 * 每次得到新轮速后执行绝对式 PI。
                 * MG513X 使用“静摩擦补偿 + 速度前馈 + PI”：静摩擦补偿只在
                 * 目标速度非零时加入；目标为零时立即清积分并输出 0。
                 */
                if (task2_speed_updated != 0U)
                {
                    task2_speed_error_1 = target_speed_1 - speed_1;
                    task2_speed_error_2 = target_speed_2 - speed_2;

                    if (target_speed_1 > 1.0f)
                        task2_speed_integral_1 += task2_speed_error_1 * 0.02f;
                    else
                        task2_speed_integral_1 = 0.0f;

                    if (target_speed_2 > 1.0f)
                        task2_speed_integral_2 += task2_speed_error_2 * 0.02f;
                    else
                        task2_speed_integral_2 = 0.0f;

                    if (task2_speed_integral_1 > TASK2_SPEED_INTEGRAL_LIMIT)
                        task2_speed_integral_1 = TASK2_SPEED_INTEGRAL_LIMIT;
                    if (task2_speed_integral_1 < -TASK2_SPEED_INTEGRAL_LIMIT)
                        task2_speed_integral_1 = -TASK2_SPEED_INTEGRAL_LIMIT;
                    if (task2_speed_integral_2 > TASK2_SPEED_INTEGRAL_LIMIT)
                        task2_speed_integral_2 = TASK2_SPEED_INTEGRAL_LIMIT;
                    if (task2_speed_integral_2 < -TASK2_SPEED_INTEGRAL_LIMIT)
                        task2_speed_integral_2 = -TASK2_SPEED_INTEGRAL_LIMIT;

                    task2_pwm_float_1 = 0.0f;
                    task2_pwm_float_2 = 0.0f;
                    if (target_speed_1 > 1.0f)
                    {
                        task2_pwm_float_1 = TASK2_SPEED_STATIC_PWM +
                                            TASK2_SPEED_KFF * target_speed_1 +
                                            TASK2_SPEED_KP * task2_speed_error_1 +
                                            TASK2_SPEED_KI * task2_speed_integral_1;
                    }
                    if (target_speed_2 > 1.0f)
                    {
                        task2_pwm_float_2 = TASK2_SPEED_STATIC_PWM +
                                            TASK2_SPEED_KFF * target_speed_2 +
                                            TASK2_SPEED_KP * task2_speed_error_2 +
                                            TASK2_SPEED_KI * task2_speed_integral_2;
                    }
                    task2_pwm_1 = (int)task2_pwm_float_1;
                    task2_pwm_2 = (int)task2_pwm_float_2;
                    if (task2_pwm_1 > TASK2_MAX_DRIVE_PWM) task2_pwm_1 = TASK2_MAX_DRIVE_PWM;
                    if (task2_pwm_2 > TASK2_MAX_DRIVE_PWM) task2_pwm_2 = TASK2_MAX_DRIVE_PWM;
                    if (task2_pwm_1 < 0) task2_pwm_1 = 0;
                    if (task2_pwm_2 < 0) task2_pwm_2 = 0;
                    motor_set_duty(1U, (uint32_t)task2_pwm_1);
                    motor_set_duty(2U, (uint32_t)task2_pwm_2);
                }
            }
        }
        else if (task == 2U)
        {
            /* 未启动时保留最终显示，下一次按键后再重新初始化。 */
            task2_initialized = 0U;
        }
        
        /* ============================================================
         * 任务 3：赛题第 4 问的小车循迹部分
         * A点起步，沿AB直线循迹，1500 mm处通过B点并锁存时间；
         * 随后匀减速到0，实际轮速接近0后关闭驱动。
         * ============================================================ */
        if (task != 3U || is_start != 1U)
        {
            /* 停止状态只解除初始化标志，保留最终时间和距离供 OLED 显示。 */
            task3_initialized = 0U;
        }
        else
        {
            static const int8_t task3_line_weight[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
            int32_t encoder_now_1;
            int32_t encoder_now_2;
            int32_t encoder_delta_1;
            int32_t encoder_delta_2;
            uint8_t line_count = 0U;
            uint8_t speed_updated = 0U;
            uint8_t drive_enabled = 1U;
            uint8_t normal_stop_reached;
            uint8_t safety_stop_requested;
            uint8_t i;
            int16_t line_weighted_sum = 0;
            int pwm_1;
            int pwm_2;
            float raw_speed_1;
            float raw_speed_2;
            float line_error;
            float line_derivative;
            float line_differential;
            float soft_stop_diff_limit;
            float desired_profile_speed;
            float raw_target_speed_1;
            float raw_target_speed_2;
            float speed_error_1;
            float speed_error_2;
            float pwm_float_1;
            float pwm_float_2;
            int64_t now_ms;

            now_ms = get_time_stamp_ms();
            encoder_now_1 = encoder_total_count_1;
            encoder_now_2 = encoder_total_count_2;

            /* 每次按 KEY2 启动任务3时，重新建立编码器零点和控制器状态。 */
            if (task3_initialized == 0U)
            {
                task3_state = TASK3_ACCEL;
                task3_speed_sample_tick = 0U;
                task3_lost_line_count = 0U;
                task3_start_encoder_1 = encoder_now_1;
                task3_start_encoder_2 = encoder_now_2;
                task3_speed_last_encoder_1 = encoder_now_1;
                task3_speed_last_encoder_2 = encoder_now_2;
                task3_profile_speed_mm_s = 0.0f;
                task3_last_line_error = 0.0f;
                task3_speed_integral_1 = 0.0f;
                task3_speed_integral_2 = 0.0f;
                task3_start_time_ms = last_change_time;
                task3_brake_start_time_ms = 0;
                task3_normal_coast_stop = 0U;
                task3_zero_speed_confirm_count = 0U;
                task3_total_time_ms = 0U;
                task3_distance_mm = 0.0f;
                task3_passed_b = 0U;
                task3_finished = 0U;
                speed_1 = 0.0f;
                speed_2 = 0.0f;
                target_speed_1 = 0.0f;
                target_speed_2 = 0.0f;
                /* 上一次正常滑停会拉低STBY；重新启动任务时必须重新使能TB6612。 */
                DL_GPIO_setPins(DC_MOTOR_STBY_PORT, DC_MOTOR_STBY_PIN);
                motor_set_direction(1U, 1U);
                motor_set_direction(2U, 1U);
                motor_set_duty(1U, 0U);
                motor_set_duty(2U, 0U);
                task3_initialized = 1U;
            }

            /* 左右轮可能镜像安装，分别取相对位移绝对值后求平均车体里程。 */
            encoder_delta_1 = encoder_now_1 - task3_start_encoder_1;
            encoder_delta_2 = encoder_now_2 - task3_start_encoder_2;
            if (encoder_delta_1 < 0) encoder_delta_1 = -encoder_delta_1;
            if (encoder_delta_2 < 0) encoder_delta_2 = -encoder_delta_2;
            task3_distance_mm = ((float)encoder_delta_1 +
                                 (float)encoder_delta_2) *
                                0.5f * MOTOR_ENCODER_MM_PER_COUNT;
            /* B点前实时计时；到达B点后冻结A-B比赛时间。 */
            if (task3_passed_b == 0U)
            {
                task3_total_time_ms =
                    (uint32_t)(now_ms - task3_start_time_ms);
            }

            if ((task3_passed_b == 0U) &&
                (task3_distance_mm >= TASK3_B_POINT_DISTANCE_MM))
            {
                task3_passed_b = 1U;
                task3_total_time_ms =
                    (uint32_t)(now_ms - task3_start_time_ms);
            }

            /* 每20 ms计算左右实际轮速，并低通滤波，供速度PI使用。 */
            task3_speed_sample_tick++;
            if (task3_speed_sample_tick >= TASK2_SPEED_SAMPLE_TICKS)
            {
                task3_speed_sample_tick = 0U;
                encoder_delta_1 = encoder_now_1 - task3_speed_last_encoder_1;
                encoder_delta_2 = encoder_now_2 - task3_speed_last_encoder_2;
                task3_speed_last_encoder_1 = encoder_now_1;
                task3_speed_last_encoder_2 = encoder_now_2;
                if (encoder_delta_1 < 0) encoder_delta_1 = -encoder_delta_1;
                if (encoder_delta_2 < 0) encoder_delta_2 = -encoder_delta_2;

                raw_speed_1 = (float)encoder_delta_1 *
                              MOTOR_ENCODER_MM_PER_COUNT / 0.02f;
                raw_speed_2 = (float)encoder_delta_2 *
                              MOTOR_ENCODER_MM_PER_COUNT / 0.02f;
                speed_1 = TASK2_SPEED_FILTER_ALPHA * raw_speed_1 +
                          (1.0f - TASK2_SPEED_FILTER_ALPHA) * speed_1;
                speed_2 = TASK2_SPEED_FILTER_ALPHA * raw_speed_2 +
                          (1.0f - TASK2_SPEED_FILTER_ALPHA) * speed_2;
                speed_updated = 1U;
            }

            /* 八路传感器加权误差：正常细线通常只有中间1～2路检测到黑色。 */
            huidu_get_value();
            for (i = 0U; i < 8U; i++)
            {
                if (huidu_value[i] != 0U)
                {
                    line_weighted_sum += task3_line_weight[i];
                    line_count++;
                }
            }

            if (line_count == 0U)
            {
                task3_lost_line_count++;
                line_error = task3_last_line_error;
            }
            else
            {
                task3_lost_line_count = 0U;
                line_error = (float)line_weighted_sum / (float)line_count;
            }
            line_derivative = line_error - task3_last_line_error;
            task3_last_line_error = line_error;

            if ((task3_state == TASK3_ACCEL) &&
                (task3_profile_speed_mm_s >= TASK3_CRUISE_SPEED_MM_S - 1.0f))
            {
                task3_state = TASK3_CRUISE;
            }
            if (((task3_state == TASK3_ACCEL) ||
                 (task3_state == TASK3_CRUISE)) &&
                (task3_distance_mm >= TASK3_PRE_DECEL_DISTANCE_MM))
            {
                task3_state = TASK3_PRE_DECEL;
            }

            /*
             * 正常停车不再按固定距离突然断驱动。速度轨迹先匀减到0，
             * 再确认左右实际轮速连续30 ms低于阈值，最后关闭TB6612。
             */
            if ((task3_state == TASK3_PRE_DECEL) &&
                (task3_profile_speed_mm_s <= 0.1f) &&
                (speed_1 <= TASK3_STOP_SPEED_THRESHOLD_MM_S) &&
                (speed_2 <= TASK3_STOP_SPEED_THRESHOLD_MM_S))
            {
                if (task3_zero_speed_confirm_count < TASK3_STOP_CONFIRM_TICKS)
                    task3_zero_speed_confirm_count++;
            }
            else
            {
                task3_zero_speed_confirm_count = 0U;
            }
            normal_stop_reached =
                (task3_zero_speed_confirm_count >= TASK3_STOP_CONFIRM_TICKS) ?
                1U : 0U;
            safety_stop_requested =
                ((task3_lost_line_count > TASK2_LOST_LINE_LIMIT_TICKS) ||
                 ((task3_passed_b == 0U) &&
                  (task3_total_time_ms >= TASK3_TIMEOUT_MS))) ? 1U : 0U;

            if (((normal_stop_reached != 0U) ||
                 (safety_stop_requested != 0U)) &&
                (task3_state != TASK3_BRAKING))
            {
                target_speed_1 = 0.0f;
                target_speed_2 = 0.0f;
                task3_speed_integral_1 = 0.0f;
                task3_speed_integral_2 = 0.0f;

                if (safety_stop_requested != 0U)
                {
                    /* 异常保护：保持原来的80 ms强短刹。 */
                    task3_normal_coast_stop = 0U;
                    motor_set_direction(1U, 0U);
                    motor_set_direction(2U, 0U);
                    motor_set_duty(1U, TASK2_BRAKE_DUTY);
                    motor_set_duty(2U, TASK2_BRAKE_DUTY);
                }
                else
                {
                    /* 正常停车：PWM清零并进入STBY高阻态，实现无制动力滑停。 */
                    task3_normal_coast_stop = 1U;
                    motor_set_duty(1U, 0U);
                    motor_set_duty(2U, 0U);
                    DL_GPIO_clearPins(DC_MOTOR_STBY_PORT, DC_MOTOR_STBY_PIN);
                }
                task3_brake_start_time_ms = now_ms;
                task3_state = TASK3_BRAKING;
                drive_enabled = 0U;
            }

            if (task3_state == TASK3_BRAKING)
            {
                drive_enabled = 0U;
                if ((uint32_t)(now_ms - task3_brake_start_time_ms) >=
                    ((task3_normal_coast_stop != 0U) ?
                     TASK3_COAST_HOLD_MS : TASK2_BRAKE_HOLD_MS))
                {
                    motor_set_duty(1U, 0U);
                    motor_set_duty(2U, 0U);
                    /* 未通过B便保护停车时，冻结实际运行至停车的时间。 */
                    if (task3_passed_b == 0U)
                    {
                        task3_total_time_ms =
                            (uint32_t)(now_ms - task3_start_time_ms);
                    }
                    task3_state = TASK3_FINISHED;
                    task3_finished = 1U;
                    /*
                     * 最终时间和最终距离在此锁存。停车后任务3控制块不再运行，
                     * 两个数值会一直保留，直到下一次按KEY2重新启动任务3。
                     */
                    task3_initialized = 0U;
                    is_start = 0U;
                    led_on();
                    beep_on();
                    led_beep_off_counter = 30;
                }
            }

            if (drive_enabled != 0U)
            {
                /* 1450 mm前升到330 mm/s；进入减速段后以60 mm/s^2匀减到0。 */
                desired_profile_speed =
                    (task3_state == TASK3_PRE_DECEL) ?
                    TASK3_FINAL_COAST_SPEED_MM_S : TASK3_CRUISE_SPEED_MM_S;

                if (task3_profile_speed_mm_s < desired_profile_speed)
                {
                    task3_profile_speed_mm_s +=
                        TASK3_ACCEL_MM_S2 * TASK2_CONTROL_PERIOD_S;
                    if (task3_profile_speed_mm_s > desired_profile_speed)
                        task3_profile_speed_mm_s = desired_profile_speed;
                }
                else if (task3_profile_speed_mm_s > desired_profile_speed)
                {
                    task3_profile_speed_mm_s -=
                        TASK3_DECEL_MM_S2 * TASK2_CONTROL_PERIOD_S;
                    if (task3_profile_speed_mm_s < desired_profile_speed)
                        task3_profile_speed_mm_s = desired_profile_speed;
                }

                /* 循迹PD只改变左右目标轮速差，不直接输出PWM。 */
                line_differential = TASK2_LINE_KP * line_error +
                                    TASK2_LINE_KD * line_derivative;
                if (line_differential > TASK2_LINE_DIFF_LIMIT_MM_S)
                    line_differential = TASK2_LINE_DIFF_LIMIT_MM_S;
                if (line_differential < -TASK2_LINE_DIFF_LIMIT_MM_S)
                    line_differential = -TASK2_LINE_DIFF_LIMIT_MM_S;

                /*
                 * 低速减速时同步缩小循迹差速。否则基础速度已接近0，
                 * PD仍可能命令某一侧车轮高速转动，造成停车前突然摆头。
                 */
                if (task3_state == TASK3_PRE_DECEL)
                {
                    soft_stop_diff_limit =
                        task3_profile_speed_mm_s * TASK3_SOFT_STOP_DIFF_RATIO;
                    if (line_differential > soft_stop_diff_limit)
                        line_differential = soft_stop_diff_limit;
                    if (line_differential < -soft_stop_diff_limit)
                        line_differential = -soft_stop_diff_limit;
                }

                raw_target_speed_1 =
                    task3_profile_speed_mm_s + line_differential;
                raw_target_speed_2 =
                    task3_profile_speed_mm_s - line_differential;
                if (raw_target_speed_1 < 0.0f) raw_target_speed_1 = 0.0f;
                if (raw_target_speed_2 < 0.0f) raw_target_speed_2 = 0.0f;

                /* 每10 ms限制左右轮目标变化，降低对独立钢球系统的加速度扰动。 */
                if (raw_target_speed_1 >
                    target_speed_1 + TASK2_WHEEL_TARGET_SLEW_MM_S)
                    target_speed_1 += TASK2_WHEEL_TARGET_SLEW_MM_S;
                else if (raw_target_speed_1 <
                         target_speed_1 - TASK2_WHEEL_TARGET_SLEW_MM_S)
                    target_speed_1 -= TASK2_WHEEL_TARGET_SLEW_MM_S;
                else
                    target_speed_1 = raw_target_speed_1;

                if (raw_target_speed_2 >
                    target_speed_2 + TASK2_WHEEL_TARGET_SLEW_MM_S)
                    target_speed_2 += TASK2_WHEEL_TARGET_SLEW_MM_S;
                else if (raw_target_speed_2 <
                         target_speed_2 - TASK2_WHEEL_TARGET_SLEW_MM_S)
                    target_speed_2 -= TASK2_WHEEL_TARGET_SLEW_MM_S;
                else
                    target_speed_2 = raw_target_speed_2;

                if (target_speed_1 < 0.0f) target_speed_1 = 0.0f;
                if (target_speed_2 < 0.0f) target_speed_2 = 0.0f;

                /* MG513X：静摩擦前馈 + 速度前馈 + 左右轮独立PI。 */
                if (speed_updated != 0U)
                {
                    speed_error_1 = target_speed_1 - speed_1;
                    speed_error_2 = target_speed_2 - speed_2;
                    if (target_speed_1 > 1.0f)
                        task3_speed_integral_1 += speed_error_1 * 0.02f;
                    else
                        task3_speed_integral_1 = 0.0f;
                    if (target_speed_2 > 1.0f)
                        task3_speed_integral_2 += speed_error_2 * 0.02f;
                    else
                        task3_speed_integral_2 = 0.0f;

                    if (task3_speed_integral_1 > TASK2_SPEED_INTEGRAL_LIMIT)
                        task3_speed_integral_1 = TASK2_SPEED_INTEGRAL_LIMIT;
                    if (task3_speed_integral_1 < -TASK2_SPEED_INTEGRAL_LIMIT)
                        task3_speed_integral_1 = -TASK2_SPEED_INTEGRAL_LIMIT;
                    if (task3_speed_integral_2 > TASK2_SPEED_INTEGRAL_LIMIT)
                        task3_speed_integral_2 = TASK2_SPEED_INTEGRAL_LIMIT;
                    if (task3_speed_integral_2 < -TASK2_SPEED_INTEGRAL_LIMIT)
                        task3_speed_integral_2 = -TASK2_SPEED_INTEGRAL_LIMIT;

                    if (target_speed_1 > 1.0f)
                        pwm_float_1 = TASK2_SPEED_STATIC_PWM +
                                      TASK2_SPEED_KFF * target_speed_1 +
                                      TASK2_SPEED_KP * speed_error_1 +
                                      TASK2_SPEED_KI * task3_speed_integral_1;
                    else
                        pwm_float_1 = 0.0f;
                    if (target_speed_2 > 1.0f)
                        pwm_float_2 = TASK2_SPEED_STATIC_PWM +
                                      TASK2_SPEED_KFF * target_speed_2 +
                                      TASK2_SPEED_KP * speed_error_2 +
                                      TASK2_SPEED_KI * task3_speed_integral_2;
                    else
                        pwm_float_2 = 0.0f;
                    pwm_1 = (int)pwm_float_1;
                    pwm_2 = (int)pwm_float_2;
                    if (pwm_1 > TASK2_MAX_DRIVE_PWM) pwm_1 = TASK2_MAX_DRIVE_PWM;
                    if (pwm_2 > TASK2_MAX_DRIVE_PWM) pwm_2 = TASK2_MAX_DRIVE_PWM;
                    if (pwm_1 < 0) pwm_1 = 0;
                    if (pwm_2 < 0) pwm_2 = 0;
                    motor_set_duty(1U, (uint32_t)pwm_1);
                    motor_set_duty(2U, (uint32_t)pwm_2);
                }
            }
        }

        /* ============================================================
         * 任务 4：合并赛题第5、6问的小车循迹部分
         * 一圈再次通过A点时锁存比赛时间，随后匀减速到0再关闭驱动。
         * 不使用陀螺仪、舵机或K230，钢球由独立系统控制。
         * ============================================================ */
        if (task != 4U || is_start != 1U)
        {
            /* 停车后只解除初始化标志，最终时间和距离继续供OLED显示。 */
            task4_initialized = 0U;
        }
        else
        {
            static const int8_t task4_line_weight[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
            int32_t encoder_now_1;
            int32_t encoder_now_2;
            int32_t encoder_delta_1;
            int32_t encoder_delta_2;
            uint8_t line_count = 0U;
            uint8_t marker_black;
            uint8_t speed_updated = 0U;
            uint8_t drive_enabled = 1U;
            uint8_t normal_stop_reached;
            uint8_t safety_stop_requested;
            uint8_t i;
            int16_t line_weighted_sum = 0;
            int pwm_1;
            int pwm_2;
            float raw_speed_1;
            float raw_speed_2;
            float line_error;
            float line_derivative;
            float line_differential;
            float desired_profile_speed;
            float effective_base_speed;
            float curve_slowdown;
            float soft_stop_diff_limit;
            float raw_target_speed_1;
            float raw_target_speed_2;
            float speed_error_1;
            float speed_error_2;
            float pwm_float_1;
            float pwm_float_2;
            int64_t now_ms;

            now_ms = get_time_stamp_ms();
            encoder_now_1 = encoder_total_count_1;
            encoder_now_2 = encoder_total_count_2;

            /* 每次按KEY2启动任务4时，重新建立编码器零点和全部控制状态。 */
            if (task4_initialized == 0U)
            {
                task4_state = TASK4_LEAVE_START;
                task4_marker_confirm_count = 0U;
                task4_lost_line_count = 0U;
                task4_speed_sample_tick = 0U;
                task4_start_line_left = 0U;
                task4_start_encoder_1 = encoder_now_1;
                task4_start_encoder_2 = encoder_now_2;
                task4_speed_last_encoder_1 = encoder_now_1;
                task4_speed_last_encoder_2 = encoder_now_2;
                task4_profile_speed_mm_s = 0.0f;
                task4_last_line_error = 0.0f;
                task4_speed_integral_1 = 0.0f;
                task4_speed_integral_2 = 0.0f;
                task4_start_time_ms = last_change_time;
                task4_brake_start_time_ms = 0;
                task4_normal_coast_stop = 0U;
                task4_zero_speed_confirm_count = 0U;
                task4_total_time_ms = 0U;
                task4_distance_mm = 0.0f;
                task4_passed_a = 0U;
                task4_finished = 0U;
                speed_1 = 0.0f;
                speed_2 = 0.0f;
                target_speed_1 = 0.0f;
                target_speed_2 = 0.0f;
                /* 上一次正常滑停会拉低STBY；重新启动任务时必须重新使能TB6612。 */
                DL_GPIO_setPins(DC_MOTOR_STBY_PORT, DC_MOTOR_STBY_PIN);
                motor_set_direction(1U, 1U);
                motor_set_direction(2U, 1U);
                motor_set_duty(1U, 0U);
                motor_set_duty(2U, 0U);
                task4_initialized = 1U;
            }

            /* 左右编码器相对位移取绝对值，再求平均作为车辆中心累计里程。 */
            encoder_delta_1 = encoder_now_1 - task4_start_encoder_1;
            encoder_delta_2 = encoder_now_2 - task4_start_encoder_2;
            if (encoder_delta_1 < 0) encoder_delta_1 = -encoder_delta_1;
            if (encoder_delta_2 < 0) encoder_delta_2 = -encoder_delta_2;
            task4_distance_mm = ((float)encoder_delta_1 +
                                 (float)encoder_delta_2) *
                                0.5f * MOTOR_ENCODER_MM_PER_COUNT;

            /* A点通过前实时计时；通过后保持整圈时间，不计入额外500 mm。 */
            if (task4_passed_a == 0U)
            {
                task4_total_time_ms =
                    (uint32_t)(now_ms - task4_start_time_ms);
            }

            /* 每20 ms计算左右轮实际速度并进行低通滤波。 */
            task4_speed_sample_tick++;
            if (task4_speed_sample_tick >= TASK2_SPEED_SAMPLE_TICKS)
            {
                task4_speed_sample_tick = 0U;
                encoder_delta_1 = encoder_now_1 - task4_speed_last_encoder_1;
                encoder_delta_2 = encoder_now_2 - task4_speed_last_encoder_2;
                task4_speed_last_encoder_1 = encoder_now_1;
                task4_speed_last_encoder_2 = encoder_now_2;
                if (encoder_delta_1 < 0) encoder_delta_1 = -encoder_delta_1;
                if (encoder_delta_2 < 0) encoder_delta_2 = -encoder_delta_2;

                raw_speed_1 = (float)encoder_delta_1 *
                              MOTOR_ENCODER_MM_PER_COUNT / 0.02f;
                raw_speed_2 = (float)encoder_delta_2 *
                              MOTOR_ENCODER_MM_PER_COUNT / 0.02f;
                speed_1 = TASK2_SPEED_FILTER_ALPHA * raw_speed_1 +
                          (1.0f - TASK2_SPEED_FILTER_ALPHA) * speed_1;
                speed_2 = TASK2_SPEED_FILTER_ALPHA * raw_speed_2 +
                          (1.0f - TASK2_SPEED_FILTER_ALPHA) * speed_2;
                speed_updated = 1U;
            }

            /* 八路红外/灰度统一接口：黑色为1，使用加权平均计算线路偏差。 */
            huidu_get_value();
            for (i = 0U; i < 8U; i++)
            {
                if (huidu_value[i] != 0U)
                {
                    line_weighted_sum += task4_line_weight[i];
                    line_count++;
                }
            }

            if (line_count == 0U)
            {
                task4_lost_line_count++;
                line_error = task4_last_line_error;
            }
            else
            {
                task4_lost_line_count = 0U;
                line_error = (float)line_weighted_sum / (float)line_count;
            }
            line_derivative = line_error - task4_last_line_error;
            task4_last_line_error = line_error;
            marker_black =
                (line_count >= TASK4_MARKER_MIN_BLACK_SENSORS) ? 1U : 0U;

            /* 必须先驶离起点横线，之后才可能把下一次横线识别为终点A。 */
            if ((task4_state == TASK4_LEAVE_START) && (marker_black == 0U))
            {
                task4_start_line_left = 1U;
                task4_state = TASK4_ACCEL;
            }
            if ((task4_state == TASK4_ACCEL) &&
                (task4_profile_speed_mm_s >= TASK4_CRUISE_SPEED_MM_S - 1.0f))
            {
                task4_state = TASK4_CRUISE;
            }
            if (task4_state == TASK4_PASS_A)
            {
                task4_state = TASK4_AFTER_A;
            }

            /*
             * 累计超过5000 mm后才布防A点检测；至少3路黑色并连续确认20 ms。
             * 首次压到横线时保存局部编码器零点，确认后锁存整圈时间。
             */
            if ((task4_start_line_left != 0U) &&
                (task4_passed_a == 0U) &&
                (task4_distance_mm >= TASK4_FINISH_ARM_DISTANCE_MM))
            {
                if (marker_black != 0U)
                {
                    if (task4_marker_confirm_count < TASK2_MARKER_CONFIRM_TICKS)
                    {
                        task4_marker_confirm_count++;
                    }
                    if (task4_marker_confirm_count >= TASK2_MARKER_CONFIRM_TICKS)
                    {
                        task4_passed_a = 1U;
                        task4_total_time_ms =
                            (uint32_t)(now_ms - task4_start_time_ms);
                        task4_state = TASK4_PASS_A;
                    }
                }
                else
                {
                    task4_marker_confirm_count = 0U;
                }
            }

            /*
             * 正常结束：通过A后匀减速到0，实际轮速连续30 ms低于阈值
             * 才关闭驱动，不再按固定距离突然切断。
             * 保护结束：持续丢线300 ms，或A前达到30 s，仍执行强短刹。
             */
            if ((task4_passed_a != 0U) &&
                ((task4_state == TASK4_PASS_A) ||
                 (task4_state == TASK4_AFTER_A)) &&
                (task4_profile_speed_mm_s <= 0.1f) &&
                (speed_1 <= TASK4_STOP_SPEED_THRESHOLD_MM_S) &&
                (speed_2 <= TASK4_STOP_SPEED_THRESHOLD_MM_S))
            {
                if (task4_zero_speed_confirm_count < TASK4_STOP_CONFIRM_TICKS)
                    task4_zero_speed_confirm_count++;
            }
            else
            {
                task4_zero_speed_confirm_count = 0U;
            }
            normal_stop_reached =
                (task4_zero_speed_confirm_count >= TASK4_STOP_CONFIRM_TICKS) ?
                1U : 0U;
            safety_stop_requested =
                ((task4_lost_line_count > TASK2_LOST_LINE_LIMIT_TICKS) ||
                 ((task4_passed_a == 0U) &&
                  (task4_total_time_ms >= TASK4_TIMEOUT_MS))) ? 1U : 0U;

            if (((normal_stop_reached != 0U) ||
                 (safety_stop_requested != 0U)) &&
                (task4_state != TASK4_BRAKING))
            {
                target_speed_1 = 0.0f;
                target_speed_2 = 0.0f;
                task4_speed_integral_1 = 0.0f;
                task4_speed_integral_2 = 0.0f;

                if (safety_stop_requested != 0U)
                {
                    /* 异常保护：保持原来的80 ms强短刹。 */
                    task4_normal_coast_stop = 0U;
                    motor_set_direction(1U, 0U);
                    motor_set_direction(2U, 0U);
                    motor_set_duty(1U, TASK2_BRAKE_DUTY);
                    motor_set_duty(2U, TASK2_BRAKE_DUTY);
                }
                else
                {
                    /* 正常停车：PWM清零并进入STBY高阻态，实现无制动力滑停。 */
                    task4_normal_coast_stop = 1U;
                    motor_set_duty(1U, 0U);
                    motor_set_duty(2U, 0U);
                    DL_GPIO_clearPins(DC_MOTOR_STBY_PORT, DC_MOTOR_STBY_PIN);
                }
                task4_brake_start_time_ms = now_ms;
                task4_state = TASK4_BRAKING;
                drive_enabled = 0U;
            }

            if (task4_state == TASK4_BRAKING)
            {
                drive_enabled = 0U;
                if ((uint32_t)(now_ms - task4_brake_start_time_ms) >=
                    ((task4_normal_coast_stop != 0U) ?
                     TASK4_COAST_HOLD_MS : TASK2_BRAKE_HOLD_MS))
                {
                    motor_set_duty(1U, 0U);
                    motor_set_duty(2U, 0U);
                    /* 未通过A就保护停车时，冻结实际运行到停车的时间。 */
                    if (task4_passed_a == 0U)
                    {
                        task4_total_time_ms =
                            (uint32_t)(now_ms - task4_start_time_ms);
                    }
                    task4_state = TASK4_FINISHED;
                    task4_finished = 1U;
                    task4_initialized = 0U;
                    is_start = 0U;
                    led_on();
                    beep_on();
                    led_beep_off_counter = 30;
                }
            }

            if (drive_enabled != 0U)
            {
                /* A点前保持300 mm/s；再次通过A后以60 mm/s^2匀减到0。 */
                desired_profile_speed =
                    ((task4_state == TASK4_PASS_A) ||
                     (task4_state == TASK4_AFTER_A)) ?
                    TASK4_FINAL_COAST_SPEED_MM_S : TASK4_CRUISE_SPEED_MM_S;

                if (task4_profile_speed_mm_s < desired_profile_speed)
                {
                    task4_profile_speed_mm_s +=
                        TASK4_ACCEL_MM_S2 * TASK2_CONTROL_PERIOD_S;
                    if (task4_profile_speed_mm_s > desired_profile_speed)
                        task4_profile_speed_mm_s = desired_profile_speed;
                }
                else if (task4_profile_speed_mm_s > desired_profile_speed)
                {
                    task4_profile_speed_mm_s -=
                        TASK4_DECEL_MM_S2 * TASK2_CONTROL_PERIOD_S;
                    if (task4_profile_speed_mm_s < desired_profile_speed)
                        task4_profile_speed_mm_s = desired_profile_speed;
                }

                /* 弯道偏差越大，基础速度越低；A后200 mm/s阶段不再额外降速。 */
                curve_slowdown = TASK4_CURVE_SLOWDOWN_MM_S *
                                 ((line_error >= 0.0f) ? line_error : -line_error);
                effective_base_speed = task4_profile_speed_mm_s;
                if (effective_base_speed > TASK4_CURVE_MIN_SPEED_MM_S)
                {
                    effective_base_speed -= curve_slowdown;
                    if (effective_base_speed < TASK4_CURVE_MIN_SPEED_MM_S)
                        effective_base_speed = TASK4_CURVE_MIN_SPEED_MM_S;
                }

                line_differential = TASK2_LINE_KP * line_error +
                                    TASK2_LINE_KD * line_derivative;
                if (line_differential > TASK2_LINE_DIFF_LIMIT_MM_S)
                    line_differential = TASK2_LINE_DIFF_LIMIT_MM_S;
                if (line_differential < -TASK2_LINE_DIFF_LIMIT_MM_S)
                    line_differential = -TASK2_LINE_DIFF_LIMIT_MM_S;

                /* A点后的减速末段同步缩小差速，避免低速时单侧电机突然加速。 */
                if ((task4_state == TASK4_PASS_A) ||
                    (task4_state == TASK4_AFTER_A))
                {
                    soft_stop_diff_limit =
                        effective_base_speed * TASK4_SOFT_STOP_DIFF_RATIO;
                    if (line_differential > soft_stop_diff_limit)
                        line_differential = soft_stop_diff_limit;
                    if (line_differential < -soft_stop_diff_limit)
                        line_differential = -soft_stop_diff_limit;
                }

                raw_target_speed_1 = effective_base_speed + line_differential;
                raw_target_speed_2 = effective_base_speed - line_differential;
                if (raw_target_speed_1 < 0.0f) raw_target_speed_1 = 0.0f;
                if (raw_target_speed_2 < 0.0f) raw_target_speed_2 = 0.0f;
                if (raw_target_speed_1 >
                    TASK4_CRUISE_SPEED_MM_S + TASK2_LINE_DIFF_LIMIT_MM_S)
                    raw_target_speed_1 =
                        TASK4_CRUISE_SPEED_MM_S + TASK2_LINE_DIFF_LIMIT_MM_S;
                if (raw_target_speed_2 >
                    TASK4_CRUISE_SPEED_MM_S + TASK2_LINE_DIFF_LIMIT_MM_S)
                    raw_target_speed_2 =
                        TASK4_CRUISE_SPEED_MM_S + TASK2_LINE_DIFF_LIMIT_MM_S;

                /* 目标轮速斜坡限制，减少车辆加速度对独立钢球系统的扰动。 */
                if (raw_target_speed_1 >
                    target_speed_1 + TASK2_WHEEL_TARGET_SLEW_MM_S)
                    target_speed_1 += TASK2_WHEEL_TARGET_SLEW_MM_S;
                else if (raw_target_speed_1 <
                         target_speed_1 - TASK2_WHEEL_TARGET_SLEW_MM_S)
                    target_speed_1 -= TASK2_WHEEL_TARGET_SLEW_MM_S;
                else
                    target_speed_1 = raw_target_speed_1;

                if (raw_target_speed_2 >
                    target_speed_2 + TASK2_WHEEL_TARGET_SLEW_MM_S)
                    target_speed_2 += TASK2_WHEEL_TARGET_SLEW_MM_S;
                else if (raw_target_speed_2 <
                         target_speed_2 - TASK2_WHEEL_TARGET_SLEW_MM_S)
                    target_speed_2 -= TASK2_WHEEL_TARGET_SLEW_MM_S;
                else
                    target_speed_2 = raw_target_speed_2;

                if (target_speed_1 < 0.0f) target_speed_1 = 0.0f;
                if (target_speed_2 < 0.0f) target_speed_2 = 0.0f;

                /* MG513X静摩擦前馈 + 速度前馈 + 左右轮独立PI闭环。 */
                if (speed_updated != 0U)
                {
                    speed_error_1 = target_speed_1 - speed_1;
                    speed_error_2 = target_speed_2 - speed_2;
                    if (target_speed_1 > 1.0f)
                        task4_speed_integral_1 += speed_error_1 * 0.02f;
                    else
                        task4_speed_integral_1 = 0.0f;
                    if (target_speed_2 > 1.0f)
                        task4_speed_integral_2 += speed_error_2 * 0.02f;
                    else
                        task4_speed_integral_2 = 0.0f;

                    if (task4_speed_integral_1 > TASK2_SPEED_INTEGRAL_LIMIT)
                        task4_speed_integral_1 = TASK2_SPEED_INTEGRAL_LIMIT;
                    if (task4_speed_integral_1 < -TASK2_SPEED_INTEGRAL_LIMIT)
                        task4_speed_integral_1 = -TASK2_SPEED_INTEGRAL_LIMIT;
                    if (task4_speed_integral_2 > TASK2_SPEED_INTEGRAL_LIMIT)
                        task4_speed_integral_2 = TASK2_SPEED_INTEGRAL_LIMIT;
                    if (task4_speed_integral_2 < -TASK2_SPEED_INTEGRAL_LIMIT)
                        task4_speed_integral_2 = -TASK2_SPEED_INTEGRAL_LIMIT;

                    if (target_speed_1 > 1.0f)
                        pwm_float_1 = TASK2_SPEED_STATIC_PWM +
                                      TASK2_SPEED_KFF * target_speed_1 +
                                      TASK2_SPEED_KP * speed_error_1 +
                                      TASK2_SPEED_KI * task4_speed_integral_1;
                    else
                        pwm_float_1 = 0.0f;
                    if (target_speed_2 > 1.0f)
                        pwm_float_2 = TASK2_SPEED_STATIC_PWM +
                                      TASK2_SPEED_KFF * target_speed_2 +
                                      TASK2_SPEED_KP * speed_error_2 +
                                      TASK2_SPEED_KI * task4_speed_integral_2;
                    else
                        pwm_float_2 = 0.0f;
                    pwm_1 = (int)pwm_float_1;
                    pwm_2 = (int)pwm_float_2;
                    if (pwm_1 > TASK2_MAX_DRIVE_PWM) pwm_1 = TASK2_MAX_DRIVE_PWM;
                    if (pwm_2 > TASK2_MAX_DRIVE_PWM) pwm_2 = TASK2_MAX_DRIVE_PWM;
                    if (pwm_1 < 0) pwm_1 = 0;
                    if (pwm_2 < 0) pwm_2 = 0;
                    motor_set_duty(1U, (uint32_t)pwm_1);
                    motor_set_duty(2U, (uint32_t)pwm_2);
                }
            }
        }

        // 任务5：矩形巡线，共完成 8 个拐角（两圈）。
        // status=1：正常巡线，沿矩阵当前这一条变走。
        // status=0：检测到丢线后，停车，按陀螺仪目标航向转90°。
        // status=2：已经转到目标方向，低速向前找下一条黑线，连续检测到黑线后，回到status == 1中。
        if (task != 5U || is_start != 1U)
        {
            task5_initialized = 0U;
        }
        else
        {
            if (task5_initialized == 0U)
            {
                // 每次启动任务5只初始化一次，起点默认已压在黑线上。
                corner_count = 0;
                turn_direction = 0;
                yaw_line_entry = current_attitude.yaw;
                yaw_line_exit = yaw_line_entry;
                yaw_total_error = 0.0f;
                yaw_pwm_base = 1200;
                pwm_huidu_base = 800;
                pwm_huidu_diff_half = 0;
                status = 1;
                status_change_counter = TASK5_DEBOUNCE_CNT;
                task5_find_line_count = 0U;
                last_change_time = get_time_stamp_ms();
                task5_initialized = 1U;
            }

            if (status == 1)
            {
                // 状态1：以正常速度沿当前边巡线。
                pwm_huidu_base = 1000;
                adjust_motor_pwm();

                // 八路全白表示丢线；连续多次全白才视为到达拐角，避免误判。
                if (huidu_value[0] == 0 && huidu_value[1] == 0 &&
                    huidu_value[2] == 0 && huidu_value[3] == 0 &&
                    huidu_value[4] == 0 && huidu_value[5] == 0 &&
                    huidu_value[6] == 0 && huidu_value[7] == 0)
                {
                    status_change_counter--;
                }
                else
                {
                    status_change_counter = TASK5_DEBOUNCE_CNT;
                }

                if (status_change_counter <= 0 &&
                    get_time_stamp_ms() - last_change_time > 500)
                {
                    float yaw_delta;

                    motor_set_duty(1, 0);
                    motor_set_duty(2, 0);
                    led_on();
                    beep_on();
                    led_beep_off_counter = 30;
                    yaw_line_exit = current_attitude.yaw;

                    // 记录本段行驶中已经发生的偏航，用于首次判断矩形转向方向。
                    yaw_delta = yaw_line_exit - yaw_line_entry;
                    if (yaw_delta > 180.0f) yaw_delta -= 360.0f;
                    else if (yaw_delta < -180.0f) yaw_delta += 360.0f;

                    if (turn_direction == 0)
                    {
                        if (yaw_delta < -8.0f) turn_direction = 1;
                        else if (yaw_delta > 8.0f) turn_direction = -1;
                        else turn_direction = TASK5_DEFAULT_TURN_DIRECTION;
                    }

                    // 每个拐角的最终航向始终是本段入线航向的正负 90 度。
                    if (turn_direction == 1) yaw_start = yaw_line_entry - 90.0f;
                    else yaw_start = yaw_line_entry + 90.0f;
                    if (yaw_start < 0.0f) yaw_start += 360.0f;
                    else if (yaw_start >= 360.0f) yaw_start -= 360.0f;

                    yaw_total_error = 0.0f;
                    yaw_pwm_base = 0;
                    // 停车后进入状态0，用陀螺仪完成直角转向。
                    status = 0;
                    last_change_time = get_time_stamp_ms();
                }
            }
            else if (status == 0)
            {
                float yaw_error;

                // 状态0：原地差速转向。超时直接停车，防止传感器或陀螺仪异常时持续转圈。
                if (get_time_stamp_ms() - last_change_time > TASK5_TURN_TIMEOUT_MS)
                {
                    motor_set_duty(1, 0);
                    motor_set_duty(2, 0);
                    is_start = 0;
                    task5_initialized = 0U;
                }
                else
                {
                    yaw_pwm_base = 0;
                    adjust_head();
                    yaw_error = current_attitude.yaw - yaw_start;
                    if (yaw_error > 180.0f) yaw_error -= 360.0f;
                    else if (yaw_error < -180.0f) yaw_error += 360.0f;

                    if (yaw_error < TASK5_YAW_TOLERANCE &&
                        yaw_error > -TASK5_YAW_TOLERANCE)
                    {
                        motor_set_duty(1, 0);
                        motor_set_duty(2, 0);
                        yaw_total_error = 0.0f;
                        pwm_huidu_base = TASK5_FIND_LINE_PWM;
                        pwm_huidu_diff_half = 0;
                        task5_find_line_count = 0U;
                        // 已对准下一条边的方向，但仍需前进确认真正压到线。
                        status = 2;
                        last_change_time = get_time_stamp_ms();
                    }
                }
            }
            else if (status == 2)
            {
                // 状态2：以低速找下一条边，防止转到位后立刻把同一拐角重复计数。
                if (get_time_stamp_ms() - last_change_time > TASK5_FIND_LINE_TIMEOUT_MS)
                {
                    motor_set_duty(1, 0);
                    motor_set_duty(2, 0);
                    is_start = 0;
                    task5_initialized = 0U;
                }
                else
                {
                    pwm_huidu_base = TASK5_FIND_LINE_PWM;
                    adjust_motor_pwm();

                    if (huidu_value[0] == 1 || huidu_value[1] == 1 ||
                        huidu_value[2] == 1 || huidu_value[3] == 1 ||
                        huidu_value[4] == 1 || huidu_value[5] == 1 ||
                        huidu_value[6] == 1 || huidu_value[7] == 1)
                    {
                        task5_find_line_count++;
                    }
                    else
                    {
                        task5_find_line_count = 0U;
                    }

                    if (task5_find_line_count >= TASK5_FIND_LINE_DEBOUNCE_CNT)
                    {
                        // 找线连续确认后才算一个拐角完成。
                        corner_count++;
                        if (corner_count >= 8)
                        {
                            motor_set_duty(1, 0);
                            motor_set_duty(2, 0);
                            is_start = 0;
                            task5_initialized = 0U;
                        }
                        else
                        {
                            // 下一条边从当前航向开始，回到正常巡线状态。
                            yaw_line_entry = current_attitude.yaw;
                            yaw_total_error = 0.0f;
                            pwm_huidu_base = 1000;
                            pwm_huidu_diff_half = 0;
                            status = 1;
                            status_change_counter = TASK5_DEBOUNCE_CNT;
                            last_change_time = get_time_stamp_ms();
                        }
                    }
                }
            }
        }

        led_beep_off_counter --;
        if(led_beep_off_counter < 0)
        {
            led_off();
            beep_off();
            led_beep_off_counter = 0;
        }
        break;
    default:
        break;
    }
}

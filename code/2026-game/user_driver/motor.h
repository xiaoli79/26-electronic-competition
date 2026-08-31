#ifndef MOTOR_H
#define MOTOR_H

#define PI 3.14

/*
 * MG513X P28：电机轴 13 线霍尔编码器，减速比 1:28。
 * AB 正交四倍频统计两相全部边沿，输出轴一圈理论计数为 13 × 28 × 4。
 */
#define MOTOR_BIANMAQI 1456
// 实测轮胎外径，单位 mm
#define MOTOR_WHEEL_D 65.0f

/*
 * 编码器距离标定系数：
 * 实测车辆实际行驶 2000 mm 时，OLED 只显示约 1800 mm，
 * 因此实际距离 / 显示距离 = 2000 / 1800 = 1.111。
 * 后续重新标定时，只需要修改此系数，不要改动 AB 四倍频计数 1456。
 */
#define MOTOR_DISTANCE_CALIBRATION 1.111f

/* 一个 AB 四倍频计数对应的实际行驶距离，单位 mm/count。 */
#define MOTOR_ENCODER_MM_PER_COUNT \
    (PI * MOTOR_WHEEL_D / MOTOR_BIANMAQI * MOTOR_DISTANCE_CALIBRATION)

// G3507      TB6612
// PB24 <--> STBY
// PA8 <--> AIN1
// PA9 <--> AIN2
// PA12 <--> PWMA
// GND <--> GND
// 3V3 <--> VCC

// TB6612    电源模块
// VM          12V
// GND         GND

// TB6612    直流电机1
// AO1<--> M+
// AO2<--> M-

// G3507    直流电机1
// PA21 <--> A
// PA22 <--> B
// 3V3 <--> VCC
// GND <--> GND

// 直流电机接线：
// BO1<--> M+
// BO2<--> M-
// PB19 <--> A
// PB20 <--> B
// 3V3 <--> VCC
// GND <--> GND

// G3507    TB6612
// PA13 <--> PWMB
// PA7 <--> BIN2
// PB18 <--> BIN1

// 所有的GND都需要连接在一起

#include "ti_msp_dl_config.h"
#include "huidu.h"
#include "imu601.h"
#include "led_beep.h"
#include "ntb_time.h"

typedef enum {
    TASK1_ACCEL = 0,          // 平滑加速到测试速度
    TASK1_CRUISE = 1,         // 保持测试速度直行
    TASK1_PRE_DECEL = 2,      // 接近 1500 mm 时平滑减速
    TASK1_BRAKING = 3,        // 到达目标距离后执行 TB6612 短刹
    TASK1_FINISHED = 4        // 停车并冻结时间、距离和编码器计数
} Task1_State_t;

typedef enum {
    TASK2_LEAVE_START = 0,   // 驶离起点横线
    TASK2_ACCEL = 1,         // 平滑加速
    TASK2_CRUISE = 2,        // 匀速巡线
    TASK2_PRE_DECEL = 3,     // 按累计里程提前减速
    TASK2_FIND_FINISH = 4,   // 低速等待八路灰度识别 A 点横线
    TASK2_FINAL_APPROACH = 5,// 保留旧状态编号；当前检测横线后不再进入此状态
    TASK2_BRAKING = 6,       // TB6612 短刹
    TASK2_FINISHED = 7       // 停车并冻结显示
} Task2_State_t;

/* 任务 3 对应赛题第 4 问：从 A 点出发，沿线通过 1.5 m 外的 B 点。 */
typedef enum {
    TASK3_ACCEL = 0,          // 平滑加速
    TASK3_CRUISE = 1,         // A-B 直线段匀速循迹
    TASK3_PRE_DECEL = 2,      // 1450 mm后匀减速到0
    TASK3_BRAKING = 3,        // 正常时滑停，丢线/超时时短刹
    TASK3_FINISHED = 4        // 停车并冻结时间、距离
} Task3_State_t;

/* 任务 4 合并赛题第 5、6 问的小车部分：一圈后通过 A 点。 */
typedef enum {
    TASK4_LEAVE_START = 0,   // 驶离起点A横线，防止起点被当成终点
    TASK4_ACCEL = 1,         // 平滑加速
    TASK4_CRUISE = 2,        // 环线匀速循迹
    TASK4_PASS_A = 3,        // 再次识别A横线并锁存整圈时间
    TASK4_AFTER_A = 4,       // 通过A后匀减速到0并等待实际轮速归零
    TASK4_BRAKING = 5,       // 正常时滑停，丢线/超时时短刹
    TASK4_FINISHED = 6       // 停车并冻结显示
} Task4_State_t;

void motor_init(uint8_t motor_id);
void motor_set_duty(uint8_t motor_id, uint32_t duty);
void motor_set_direction(uint8_t motor_id, uint8_t direction);
int limit_duty(int duty);

/*
 * 任务 1：自动直行 1500 mm 编码器测距测试。
 * KEY2 调用 task1_encoder_reset() 并启动，左右轮采用独立速度 PI 闭环；
 * 到达 1500 mm 后短刹，OLED 持续保留最终时间、平均距离和原始计数。
 */
#if 0
void task1_encoder_reset(void);
extern volatile int32_t task1_encoder_count_1;
extern volatile int32_t task1_encoder_count_2;
extern volatile float task1_encoder_distance_1_mm;
extern volatile float task1_encoder_distance_2_mm;
extern volatile uint32_t task1_total_time_ms;
extern volatile float task1_distance_mm;
extern volatile Task1_State_t task1_state;
extern volatile uint8_t task1_finished;
#endif

/* 任务1按 KEY2 时只建立编码器零点，不启动电机。 */
void task1_encoder_reset(void);

/* Task 1 infrared test diagnostics used by the OLED page. */
extern volatile uint8_t task1_line_count;
extern volatile int16_t task1_line_error_x10;
extern volatile int16_t task1_line_diff_pwm;
extern volatile uint16_t task1_line_pwm_1;
extern volatile uint16_t task1_line_pwm_2;
extern volatile uint8_t task1_line_sensor_ok;

/*
 * 任务 2 的计时和停车诊断结果。
 * - task2_total_time_ms：按下启动键到最终停车的总时间；
 * - task2_distance_mm：本次任务由左右编码器平均速度积分得到的累计行驶距离；
 * - task2_stop_error_mm：横线确认后的制动前移量，负值表示越过检测位置的距离。
 */
extern volatile uint32_t task2_total_time_ms;
extern volatile float task2_distance_mm;
extern volatile float task2_stop_error_mm;
extern volatile float task2_target_speed_mm_s;
extern volatile float task2_actual_speed_mm_s;
extern volatile Task2_State_t task2_state;
extern volatile uint8_t task2_finished;

/* 任务 3 的 OLED 和调试数据。 */
extern volatile uint32_t task3_total_time_ms;
extern volatile float task3_distance_mm;
extern volatile Task3_State_t task3_state;
extern volatile uint8_t task3_passed_b;
extern volatile uint8_t task3_finished;

/* 任务 4 的 OLED 和调试数据。 */
extern volatile uint32_t task4_total_time_ms;
extern volatile float task4_distance_mm;
extern volatile Task4_State_t task4_state;
extern volatile uint8_t task4_passed_a;
extern volatile uint8_t task4_finished;

#endif // MOTOR_H

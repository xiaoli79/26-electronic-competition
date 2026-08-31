#ifndef MOTOR_H
#define MOTOR_H

#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

// PA12 PWM   //波形       一个脉冲转一个微步，即0.05625度
// PA13 DIR   //控制方向 : 高低电平分别表示不同转向
// PA14 DCY   //电流衰减:  高电平大扭矩；低电平小扭矩;悬空不接则自动确定电流衰减模式
// PA15 SLP   //休眠:      高电平工作，低电平休眠
// PA16 RST   //复位:      高电平工作

// 一脉冲 0.05625度
// 角速度 = 0.05625度 * 脉冲频率 
// 脉冲频率 = 角速度 / 0.05625度

/* 电机方向。FWD 对应 DIR 低电平，REV 对应 DIR 高电平。 */
typedef enum {
    MOTOR_DIR_FORWARD = 0,
    MOTOR_DIR_REVERSE = 1
} MotorDirection;

/* 初始化电机驱动器；调用一次即可。 */
void Motor_Init(void);

/* 改变方向只应在电机停止后调用。 */
void Motor_SetDirection(MotorDirection direction);

/* 以control_config.h配置的固定速度按指定角度运动；0度不会启动电机。 */
void Motor_MoveAngle(uint32_t angle_deg);
void Motor_Stop(void);

/*
 * 任务2、任务3使用的逻辑绝对角度接口，单位均为0.1度。
 * Motor_SetLogicalZero()只把“当前机械姿态”设为后续STEP计数的0度参考，
 * 不代表摆杆已经物理水平，也不代表小球已经位于Pos=0 cm。
 * FWD使逻辑角度增加，REV使逻辑角度减小。
 */
void Motor_SetLogicalZero(void);
void Motor_MoveToAngle0p1Deg(int16_t target_0p1deg);
int16_t Motor_GetAngle0p1Deg(void);

/* 读取当前状态，供应用层显示和控制。 */
MotorDirection Motor_GetDirection(void);
bool Motor_IsRunning(void);

#endif

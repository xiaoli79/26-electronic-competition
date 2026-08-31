/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include <stdio.h>
#include "uart.h"
#include "key.h"
#include "motor.h"
#include "huidu.h"
#include "ntb_time.h"
#include "K230.H"

extern float target_speed_1;
extern float target_speed_2;
extern float speed_1;
extern float speed_2;
extern uint8_t huidu_value[];

/* 上电从任务 1 开始；停车时按 KEY1 按 1→2→3→4→1 的顺序切换。 */
volatile uint8_t task = 1;
volatile uint8_t is_start = 0;

/* Stable-version debounce: two reads confirm a press; release clears latch. */
static uint8_t key1_stable_count = 0U;
static uint8_t key2_stable_count = 0U;
static uint8_t key1_latched = 0U;
static uint8_t key2_latched = 0U;

/*
 * 沿用2026-extend的累计计数消抖，并增加一次按压锁存：
 * 持续按住只触发一次，完全松手、计数回到0后才允许下一次触发。
 */
/* 旧任务源码仍在 motor.c 中，保留这两个符号以兼容链接，但任务 2 不使用。 */
float yaw_start = 1000;
float yaw_start_init = 1000;

volatile int64_t last_change_time = 0;

#define OLED_REFRESH_INTERVAL_MS 500
#define OLED_TASK1_REFRESH_INTERVAL_MS 100
#define OLED_RECOVERY_INTERVAL_MS 1000

/*
 * K230 完整帧解析结果打印开关：
 * 1：将有效目标帧或校验失败帧打印到调试串口；
 * 0：仅解析数据，不向调试串口打印。
 */
#define K230_UART_DEBUG_ENABLE 0U

/* 1：OLED 显示 K230 通信诊断；0：OLED 显示原来的任务运行信息。 */
#define K230_OLED_DEBUG_ENABLE 0U

int main(void)
{
    SYSCFG_DL_init();
    /* Keys are polled in main; disable their unused GPIO edge interrupts. */
    DL_GPIO_disableInterrupt(KEY_PORT, KEY_KEY1_PIN | KEY_KEY2_PIN);
    DL_GPIO_clearInterruptStatus(KEY_PORT, KEY_KEY1_PIN | KEY_KEY2_PIN);
    /* 当前开放的任务 1、2、3 均不使用 IMU，关闭旧 UART3 CPU 中断。 */
    NVIC_DisableIRQ(IMU601_INST_INT_IRQN);
    /* 在打开 GPIO 中断前读取 AB 初始状态，避免第一次边沿产生错误计数。 */
    encoder_quadrature_init();
    OLED_Init();
    OLED_ColorTurn(0);//0正常显示，1 反色显示
    OLED_DisplayTurn(0);//0正常显示 1 屏幕翻转显示
    /* 在启动 10 ms 电机控制中断前初始化固定地址 0x09 的八路红外模块。 */
    (void)Eight_Road_IR_Init();
    // NVIC_EnableIRQ(PRINT_INST_INT_IRQN);
    // NVIC_EnableIRQ(KEY_INT_IRQN);
    /* GPIOB中断现在只服务右轮AB编码器，按键不再占用GPIO中断。 */
    NVIC_EnableIRQ(DC_MOTOR_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(DC_MOTOR_GPIOA_INT_IRQN);
    DL_ADC12_enableConversions(xuanniu_INST);
    DL_Timer_startCounter(SERVO_INST);
    DL_Timer_setCaptureCompareValue(SERVO_INST, 50, GPIO_SERVO_C1_IDX);
    motor_init(1);
    /* 任务 2 不使用陀螺仪，不调用 IMU601_init()，因此不会开启 UART3 NVIC 中断。 */
    //给K230初始化
    K230_Init();
    // motor_set_duty(1, 2000);
    target_speed_1 = 0;
    target_speed_2 = 0;
    motor_set_direction(1, 1);
    motor_set_direction(2, 1);
    int64_t last_oled_refresh_time = get_time_stamp_ms() - OLED_TASK1_REFRESH_INTERVAL_MS;
    int64_t last_oled_recovery_time = get_time_stamp_ms() - OLED_RECOVERY_INTERVAL_MS;
#if EIGHT_ROAD_IR_RAW_DEBUG_ENABLE
    int64_t last_eight_road_debug_time = get_time_stamp_ms() - 500;
#endif


//串口通信
    K230_Target_t k230_target = {0U, 0U, 0U};
    uint32_t k230_valid_frame_count = 0U;
    uint8_t k230_has_valid_frame = 0U;

#if K230_UART_DEBUG_ENABLE
    K230_InvalidFrame_t k230_invalid_frame;
    char k230_debug_string[96];
#endif
    // char huidu_buf[] = "00000\n";
    while (1) {
        /*
         * 从 UART 中断接收缓冲区中取出原始字节，并按 K230.c 中定义的
         * AA 55 + FOUND + X + Y + CHECKSUM 协议进行解析。
         */
        K230_Process();

#if EIGHT_ROAD_IR_RAW_DEBUG_ENABLE
        if (get_time_stamp_ms() - last_eight_road_debug_time >= 500)
        {
            char eight_road_debug_string[64];
            last_eight_road_debug_time = get_time_stamp_ms();
            snprintf(eight_road_debug_string, sizeof(eight_road_debug_string),
                     "IR8: raw=0x%02X err=%lu\r\n",
                     (unsigned int)Eight_Road_IR_GetRawValue(),
                     (unsigned long)Eight_Road_IR_GetErrorCount());
            UART_send_string(DEBUG_INST, eight_road_debug_string);
        }
#endif

        /* 只有收齐一帧且校验通过时，才打印一次解析结果。 */
        if (K230_GetLatestTarget(&k230_target)) {
            k230_valid_frame_count++;
            k230_has_valid_frame = 1U;

#if K230_UART_DEBUG_ENABLE
            if (k230_target.found != 0U) {
                snprintf(k230_debug_string, sizeof(k230_debug_string),
                         "K230: found, x=%u, y=%u\r\n",
                         k230_target.x, k230_target.y);
            } else {
                snprintf(k230_debug_string, sizeof(k230_debug_string),
                         "K230: no target\r\n");
            }

            UART_send_string(DEBUG_INST, k230_debug_string);
#endif
        }

#if K230_UART_DEBUG_ENABLE
        /*
         * 帧头正确但校验失败时，也输出原始 8 字节，便于核对 K230 协议。
         * 这类数据只用于调试，不能直接拿来控制小车。
         */
        if (K230_GetLatestInvalidFrame(&k230_invalid_frame)) {
            snprintf(k230_debug_string, sizeof(k230_debug_string),
                     "K230: parse failed, raw=%02X %02X %02X %02X "
                     "%02X %02X %02X %02X\r\n",
                     k230_invalid_frame.data[0],
                     k230_invalid_frame.data[1],
                     k230_invalid_frame.data[2],
                     k230_invalid_frame.data[3],
                     k230_invalid_frame.data[4],
                     k230_invalid_frame.data[5],
                     k230_invalid_frame.data[6],
                     k230_invalid_frame.data[7]);
            UART_send_string(DEBUG_INST, k230_debug_string);
        }
#endif


        // huidu_get_value();
        // sprintf(huidu_buf, "%d%d%d%d%d\n", huidu_value[0], huidu_value[1], huidu_value[2], huidu_value[3], huidu_value[4]);
        // UART_send_string(DEBUG_INST, huidu_buf);
        // delay_ms(500);
        // last_change_time = get_time_stamp_ms();
        /*
         * Stable version: input pull-down, pressed=high.
         * Two consecutive reads confirm a press; holding only triggers once.
         */
        if (get_key_state(KEY_KEY1_PIN))
        {
            if (key1_stable_count < 2U) key1_stable_count++;
            if ((key1_stable_count >= 2U) &&
                (key1_latched == 0U) &&
                (is_start == 0U))
            {
                key1_latched = 1U;
                task = (task >= 4U) ? 1U : (uint8_t)(task + 1U);
                motor_set_duty(1U, 0U);
                motor_set_duty(2U, 0U);

                if (task == 1U)
                {
                    task1_encoder_reset();
                    last_change_time = get_time_stamp_ms();
                }
            }
        }
        else
        {
            key1_stable_count = 0U;
            key1_latched = 0U;
        }

        /* KEY2 resets task 1 debug data or starts tasks 2/3/4 once. */
        if (get_key_state(KEY_KEY2_PIN))
        {
            if (key2_stable_count < 2U) key2_stable_count++;
            if ((key2_stable_count >= 2U) && (key2_latched == 0U))
            {
                key2_latched = 1U;
                if (task == 1U)
                {
                    task1_encoder_reset();
                    last_change_time = get_time_stamp_ms();
                }
                else if (((task == 2U) || (task == 3U) || (task == 4U)) &&
                         (is_start == 0U))
                {
                    is_start = 1U;
                    last_change_time = get_time_stamp_ms();
                }
            }
        }
        else
        {
            key2_stable_count = 0U;
            key2_latched = 0U;
        }
        
        if (OLED_IsHealthy() &&
            get_time_stamp_ms() - last_oled_refresh_time >=
                ((task == 1U) ? OLED_TASK1_REFRESH_INTERVAL_MS :
                                OLED_REFRESH_INTERVAL_MS))
        {
            char oled_str[50];

            last_oled_refresh_time = get_time_stamp_ms();

#if K230_OLED_DEBUG_ENABLE
            const char *k230_status;
            uint32_t received_count = K230_GetReceivedByteCount();

            if (received_count == 0U) {
                k230_status = "K230: NO RX";
            } else if (k230_has_valid_frame == 0U) {
                k230_status = "K230: WAIT";
            } else if (k230_target.found != 0U) {
                k230_status = "K230: FOUND";
            } else {
                k230_status = "K230: NO TARGET";
            }

            snprintf(oled_str, sizeof(oled_str), "%-15s", k230_status);
            OLED_ShowString(0, 0, (u8 *)oled_str, 16);

            snprintf(oled_str, sizeof(oled_str), "X:%-5u Y:%-5u",
                     k230_target.x, k230_target.y);
            OLED_ShowString(0, 16, (u8 *)oled_str, 16);

            snprintf(oled_str, sizeof(oled_str), "R:%05lu OK:%04lu",
                     (unsigned long)(received_count % 100000U),
                     (unsigned long)(k230_valid_frame_count % 10000U));
            OLED_ShowString(0, 32, (u8 *)oled_str, 16);

            snprintf(oled_str, sizeof(oled_str), "E:%04lu O:%04lu  ",
                     (unsigned long)(K230_GetChecksumErrorCount() % 10000U),
                     (unsigned long)(K230_GetOverflowCount() % 10000U));
            OLED_ShowString(0, 48, (u8 *)oled_str, 16);
#else
            if (task == 1U) {
                uint8_t ir_bits = Eight_Road_IR_GetValueBits();

                snprintf(oled_str, sizeof(oled_str), "T1 LINE RUN:%u   ",
                         (unsigned int)is_start);
                OLED_ShowString(0, 0, (u8 *)oled_str, 16);

                snprintf(oled_str, sizeof(oled_str),
                         "IR:%u%u%u%u%u%u%u%u     ",
                         (unsigned int)((ir_bits >> 0U) & 0x01U),
                         (unsigned int)((ir_bits >> 1U) & 0x01U),
                         (unsigned int)((ir_bits >> 2U) & 0x01U),
                         (unsigned int)((ir_bits >> 3U) & 0x01U),
                         (unsigned int)((ir_bits >> 4U) & 0x01U),
                         (unsigned int)((ir_bits >> 5U) & 0x01U),
                         (unsigned int)((ir_bits >> 6U) & 0x01U),
                         (unsigned int)((ir_bits >> 7U) & 0x01U));
                OLED_ShowString(0, 16, (u8 *)oled_str, 16);

                snprintf(oled_str, sizeof(oled_str), "N:%u E:%+d       ",
                         (unsigned int)task1_line_count,
                         (int)task1_line_error_x10);
                OLED_ShowString(0, 32, (u8 *)oled_str, 16);

                snprintf(oled_str, sizeof(oled_str), "P:%04u/%04u     ",
                         (unsigned int)task1_line_pwm_1,
                         (unsigned int)task1_line_pwm_2);
                OLED_ShowString(0, 48, (u8 *)oled_str, 16);
            } else if (task == 2U) {
                uint32_t display_time_ms;

                if (is_start != 0U) {
                    display_time_ms = (uint32_t)(get_time_stamp_ms() - last_change_time);
                } else {
                    display_time_ms = task2_total_time_ms;
                }

                /* 任务 2 按统一标准显示任务号和启动标志。 */
                snprintf(oled_str, sizeof(oled_str), "T:2 is_start:%u  ",
                         (unsigned int)is_start);
                OLED_ShowString(0, 0, (u8 *)oled_str, 16);

                snprintf(oled_str, sizeof(oled_str), "time:%02lu.%02lus    ",
                         (unsigned long)(display_time_ms / 1000U),
                         (unsigned long)((display_time_ms % 1000U) / 10U));
                OLED_ShowString(0, 16, (u8 *)oled_str, 16);

                snprintf(oled_str, sizeof(oled_str), "dir:%06.0fmm    ",
                         task2_distance_mm);
                OLED_ShowString(0, 32, (u8 *)oled_str, 16);

                /* 当前标准只使用前三行，清除旧状态、速度和误差显示。 */
                snprintf(oled_str, sizeof(oled_str), "                ");
                OLED_ShowString(0, 48, (u8 *)oled_str, 16);
            } else if (task == 3U) {
                uint32_t display_time_ms;

                /*
                 * B点前显示实时时间；通过B或停车后显示锁存的A-B时间。
                 * task3_distance_mm 同样会在停车后停止更新并一直保留。
                 */
                if ((is_start != 0U) && (task3_passed_b == 0U)) {
                    display_time_ms =
                        (uint32_t)(get_time_stamp_ms() - last_change_time);
                } else {
                    display_time_ms = task3_total_time_ms;
                }

                snprintf(oled_str, sizeof(oled_str), "T:3 is_start:%u  ",
                         (unsigned int)is_start);
                OLED_ShowString(0, 0, (u8 *)oled_str, 16);

                snprintf(oled_str, sizeof(oled_str), "time:%02lu.%02lus    ",
                         (unsigned long)(display_time_ms / 1000U),
                         (unsigned long)((display_time_ms % 1000U) / 10U));
                OLED_ShowString(0, 16, (u8 *)oled_str, 16);

                snprintf(oled_str, sizeof(oled_str), "dir:%06.0fmm    ",
                         task3_distance_mm);
                OLED_ShowString(0, 32, (u8 *)oled_str, 16);

                if (task3_passed_b != 0U) {
                    snprintf(oled_str, sizeof(oled_str), "B:PASS S:2000   ");
                } else {
                    snprintf(oled_str, sizeof(oled_str), "B:WAIT S:2000   ");
                }
                OLED_ShowString(0, 48, (u8 *)oled_str, 16);
            } else if (task == 4U) {
                uint32_t display_time_ms;

                /*
                 * A点通过前显示实时时间；通过A后显示锁存的一圈时间。
                 * 停车后时间和最终编码器距离都会持续保留。
                 */
                if ((is_start != 0U) && (task4_passed_a == 0U)) {
                    display_time_ms =
                        (uint32_t)(get_time_stamp_ms() - last_change_time);
                } else {
                    display_time_ms = task4_total_time_ms;
                }

                snprintf(oled_str, sizeof(oled_str), "T:4 is_start:%u  ",
                         (unsigned int)is_start);
                OLED_ShowString(0, 0, (u8 *)oled_str, 16);

                snprintf(oled_str, sizeof(oled_str), "time:%02lu.%02lus    ",
                         (unsigned long)(display_time_ms / 1000U),
                         (unsigned long)((display_time_ms % 1000U) / 10U));
                OLED_ShowString(0, 16, (u8 *)oled_str, 16);

                snprintf(oled_str, sizeof(oled_str), "dir:%06.0fmm    ",
                         task4_distance_mm);
                OLED_ShowString(0, 32, (u8 *)oled_str, 16);

                if (task4_passed_a != 0U) {
                    snprintf(oled_str, sizeof(oled_str), "A:PASS S:+500   ");
                } else {
                    snprintf(oled_str, sizeof(oled_str), "A:WAIT S:+500   ");
                }
                OLED_ShowString(0, 48, (u8 *)oled_str, 16);
            } else {
                uint32_t generic_time_ms = 0U;

                if (is_start != 0U) {
                    generic_time_ms =
                        (uint32_t)(get_time_stamp_ms() - last_change_time);
                }

                /* 后续任务也沿用相同的前三行格式。 */
                snprintf(oled_str, sizeof(oled_str), "T:%u is_start:%u  ",
                         (unsigned int)task, (unsigned int)is_start);
                OLED_ShowString(0, 0, (u8 *)oled_str, 16);

                snprintf(oled_str, sizeof(oled_str), "time:%02lu.%02lus    ",
                         (unsigned long)(generic_time_ms / 1000U),
                         (unsigned long)((generic_time_ms % 1000U) / 10U));
                OLED_ShowString(0, 16, (u8 *)oled_str, 16);

                snprintf(oled_str, sizeof(oled_str), "dir:%06umm    ", 0U);
                OLED_ShowString(0, 32, (u8 *)oled_str, 16);

                snprintf(oled_str, sizeof(oled_str), "                ");
                OLED_ShowString(0, 48, (u8 *)oled_str, 16);
            }
#endif

            OLED_Refresh();
        }
        else if (!OLED_IsHealthy() &&
                 get_time_stamp_ms() - last_oled_recovery_time >= OLED_RECOVERY_INTERVAL_MS)
        {
            last_oled_recovery_time = get_time_stamp_ms();
            (void)OLED_Recover();
            last_oled_refresh_time = get_time_stamp_ms();
        }

        delay_ms(50);
        // motor_set_direction(1, 1);
        
    }
}

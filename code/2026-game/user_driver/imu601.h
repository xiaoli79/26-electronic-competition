#ifndef IMU601_H
#define IMU601_H

#include "ti_msp_dl_config.h"
#include "uart.h"
#include "delay.h"

#include <stdint.h>

typedef struct {
    float yaw;
    float pitch;
    float roll;
} Attitude_t;

// 接线
// 汇电籽-601   mspm0g3507
// V           5V
// G           GND
// T           PA25
// R           PA26

void IMU601_init();

#endif

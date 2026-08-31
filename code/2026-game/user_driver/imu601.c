#include "imu601.h"

void IMU601_init()
{
    // 这个是固定写法
    const uint8_t IMU_reset[] = {0xAA, 0x55, 0x60, 0x12, 0x00, 0x72};
    UART_send_buffer(IMU601_INST, IMU_reset, sizeof(IMU_reset));
    delay_ms(300);

    // 这个需要根据实际微调
    const uint8_t IMU_cali[] = {0xAA, 0x55, 0x60, 0x14, 0x04, 0x7B, 0x34, 0xB4, 0x43, 0x1E};
    UART_send_buffer(IMU601_INST, IMU_cali, sizeof(IMU_cali));
    delay_ms(300);
    NVIC_EnableIRQ(IMU601_INST_INT_IRQN);
}

uint8_t IMU_RX_buffer[12] = {0};
uint8_t IMU_RX_index = 0;
uint8_t IMU_last_byte = 0;

// AA 55 60 01 06 E0 8C 18 04 2C F7 36

Attitude_t current_attitude;

void parse_attitude_only(const uint8_t *payload, Attitude_t *out_attitude) {
    uint16_t yaw_raw   = (payload[1] << 8) | payload[0];
    int16_t  pitch_raw = (int16_t)((payload[3] << 8) | payload[2]);
    int16_t  roll_raw  = (int16_t)((payload[5] << 8) | payload[4]);
    out_attitude->yaw   = yaw_raw / 100.0f;
    out_attitude->pitch = pitch_raw / 100.0f;
    out_attitude->roll  = roll_raw / 100.0f;
}

void parse_imu601_data()
{
    uint8_t checksum = 0;
    for (int i = 2; i < 11; i++)
    {
        checksum += IMU_RX_buffer[i];
    }
    if(checksum == IMU_RX_buffer[11])
    {
        parse_attitude_only(&IMU_RX_buffer[5], &current_attitude);
    }
}



// 通过 UART串口中断来接收IMU601的数据
void IMU601_INST_IRQHandler()
{
    switch (DL_UART_getPendingInterrupt(IMU601_INST))
    {
    case DL_UART_IIDX_RX:
        {   
            IMU_RX_buffer[IMU_RX_index] = DL_UART_receiveData(IMU601_INST);
            if(IMU_RX_buffer[IMU_RX_index] == 0x55 && IMU_last_byte == 0xAA)
            {
                IMU_RX_index = 2;
                IMU_RX_buffer[0] = 0xAA;
                IMU_RX_buffer[1] = 0x55;
            }
            else
            {
                IMU_RX_index ++;
            }
            IMU_last_byte = IMU_RX_buffer[IMU_RX_index-1];
            if(IMU_RX_index >= 12)
            {
                IMU_RX_index = 0;
                parse_imu601_data();
            }
            break;
        }
    
    default:
        break;
    }
}

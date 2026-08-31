#include "uart.h"


// 发送一个字符/字节
// UART_send_char(UART0, 'A');  // 通过 UART0 发送字符 A
// UART_send_char(UART0, '\r'); // 发送回车
// UART_send_char(UART0, '\n'); // 发送换行
void UART_send_char(UART_Regs *uart, const uint8_t chr)
{
    // 调用底层驱动函数，以阻塞方式发送 chr。
    // 阻塞：在当前字节发送完成前，程序会停在这里等待
    DL_UART_transmitDataBlocking(uart, chr);
}

//发送以'\0'结尾的字符串
//'\0' 是 C 语言字符串的结束标志，数值是 0x00
// char str[] = "Hello";
// 内存的实际内容是： 'H' 'e' 'l' 'l' 'o' '\0'

// 示例
//UART_send_string(UART0, "Hello World!\r\n");
void UART_send_string(UART_Regs *uart, const char *str)
{
    while (*str) {
        UART_send_char(uart, (uint8_t) *str);
        str++;
    }
}

// 发送指定长度的字节数组
// uint8_t message[] = "OK\r\n";
// UART_send_buffer(UART0, message, sizeof(message) - 1);

// 发送二进制协议帧
// uint8_t frame[] = {
//     0xAA,       // 帧头
//     0x55,       // 帧头
//     0x01,       // 命令字
//     0x03,       // 数据长度
//     0x10, 0x20, 0x30, // 数据
//     0xBB        // 校验或帧尾
// };
// UART_send_buffer(UART0, frame, sizeof(frame));

void UART_send_buffer(UART_Regs *uart, const uint8_t *buf, const uint8_t l)
{
    for(int i = 0; i < l; i++)
    {
        UART_send_char(uart, buf[i]);
    }
}

// void PRINT_INST_IRQHandler()
// {
//     switch (DL_UART_getPendingInterrupt(PRINT_INST))
//     {
//     case DL_UART_IIDX_RX:
//         {   
//             uint8_t rec = DL_UART_receiveData(PRINT_INST);
//             UART_send_char(PRINT_INST, rec);
//             break;
//         }
    
//     default:
//         break;
//     }
// }


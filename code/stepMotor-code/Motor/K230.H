#ifndef K230_H
#define K230_H

/*
 * K230 UART0 (PA0/PA1, 115200 bps) 接收接口。
 *
 * 中断只把字节放入环形缓冲区；K230_Process() 在主循环中有限量地取出，
 * 因此 K230 连续输出异常内容时也不会阻塞电机控制或按键扫描。完整且校验
 * 正确的钢球帧会被转换为可读文本，进入独立的电脑转发队列，供 UART3/PA26
 * 无阻塞发送；最新的有效/丢失状态同时提供给应用层局部刷新 OLED。
 *
 * 当前 K230 脚本使用的二进制协议：
 *   AA 55 FOUND POS_L POS_H 00 00 CHECKSUM
 *   POS 是有符号 int16、小端序，单位为 0.1 cm；表示钢球沿水管相对
 *   O 点的一维位置，不是图像像素 X/Y。CHECKSUM 是 FOUND + POS_L +
 *   POS_H 的低 8 位；后两个 00 为保留字节。
 *
 * 保留的 ASCII 解析代码兼容旧格式：
 *      BALL,123,456\n
 *      {"label":"steel_ball","x":123,"y":456}\n
 * 其他 K230 日志、类别、置信度和无关数字都会被丢弃。
 */

#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

#define K230_RX_BUFFER_SIZE  (256U)

/* 启用钢球协议解析；仅用于生成电脑端文本，不会恢复 OLED 显示。 */
#define K230_ENABLE_TARGET_PARSE       (1U)

/*
 * 电脑端默认接收易读文本，例如：
 *   BALL: POS=-3.2 cm\r\n
 * 未识别到钢球的有效帧只更新内部状态，不向电脑发送任何内容，避免
 * "BALL: NOT FOUND" 连续刷屏。
 * 如需抓取协议原始字节，可将此项设为 0U、下一项设为 1U 后重新编译烧录。
 */
#define K230_PC_FORMATTED_OUTPUT_ENABLE (1U)
#define K230_PC_RAW_FORWARD_ENABLE      (0U)

typedef struct {
    bool found;       /* true: 当前帧检测到钢球；false: 协议明确报告未检测到。 */
    int16_t position_0p1cm; /* 相对 O 点的一维位置，单位为 0.1 cm。 */
} K230_Target_t;

/* 必须在 SYSCFG_DL_init() 后调用一次。 */
void K230_Init(void);

/* 每次最多处理有限个已接收字节，不等待后续串口数据。 */
void K230_Process(void);

/* 取出最新的已校验钢球结果；该接口不负责 OLED 显示。 */
bool K230_GetLatestTarget(K230_Target_t *target);

/*
 * 取出一字节待发送到电脑的数据。默认是可读文本；切换
 * K230_PC_RAW_FORWARD_ENABLE 后才是 K230 原始字节。
 */
bool K230_ReadForwardByte(uint8_t *data);

/* 可用于观察接收是否跟不上 K230 输出；不会影响正常控制。 */
uint32_t K230_GetOverflowCount(void);
uint32_t K230_GetForwardOverflowCount(void);
uint32_t K230_GetReceivedByteCount(void);
uint32_t K230_GetRejectedFrameCount(void);

#endif /* K230_H */

#ifndef K230_H
#define K230_H

/*
 * K230 串口接收驱动对外接口
 *
 * 数据流向：
 *   K230 TX -> MCU UART0 RX -> UART0 接收中断
 *            -> 软件环形缓冲区 -> 主循环读取
 *
 * 当前驱动只负责可靠地保存“原始字节流”，不规定帧头、数据长度、
 * 坐标格式或校验方式。视觉协议应在主循环中通过下方读取函数解析。
 */

/* SysConfig 生成的 K230_INST、K230_INST_INT_IRQN 等硬件宏。 */
#include "ti_msp_dl_config.h"

/* bool、size_t、uint8_t/uint16_t/uint32_t 类型定义。 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


/**
 * @brief K230 UART 软件接收缓冲区大小。
 *
 * 环形缓冲区会保留一个空位置来区分“空”和“满”，因此数组大小虽然
 * 是 128 字节，但实际最多保存 127 个尚未读取的字节。
 * 该值必须是 2 的整数次幂，例如 64、128、256。
 */
#define K230_RX_BUFFER_SIZE (128U)
#define K230_FRAME_SIZE     (8U)

/*
 * K230 视觉结果数据帧（固定 8 字节）：
 *
 *   AA 55 FOUND X_L X_H Y_L Y_H CHECKSUM
 *
 * CHECKSUM 的计算方式：
 *   (FOUND + X_L + X_H + Y_L + Y_H) & 0xFF
 */

/** K230 解析完成后提供给主循环的最新视觉结果。 */
typedef struct {
    uint8_t found;  /* 1：当前帧识别到目标；0：当前帧未识别到目标。 */
    uint16_t x;     /* 目标中心的图像 X 坐标（像素）。 */
    uint16_t y;     /* 目标中心的图像 Y 坐标（像素）。 */
} K230_Target_t;

/** 收到帧头但校验失败时，保存供调试查看的原始 8 字节候选帧。 */
typedef struct {
    uint8_t data[K230_FRAME_SIZE];
} K230_InvalidFrame_t;


/**
 * @brief 清空接收状态并开启 K230 UART 接收中断。
 *
 * @note 必须在 SYSCFG_DL_init() 之后调用。
 * @note 通常只需要在 main() 初始化阶段调用一次。
 */
void K230_Init(void);


/**
 * @brief 返回软件接收缓冲区中尚未读取的字节数。
 *
 * @return 当前可以通过 K230_ReadByte() 或 K230_Read() 读出的字节数。
 */
uint16_t K230_Available(void);


/**
 * @brief 非阻塞读取一个字节。
 *
 * @param[out] data 接收字节的存放地址。
 * @return 有数据且读取成功时返回 true；无数据或 data 为 NULL 时返回 false。
 *
 * @note 本函数不会等待 K230 发送数据，适合在主循环中反复调用。
 */
bool K230_ReadByte(uint8_t *data);


/**
 * @brief 非阻塞批量读取当前已经收到的数据。
 *
 * @param[out] buffer    数据存放缓冲区。
 * @param[in]  maxLength 本次最多读取的字节数。
 * @return 实际读取的字节数。
 *
 * @note 返回值可能小于 maxLength；返回 0 表示当前没有数据或参数无效。
 */
size_t K230_Read(uint8_t *buffer, size_t maxLength);


/**
 * @brief 解析当前已经接收的 K230 原始字节。
 *
 * 本函数应在 main() 的 while(1) 中频繁调用。它会寻找帧头、收集完整
 * 数据帧并校验；解析成功后，可通过 K230_GetLatestTarget() 获取结果。
 */
void K230_Process(void);


/**
 * @brief 取出最新的一帧有效视觉结果。
 *
 * @param[out] target 接收目标识别结果的结构体地址。
 * @return 有新的、校验通过的数据帧时返回 true；否则返回 false。
 *
 * @note 成功读取一次后，新数据标志会清除，同一帧不会重复返回。
 */
bool K230_GetLatestTarget(K230_Target_t *target);

/**
 * @brief 获取最近一帧校验失败的原始数据。
 *
 * @param[out] frame 用于接收原始 8 字节候选帧的结构体地址。
 * @return 有新的校验失败帧时返回 true；否则返回 false。
 *
 * @note 该接口用于调试 K230 发送格式或校验规则；成功读取后该帧不再重复返回。
 */
bool K230_GetLatestInvalidFrame(K230_InvalidFrame_t *frame);


/**
 * @brief 返回因软件接收缓冲区已满而丢弃的累计字节数。 查询是否因为处理太慢而丢失数据
 *
 * 若该数值持续增加，说明主循环读取太慢、关中断时间太长，或者缓冲区
 * 太小。此时应加快数据处理速度或适当增大 K230_RX_BUFFER_SIZE。
 */
uint32_t K230_GetOverflowCount(void);

/** @brief 返回 UART 中断累计接收到的原始字节数。 */
uint32_t K230_GetReceivedByteCount(void);

/** @brief 返回因数据帧校验失败而丢弃的累计帧数。 */
uint32_t K230_GetChecksumErrorCount(void);

#endif /* K230_H */

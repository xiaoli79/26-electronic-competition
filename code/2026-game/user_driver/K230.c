#include "K230.H"

/*
 * 缓冲区大小为 2 的整数次幂时，可以用“与运算”代替取模运算：
 *   index = (index + 1) & (size - 1)
 * 例如 size=128 时，MASK=127，索引到达 127 后会自动回绕到 0。
 */
#define K230_RX_BUFFER_MASK (K230_RX_BUFFER_SIZE - 1U)

/* 编译阶段检查缓冲区大小，配置错误时直接停止编译。 */
#if ((K230_RX_BUFFER_SIZE == 0U) || \
     ((K230_RX_BUFFER_SIZE & K230_RX_BUFFER_MASK) != 0U))
#error "K230_RX_BUFFER_SIZE must be a power of two"
#endif

/*
 * 单生产者/单消费者环形缓冲区：
 * - UART 中断是“生产者”，收到数据后只移动写指针 k230RxHead；
 * - 主循环是“消费者”，取走数据后只移动读指针 k230RxTail；
 * 预留一个空位用于区分“缓冲区空”和“缓冲区满”。
 *
 * 判断规则：
 * - head == tail：缓冲区为空；
 * - head 的下一个位置 == tail：缓冲区已满。
 */
/* 保存从 K230 接收到、但主循环还没有读走的原始字节。 */
static uint8_t k230RxBuffer[K230_RX_BUFFER_SIZE];

/* 下一个新字节的写入位置，只在 UART 中断中修改。 */
static volatile uint16_t k230RxHead = 0U;

/* 下一个待读字节的位置，只在主循环读取函数中修改。 */
static volatile uint16_t k230RxTail = 0U;

/* 缓冲区满时被丢弃的新字节总数，用于诊断接收处理是否及时。 */
static volatile uint32_t k230OverflowCount = 0U;

/* UART 硬件实际收到的累计字节数，包括软件缓冲区满时丢弃的字节。 */
static volatile uint32_t k230ReceivedByteCount = 0U;

/* 接收状态机的各个阶段，对应固定数据帧中下一个期待的字节。 */
typedef enum {
    K230_WAIT_HEADER_AA = 0,
    K230_WAIT_HEADER_55,
    K230_READ_FOUND,
    K230_READ_X_LOW,
    K230_READ_X_HIGH,
    K230_READ_Y_LOW,
    K230_READ_Y_HIGH,
    K230_READ_CHECKSUM
} K230_ParseState_t;

/* 以下变量只由主循环中的 K230_Process() 使用，不会在中断中修改。 */
static K230_ParseState_t k230ParseState = K230_WAIT_HEADER_AA;
static uint8_t k230FrameFound = 0U;
static uint8_t k230FrameXLow = 0U;
static uint8_t k230FrameXHigh = 0U;
static uint8_t k230FrameYLow = 0U;
static uint8_t k230FrameYHigh = 0U;

/* 最近一次校验通过的完整视觉结果，以及它是否还未被主循环取走。 */
static K230_Target_t k230LatestTarget = {0U, 0U, 0U};
static bool k230NewTargetAvailable = false;

/* 最近一次校验失败的完整候选帧，供主循环打印原始数据进行排查。 */
static K230_InvalidFrame_t k230LatestInvalidFrame = {{0U}};
static bool k230NewInvalidFrameAvailable = false;

/* 帮助判断 K230 端发送格式或串口参数是否正确。 */
static uint32_t k230ChecksumErrorCount = 0U;

void K230_Init(void)
{
    /* 防止清空软件状态时 UART 中断同时写入缓冲区。 */
    NVIC_DisableIRQ(K230_INST_INT_IRQN);

    k230RxHead = 0U;          /* 写指针回到缓冲区起点。 */
    k230RxTail = 0U;          /* 读指针回到缓冲区起点。 */
    k230OverflowCount = 0U;   /* 清除上一次运行留下的溢出统计。 */
    k230ReceivedByteCount = 0U;
    k230ParseState = K230_WAIT_HEADER_AA;
    k230LatestTarget.found = 0U;
    k230LatestTarget.x = 0U;
    k230LatestTarget.y = 0U;
    k230NewTargetAvailable = false;
    k230NewInvalidFrameAvailable = false;
    k230ChecksumErrorCount = 0U;

    /* 丢弃初始化前已经留在硬件 FIFO 中的旧数据。 */
    while (!DL_UART_isRXFIFOEmpty(K230_INST)) {
        (void) DL_UART_receiveData(K230_INST);
    }

    /* 清除旧的中断挂起标志，避免启用后立即响应无效的历史中断。 */
    NVIC_ClearPendingIRQ(K230_INST_INT_IRQN);

    /*
     * SysConfig 已经打开 UART 外设内部的 RX 中断；这里再打开 NVIC
     * 对应通道后，K230 发来数据才会进入 K230_INST_IRQHandler()。
     */
    NVIC_EnableIRQ(K230_INST_INT_IRQN);
}

uint16_t K230_Available(void)
{
    /* 先读取一份快照，避免计算过程中反复读取 volatile 变量。 */
    uint16_t head = k230RxHead;
    uint16_t tail = k230RxTail;

    /* 与 MASK 运算同时处理了 head 已回绕、tail 尚未回绕的情况。 */
    return (uint16_t) ((head - tail) & K230_RX_BUFFER_MASK);
}

bool K230_ReadByte(uint8_t *data)
{
    uint16_t tail;

    /* 调用者没有提供有效存储地址时，不能写入接收结果。 */
    if (data == NULL) {
        return false;
    }

    tail = k230RxTail;

    /* 读写指针相等表示当前没有任何待读数据。 */
    if (tail == k230RxHead) {
        return false;
    }

    /* 取走当前字节，然后将读指针向前移动并在末尾自动回绕。 */
    *data = k230RxBuffer[tail];
    k230RxTail = (uint16_t) ((tail + 1U) & K230_RX_BUFFER_MASK);
    return true;
}

size_t K230_Read(uint8_t *buffer, size_t maxLength)
{
    size_t count = 0U;

    /* 无效目标地址不能接收数据。maxLength 为 0 时下面循环自然不执行。 */
    if (buffer == NULL) {
        return 0U;
    }

    /*
     * 逐字节读取，直到达到调用者要求的长度，或者软件缓冲区已经为空。
     * 整个过程不等待新数据，所以不会阻塞主循环。
     */
    while ((count < maxLength) && K230_ReadByte(&buffer[count])) {
        count++;
    }

    return count;
}

void K230_Process(void)
{
    uint8_t data;

    /*
     * 逐个取出 UART 中断已保存到软件缓冲区的字节。
     * K230_ReadByte() 无数据时返回 false，因此不会阻塞主循环。
     */
    while (K230_ReadByte(&data)) {
        switch (k230ParseState) {
        case K230_WAIT_HEADER_AA:
            /* 只有收到第一个帧头 AA，才开始尝试解析一帧数据。 */
            if (data == 0xAAU) {
                k230ParseState = K230_WAIT_HEADER_55;
            }
            break;

        case K230_WAIT_HEADER_55:
            if (data == 0x55U) {
                /* AA 55 均正确，继续接收本帧的实际数据。 */
                k230ParseState = K230_READ_FOUND;
            } else if (data == 0xAAU) {
                /* 当前字节可作为下一帧的 AA，继续等待 55。 */
                k230ParseState = K230_WAIT_HEADER_55;
            } else {
                /* 帧头不匹配，重新搜索 AA。 */
                k230ParseState = K230_WAIT_HEADER_AA;
            }
            break;

        case K230_READ_FOUND:
            k230FrameFound = data;
            k230ParseState = K230_READ_X_LOW;
            break;

        case K230_READ_X_LOW:
            k230FrameXLow = data;
            k230ParseState = K230_READ_X_HIGH;
            break;

        case K230_READ_X_HIGH:
            k230FrameXHigh = data;
            k230ParseState = K230_READ_Y_LOW;
            break;

        case K230_READ_Y_LOW:
            k230FrameYLow = data;
            k230ParseState = K230_READ_Y_HIGH;
            break;

        case K230_READ_Y_HIGH:
            k230FrameYHigh = data;
            k230ParseState = K230_READ_CHECKSUM;
            break;

        case K230_READ_CHECKSUM:
        {
            uint8_t checksum = (uint8_t) (k230FrameFound +
                                          k230FrameXLow +
                                          k230FrameXHigh +
                                          k230FrameYLow +
                                          k230FrameYHigh);

            if (data == checksum) {
                /* 校验成功：组合高低字节，发布最新一帧目标信息。 */
                k230LatestTarget.found = k230FrameFound;
                k230LatestTarget.x = (uint16_t) k230FrameXLow |
                                     ((uint16_t) k230FrameXHigh << 8U);
                k230LatestTarget.y = (uint16_t) k230FrameYLow |
                                     ((uint16_t) k230FrameYHigh << 8U);
                k230NewTargetAvailable = true;
            } else {
                /*
                 * 校验失败：不把它作为识别结果使用，但保留完整原始帧，
                 * 让 main.c 能发送到串口助手，方便检查 K230 的发送格式。
                 */
                k230LatestInvalidFrame.data[0] = 0xAAU;
                k230LatestInvalidFrame.data[1] = 0x55U;
                k230LatestInvalidFrame.data[2] = k230FrameFound;
                k230LatestInvalidFrame.data[3] = k230FrameXLow;
                k230LatestInvalidFrame.data[4] = k230FrameXHigh;
                k230LatestInvalidFrame.data[5] = k230FrameYLow;
                k230LatestInvalidFrame.data[6] = k230FrameYHigh;
                k230LatestInvalidFrame.data[7] = data;
                k230NewInvalidFrameAvailable = true;
                k230ChecksumErrorCount++;
            }

            k230ParseState = K230_WAIT_HEADER_AA;
            break;
        }

        default:
            /* 防御性处理：状态异常时重新搜索帧头。 */
            k230ParseState = K230_WAIT_HEADER_AA;
            break;
        }
    }
}

bool K230_GetLatestTarget(K230_Target_t *target)
{
    if ((target == NULL) || !k230NewTargetAvailable) {
        return false;
    }

    /* 将最近一帧复制给调用者，并标记为“已读取”。 */
    *target = k230LatestTarget;
    k230NewTargetAvailable = false;
    return true;
}

bool K230_GetLatestInvalidFrame(K230_InvalidFrame_t *frame)
{
    if ((frame == NULL) || !k230NewInvalidFrameAvailable) {
        return false;
    }

    *frame = k230LatestInvalidFrame;
    k230NewInvalidFrameAvailable = false;
    return true;
}

uint32_t K230_GetOverflowCount(void)
{
    /* 只返回统计值，不会自动清零。重新调用 K230_Init() 才会清零。 */
    return k230OverflowCount;
}

uint32_t K230_GetReceivedByteCount(void)
{
    return k230ReceivedByteCount;
}

uint32_t K230_GetChecksumErrorCount(void)
{
    return k230ChecksumErrorCount;
}




/*
 * K230 UART 接收中断服务函数。
 * K230_INST_IRQHandler 是 SysConfig 生成的宏，当前会展开为
 * UART0_IRQHandler，因此能与启动文件中的 UART0 中断向量对应。
 */
void K230_INST_IRQHandler(void)
{
    /* 读取并确认当前 UART0 中断产生的原因。 */
    switch (DL_UART_getPendingInterrupt(K230_INST)) {
    case DL_UART_IIDX_RX:
        /* 一次中断中排空硬件 FIFO，避免连续数据积压。 */
        while (!DL_UART_isRXFIFOEmpty(K230_INST)) {
            /* 从 UART 硬件接收 FIFO 取出一个字节。 */
            uint8_t data = DL_UART_receiveData(K230_INST);
            k230ReceivedByteCount++;

            /* 计算当前写入位置以及写入完成后的下一个位置。 */
            uint16_t head = k230RxHead;
            uint16_t nextHead =
                (uint16_t) ((head + 1U) & K230_RX_BUFFER_MASK);

            /*
             * nextHead 与读指针相等表示软件缓冲区已满。
             * 此处选择丢弃“新字节”，从而保证尚未处理的旧数据不被覆盖。
             */
            if (nextHead == k230RxTail) {
                k230OverflowCount++;
            } else {
                /* 先保存数据，再移动写指针，让主循环看到这个新字节。 */
                k230RxBuffer[head] = data;
                k230RxHead = nextHead;
            }
        }
        break;

    default:
        break;
    }
}

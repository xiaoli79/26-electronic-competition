#include "K230.H"

#define K230_RX_BUFFER_MASK      (K230_RX_BUFFER_SIZE - 1U)
#define K230_PROCESS_BYTE_BUDGET (128U)
#define K230_ASCII_LINE_SIZE      (96U)
#define K230_FORWARD_BUFFER_SIZE (256U)
#define K230_FORWARD_BUFFER_MASK (K230_FORWARD_BUFFER_SIZE - 1U)

#if ((K230_RX_BUFFER_SIZE == 0U) || \
     ((K230_RX_BUFFER_SIZE & K230_RX_BUFFER_MASK) != 0U))
#error "K230_RX_BUFFER_SIZE must be a power of two"
#endif

#if ((K230_FORWARD_BUFFER_SIZE == 0U) || \
     ((K230_FORWARD_BUFFER_SIZE & K230_FORWARD_BUFFER_MASK) != 0U))
#error "K230_FORWARD_BUFFER_SIZE must be a power of two"
#endif

#if K230_ENABLE_TARGET_PARSE
typedef enum {
    K230_WAIT_HEADER_AA = 0,
    K230_WAIT_HEADER_55,
    K230_READ_FOUND,
    K230_READ_X_LOW,
    K230_READ_X_HIGH,
    K230_READ_Y_LOW,
    K230_READ_Y_HIGH,
    K230_READ_CHECKSUM
} K230_BinaryParseState_t;
#endif

static uint8_t k230RxBuffer[K230_RX_BUFFER_SIZE];
static volatile uint16_t k230RxHead = 0U;
static volatile uint16_t k230RxTail = 0U;
static volatile uint32_t k230OverflowCount = 0U;
static volatile uint32_t k230ReceivedByteCount = 0U;

/* 主循环写入、电脑 UART TX 中断读出；与接收队列独立，互不阻塞。 */
static uint8_t k230ForwardBuffer[K230_FORWARD_BUFFER_SIZE];
static volatile uint8_t k230ForwardHead = 0U;
static volatile uint8_t k230ForwardTail = 0U;
static volatile uint32_t k230ForwardOverflowCount = 0U;

#if K230_ENABLE_TARGET_PARSE
static K230_BinaryParseState_t k230BinaryState = K230_WAIT_HEADER_AA;
static uint8_t k230FrameFound = 0U;
static uint8_t k230FrameXLow = 0U;
static uint8_t k230FrameXHigh = 0U;
static uint8_t k230FrameYLow = 0U;
static uint8_t k230FrameYHigh = 0U;

static char k230AsciiLine[K230_ASCII_LINE_SIZE];
static uint8_t k230AsciiLength = 0U;
static K230_Target_t k230LatestTarget = {false, 0};
static bool k230NewTargetAvailable = false;
static uint32_t k230RejectedFrameCount = 0U;
#endif

static bool K230_ReadByte(uint8_t *data)
{
    uint16_t tail;

    if (data == NULL) {
        return false;
    }

    tail = k230RxTail;
    if (tail == k230RxHead) {
        return false;
    }

    *data = k230RxBuffer[tail];
    k230RxTail = (uint16_t) ((tail + 1U) & K230_RX_BUFFER_MASK);
    return true;
}

static void K230_QueueForwardByte(uint8_t data)
{
    uint8_t head = k230ForwardHead;
    uint8_t nextHead = (uint8_t) (head + 1U);

    if (nextHead == k230ForwardTail) {
        /* 电脑未及时读取时只丢弃转发副本，不影响 UART0 后续接收。 */
        k230ForwardOverflowCount++;
        return;
    }

    k230ForwardBuffer[head] = data;
    k230ForwardHead = nextHead;
}

/* 返回电脑转发队列尚可容纳的字节数。该队列只有主循环写入，TX 中断只会
 * 推进 tail，因此这里读取到的值即使瞬间变化，也只会低估空闲空间，不会越界。 */
static uint8_t K230_GetForwardFreeSpace(void)
{
    uint8_t used = (uint8_t) (k230ForwardHead - k230ForwardTail);

    return (uint8_t) (UINT8_MAX - used);
}

#if K230_PC_FORMATTED_OUTPUT_ENABLE
static void K230_QueueText(const char *text)
{
    while (*text != '\0') {
        K230_QueueForwardByte((uint8_t) *text);
        text++;
    }
}

static void K230_QueueUnsignedDecimal(uint16_t value)
{
    char digits[5];
    uint8_t count = 0U;

    do {
        digits[count++] = (char) ('0' + (value % 10U));
        value = (uint16_t) (value / 10U);
    } while (value != 0U);

    while (count != 0U) {
        count--;
        K230_QueueForwardByte((uint8_t) digits[count]);
    }
}

/* 一条最长的文本为 "BALL: POS=-3276.8 cm\r\n"，共 22 字节。 */
static void K230_QueueFormattedTarget(bool found, int16_t position_0p1cm)
{
    int32_t position = position_0p1cm;
    uint16_t wholeCm;
    uint8_t tenthCm;
    uint8_t needed = 22U;

    /* 未识别帧仍会更新内部状态，但不发送到电脑，避免重复刷屏。 */
    if (!found) {
        return;
    }

    /* 必须整行入队，避免电脑端出现半条文字。 */
    if (K230_GetForwardFreeSpace() < needed) {
        k230ForwardOverflowCount++;
        return;
    }

    K230_QueueText("BALL: POS=");
    if (position < 0) {
        K230_QueueForwardByte((uint8_t) '-');
        position = -position;
    }
    wholeCm = (uint16_t) (position / 10);
    tenthCm = (uint8_t) (position % 10);
    K230_QueueUnsignedDecimal(wholeCm);
    K230_QueueForwardByte((uint8_t) '.');
    K230_QueueForwardByte((uint8_t) ('0' + tenthCm));
    K230_QueueText(" cm\r\n");
}
#endif /* K230_PC_FORMATTED_OUTPUT_ENABLE */

#if K230_ENABLE_TARGET_PARSE
static bool K230_IsDigit(char character)
{
    return (character >= '0') && (character <= '9');
}

static char K230_ToUpper(char character)
{
    if ((character >= 'a') && (character <= 'z')) {
        return (char) (character - ('a' - 'A'));
    }
    return character;
}

static bool K230_ContainsIgnoreCase(const char *text, const char *word)
{
    uint8_t index;
    uint8_t wordIndex;

    for (index = 0U; text[index] != '\0'; index++) {
        wordIndex = 0U;
        while ((word[wordIndex] != '\0') &&
               (text[index + wordIndex] != '\0') &&
               (K230_ToUpper(text[index + wordIndex]) ==
                K230_ToUpper(word[wordIndex]))) {
            wordIndex++;
        }
        if (word[wordIndex] == '\0') {
            return true;
        }
    }

    return false;
}

/* 从 x:123、x=123、"x" : 123 等字段读取一个无符号数。 */
static bool K230_FindNamedNumber(const char *text, char name, uint16_t *value)
{
    uint8_t index;

    for (index = 0U; text[index] != '\0'; index++) {
        uint8_t cursor;
        uint32_t number = 0U;
        bool hasDigit = false;

        if (K230_ToUpper(text[index]) != K230_ToUpper(name)) {
            continue;
        }

        /* 字段名必须独立，避免把 box、type 等单词中的字母误当成 x/y。 */
        if ((index != 0U) &&
            ((text[index - 1U] >= 'A' && text[index - 1U] <= 'Z') ||
             (text[index - 1U] >= 'a' && text[index - 1U] <= 'z') ||
             K230_IsDigit(text[index - 1U]))) {
            continue;
        }

        cursor = (uint8_t) (index + 1U);
        while ((text[cursor] == ' ') || (text[cursor] == '\t') ||
               (text[cursor] == '\"') || (text[cursor] == ':') ||
               (text[cursor] == '=') || (text[cursor] == ',')) {
            cursor++;
        }
        while (K230_IsDigit(text[cursor])) {
            hasDigit = true;
            number = (number * 10U) + (uint32_t) (text[cursor] - '0');
            if (number > UINT16_MAX) {
                return false;
            }
            cursor++;
        }
        if (hasDigit) {
            *value = (uint16_t) number;
            return true;
        }
    }

    return false;
}

/* 兼容简单格式 BALL,123,456 或 STEEL_BALL 123 456。 */
static bool K230_FindFirstTwoNumbers(const char *text, uint16_t *x, uint16_t *y)
{
    uint8_t index = 0U;
    uint8_t count = 0U;
    uint16_t values[2];

    while (text[index] != '\0') {
        uint32_t number = 0U;

        if (!K230_IsDigit(text[index])) {
            index++;
            continue;
        }

        do {
            number = (number * 10U) + (uint32_t) (text[index] - '0');
            if (number > UINT16_MAX) {
                return false;
            }
            index++;
        } while (K230_IsDigit(text[index]));

        values[count++] = (uint16_t) number;
        if (count == 2U) {
            *x = values[0];
            *y = values[1];
            return true;
        }
    }

    return false;
}

static void K230_PublishTarget(bool found, int16_t position_0p1cm)
{
    k230LatestTarget.found = found;
    k230LatestTarget.position_0p1cm = position_0p1cm;
    k230NewTargetAvailable = true;
#if K230_PC_FORMATTED_OUTPUT_ENABLE
    K230_QueueFormattedTarget(found, position_0p1cm);
#endif
}

static void K230_ParseAsciiLine(void)
{
    uint16_t x;
    uint16_t y;

    k230AsciiLine[k230AsciiLength] = '\0';

    /* 没有 BALL 标签的信息可能是 FPS、置信度或其他类别，直接忽略。 */
    if (!K230_ContainsIgnoreCase(k230AsciiLine, "BALL")) {
        return;
    }

    if (K230_FindNamedNumber(k230AsciiLine, 'X', &x) &&
        K230_FindNamedNumber(k230AsciiLine, 'Y', &y)) {
        /* 旧 ASCII 格式没有单位信息；保留时仅使用第一个数作为 0.1 cm。 */
        K230_PublishTarget(true, (int16_t) x);
    } else if (K230_FindFirstTwoNumbers(k230AsciiLine, &x, &y)) {
        K230_PublishTarget(true, (int16_t) x);
    } else {
        k230RejectedFrameCount++;
    }
}

static void K230_CollectAscii(uint8_t data)
{
    if ((data == '\r') || (data == '\n')) {
        if (k230AsciiLength != 0U) {
            K230_ParseAsciiLine();
        }
        k230AsciiLength = 0U;
        return;
    }

    if ((data < 0x20U) || (data > 0x7EU)) {
        k230AsciiLength = 0U;
        return;
    }

    if (k230AsciiLength >= (K230_ASCII_LINE_SIZE - 1U)) {
        /* 过长日志行不进入解析，防止异常输出占用内存或 CPU。 */
        k230AsciiLength = 0U;
        k230RejectedFrameCount++;
        return;
    }

    k230AsciiLine[k230AsciiLength++] = (char) data;
}

static void K230_ParseBinary(uint8_t data)
{
    switch (k230BinaryState) {
    case K230_WAIT_HEADER_AA:
        if (data == 0xAAU) {
            k230BinaryState = K230_WAIT_HEADER_55;
        }
        break;

    case K230_WAIT_HEADER_55:
        if (data == 0x55U) {
            k230BinaryState = K230_READ_FOUND;
        } else if (data != 0xAAU) {
            k230BinaryState = K230_WAIT_HEADER_AA;
        }
        break;

    case K230_READ_FOUND:
        k230FrameFound = data;
        k230BinaryState = K230_READ_X_LOW;
        break;

    case K230_READ_X_LOW:
        k230FrameXLow = data;
        k230BinaryState = K230_READ_X_HIGH;
        break;

    case K230_READ_X_HIGH:
        k230FrameXHigh = data;
        k230BinaryState = K230_READ_Y_LOW;
        break;

    case K230_READ_Y_LOW:
        k230FrameYLow = data;
        k230BinaryState = K230_READ_Y_HIGH;
        break;

    case K230_READ_Y_HIGH:
        k230FrameYHigh = data;
        k230BinaryState = K230_READ_CHECKSUM;
        break;

    case K230_READ_CHECKSUM:
    {
        uint8_t checksum = (uint8_t) (k230FrameFound + k230FrameXLow +
                                      k230FrameXHigh + k230FrameYLow +
                                      k230FrameYHigh);

        if ((data == checksum) && ((k230FrameFound == 0U) ||
                                   (k230FrameFound == 1U))) {
            uint16_t rawPosition = (uint16_t) k230FrameXLow |
                ((uint16_t) k230FrameXHigh << 8U);

            /* 字节 3/4 是小端 int16 位置值；字节 5/6 为 K230 保留字节。 */
            K230_PublishTarget(k230FrameFound != 0U,
                               (int16_t) rawPosition);
        } else {
            k230RejectedFrameCount++;
        }
        k230BinaryState = K230_WAIT_HEADER_AA;
        break;
    }

    default:
        k230BinaryState = K230_WAIT_HEADER_AA;
        break;
    }
}
#endif /* K230_ENABLE_TARGET_PARSE */

void K230_Init(void)
{
    NVIC_DisableIRQ(K230_INST_INT_IRQN);

    k230RxHead = 0U;
    k230RxTail = 0U;
    k230OverflowCount = 0U;
    k230ReceivedByteCount = 0U;
    k230ForwardHead = 0U;
    k230ForwardTail = 0U;
    k230ForwardOverflowCount = 0U;
#if K230_ENABLE_TARGET_PARSE
    k230BinaryState = K230_WAIT_HEADER_AA;
    k230AsciiLength = 0U;
    k230LatestTarget.found = false;
    k230LatestTarget.position_0p1cm = 0;
    k230NewTargetAvailable = false;
    k230RejectedFrameCount = 0U;
#endif

    while (!DL_UART_isRXFIFOEmpty(K230_INST)) {
        (void) DL_UART_receiveData(K230_INST);
    }
    NVIC_ClearPendingIRQ(K230_INST_INT_IRQN);
    NVIC_EnableIRQ(K230_INST_INT_IRQN);
}

void K230_Process(void)
{
    uint8_t data;
    uint8_t processed = 0U;

    while ((processed < K230_PROCESS_BYTE_BUDGET) && K230_ReadByte(&data)) {
        /* 抓取协议时可保留原始字节；正常运行默认只向电脑输出易读文本。 */
#if K230_PC_RAW_FORWARD_ENABLE
        K230_QueueForwardByte(data);
#endif
#if K230_ENABLE_TARGET_PARSE
        K230_ParseBinary(data);
        K230_CollectAscii(data);
#endif
        processed++;
    }
}

bool K230_ReadForwardByte(uint8_t *data)
{
    uint8_t tail;

    if (data == NULL) {
        return false;
    }

    tail = k230ForwardTail;
    if (tail == k230ForwardHead) {
        return false;
    }

    *data = k230ForwardBuffer[tail];
    k230ForwardTail = (uint8_t) (tail + 1U);
    return true;
}

bool K230_GetLatestTarget(K230_Target_t *target)
{
#if K230_ENABLE_TARGET_PARSE
    if ((target == NULL) || !k230NewTargetAvailable) {
        return false;
    }

    *target = k230LatestTarget;
    k230NewTargetAvailable = false;
    return true;
#else
    (void) target;
    return false;
#endif
}

uint32_t K230_GetOverflowCount(void)
{
    return k230OverflowCount;
}

uint32_t K230_GetForwardOverflowCount(void)
{
    return k230ForwardOverflowCount;
}

uint32_t K230_GetReceivedByteCount(void)
{
    return k230ReceivedByteCount;
}

uint32_t K230_GetRejectedFrameCount(void)
{
#if K230_ENABLE_TARGET_PARSE
    return k230RejectedFrameCount;
#else
    return 0U;
#endif
}

void K230_INST_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(K230_INST) == DL_UART_IIDX_RX) {
        while (!DL_UART_isRXFIFOEmpty(K230_INST)) {
            uint8_t data = DL_UART_receiveData(K230_INST);
            uint16_t head = k230RxHead;
            uint16_t nextHead =
                (uint16_t) ((head + 1U) & K230_RX_BUFFER_MASK);

            k230ReceivedByteCount++;
            if (nextHead == k230RxTail) {
                k230OverflowCount++;
            } else {
                k230RxBuffer[head] = data;
                k230RxHead = nextHead;
            }
        }
    }
}

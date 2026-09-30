#include "Com.h"
#include "Board_485.h"

static UartTransport_t s_transport;
static ProtoParser_t   s_parser;


static void Com_OnFrameReceived(const uint8_t *data, uint16_t len);

/**
 * @brief 初始化UART
 * 
 * @param huart UART句柄
 */
void Com_Uart_Init(UART_HandleTypeDef *huart) {
    UartTransport_Init(&s_transport, huart);
    Proto_Init(&s_parser);
}

/**
 * @brief 主循环调用，处理发送数据
 * 
 * 解析UART发送缓冲区中的数据，调用用户实现的回调函数处理有效帧。
 */
void Com_TX_MainFunction(void)
{
    if (UartTransport_Available(&s_transport) > 0U)
    {
        UartTransport_StartTx(&s_transport);
    }
}
/**
 * @brief 主循环调用，处理接收数据
 * 
 * 解析UART接收缓冲区中的数据，调用用户实现的回调函数处理有效帧。
 */
void Com_RX_MainFunction(void) {
    uint8_t byte;
    /* 接收方向：只解析自定义帧协议 */
    while (UartTransport_ReadByte(&s_transport, &byte)) {
        if (Proto_ParseByte(&s_parser, byte)) {
            Com_OnFrameReceived(s_parser.frame.data,
                                s_parser.frame.length);
        }
    }
}

/**
 * @brief 发送自定义帧协议
 * 
 * @param data 帧数据指针
 * @param len 帧数据长度
 */
void Com_Uart_SendFrame(const uint8_t *data, uint16_t len) {
    uint8_t buf[PROTO_FRAME_MAX_SIZE];
    uint16_t total = Proto_BuildFrame(data, len, buf, sizeof(buf));
    if (total > 0) {
        UartTransport_SendBytes(&s_transport, buf, total);
    }
}

/* ============ 调试通道发送：JustFloat ============ */
#if APP_ENABLE_JUSTFLOAT
void Com_Uart_SendJustFloat(const float *ch_data, uint8_t ch_count) {
    uint8_t buf[JUSTFLOAT_MAX_CHANNELS * 4 + JUSTFLOAT_TAIL_SIZE];
    uint16_t total = JustFloat_Build(ch_data, ch_count, buf, sizeof(buf));
    if (total > 0) {
        UartTransport_SendBytes(&s_transport, buf, total);
    }
}
#endif

/**
 * @brief 处理接收的自定义帧协议
 * 
 * @param data 帧数据指针
 * @param len 帧数据长度
 */
static void Com_OnFrameReceived(const uint8_t *data, uint16_t len) {
    if (len < 1) return;

    uint8_t cmd = data[0];   /* 命令码 */
    switch (cmd) {
    case 0x01:   /* 设置目标电流 */
        break;

    case 0x02:   /* 设置目标位置 */

        break;

    case 0x03:   /* 查询状态 */

        break;

    default:
        break;
    }   
}

/**
 * @brief UART接收事件回调
 *
 * 由HAL在DMA半传输、全传输或UART空闲事件时调用。
 */
void HAL_UARTEx_RxEventCallback(
    UART_HandleTypeDef *huart,
    uint16_t size)
{
    if ((huart != (UART_HandleTypeDef *)0) &&
        (huart->Instance == s_transport.huart->Instance))
    {
        UartTransport_RxEventHandler(&s_transport, size);
    }
    else if ((huart != (UART_HandleTypeDef *)0) &&
             (huart->Instance == USART3))
    {
        Rs485_RxEventHandler(size);
    }
}



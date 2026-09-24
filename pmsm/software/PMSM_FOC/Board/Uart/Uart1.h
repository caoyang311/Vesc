/**
 * @file Uart1.h
 * @brief USART1 独立板级驱动接口。
 */
#ifndef BOARD_UART1_H
#define BOARD_UART1_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "usart.h"

/**
 * @brief UART1 模块操作结果。
 */
#define UART1_RX_MESSAGE_MAX_LENGTH (64U) /** @brief 接收消息最大长度，单位为字节 */

typedef enum
{
    UART1_RESULT_OK = 0U,
    UART1_RESULT_NOT_INITIALIZED,
    UART1_RESULT_INVALID_PARAMETER,
    UART1_RESULT_BUSY,
    UART1_RESULT_NO_DATA,
    UART1_RESULT_HAL_ERROR
} Uart1_ResultType;

typedef struct
{
    uint8_t Length; /** @brief 数据长度，单位为字节 */
    uint8_t Data[UART1_RX_MESSAGE_MAX_LENGTH]; /** @brief 数据缓冲区 */
} Uart1_MessageType;

/**
 * @brief 初始化 UART1 驱动并启动 DMA + UART 空闲接收。
 */
void Uart1_Init(void);

/**
 * @brief 发送一段数据。
 *
 * @param[in] Data 数据缓冲区。
 * @param[in] Length 数据长度，单位为字节。
 * @return UART1_RESULT_OK 表示发送成功。
 */
Uart1_ResultType Uart1_Send(const uint8_t *Data, uint16_t Length);

/**
 * @brief 从消息队列取出一帧 DMA 接收数据。
 * @param[out] Message 消息输出缓冲区。
 * @return UART1_RESULT_OK 表示读取成功；UART1_RESULT_NO_DATA 表示当前无消息。
 */
Uart1_ResultType Uart1_ReceiveMessage(Uart1_MessageType *Message);

/**
 * @brief 获取 UART1 接收消息队列中的消息数量。
 * @return 当前待处理消息数量。
 */
uint16_t Uart1_GetReceivedMessageCount(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_UART1_H */

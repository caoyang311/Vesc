#ifndef BOARD_MAX_485_H
#define BOARD_MAX_485_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "usart.h"

typedef enum
{
    MAX_485_RESULT_OK = 0U,
    MAX_485_RESULT_NOT_INITIALIZED,
    MAX_485_RESULT_INVALID_PARAMETER,
    MAX_485_RESULT_BUSY,
    MAX_485_RESULT_NO_DATA,
    MAX_485_RESULT_HAL_ERROR
} Max_485_ResultType;

/**
 * @brief 初始化 MAX485，并启动 USART3 单字节接收中断。
 */
void Max_485_Init(void);

/**
 * @brief 通过 USART3 发送数据。
 *
 * @param[in] Data 数据缓冲区。
 * @param[in] Length 数据长度，单位为字节。
 * @return Max_485_ResultType 操作结果。
 */
Max_485_ResultType Max_485_Send(const uint8_t *Data, uint16_t Length);

/**
 * @brief 从 MAX485 接收缓冲区读取一个字节。
 *
 * @param[out] Data 接收数据输出地址。
 * @return Max_485_ResultType 操作结果。
 */
Max_485_ResultType Max_485_ReceiveByte(uint8_t *Data);

/**
 * @brief 获取 MAX485 接收缓冲区中的数据量。
 */
uint16_t Max_485_GetReceivedCount(void);

/**
 * @brief 处理 USART3 接收完成事件。
 *
 * @param[in] UartHandle UART 句柄。
 */
void Max_485_HandleRxComplete(UART_HandleTypeDef *UartHandle);

/**
 * @brief HAL UART 接收完成回调分发函数。
 *
 * @param[in] UartHandle UART 句柄。
 */
void Max_485_HandleUartRxComplete(UART_HandleTypeDef *UartHandle);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_MAX_485_H */

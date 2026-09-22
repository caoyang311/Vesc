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

/**
 * @brief UART1 模块操作结果。
 */
typedef enum
{
    UART1_RESULT_OK = 0U,
    UART1_RESULT_NOT_INITIALIZED,
    UART1_RESULT_INVALID_PARAMETER,
    UART1_RESULT_BUSY,
    UART1_RESULT_NO_DATA,
    UART1_RESULT_HAL_ERROR
} Uart1_ResultType;

/**
 * @brief 初始化 UART1 驱动并使能单字节接收中断。
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
 * @brief 从 UART1 接收环形缓冲区读取一个字节。
 *
 * @param[out] Data 用于保存接收字节的指针。
 * @return UART1_RESULT_OK 表示读取成功；UART1_RESULT_NO_DATA 表示当前无数据。
 */
Uart1_ResultType Uart1_ReceiveByte(uint8_t *Data);

/**
 * @brief 获取 UART1 接收缓冲区中的字节数。
 *
 * @return 当前待读取的字节数。
 */
uint16_t Uart1_GetReceivedCount(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_UART1_H */

/**
 * @file Uart1.c
 * @brief USART1 独立板级驱动实现。
 */
#include "Uart1.h"

#include "usart.h"

#define UART1_RX_BUFFER_SIZE (128U)

#if (UART1_RX_BUFFER_SIZE > UINT16_MAX)
#error "UART1_RX_BUFFER_SIZE exceeds uint16_t range"
#endif

static uint8_t Uart1_RxByte = 0U;
static uint8_t Uart1_RxBuffer[UART1_RX_BUFFER_SIZE] = {0U};
static volatile uint16_t Uart1_RxWriteIndex = 0U;
static volatile uint16_t Uart1_RxReadIndex = 0U;
static volatile uint16_t Uart1_RxCount = 0U;
static volatile uint8_t Uart1_Initialized = 0U;

static void Uart1_ReceiveInterruptStart(void);

/**
 * @brief 初始化 UART1 驱动并启动单字节接收中断。
 */
void Uart1_Init(void)
{
    uint16_t index = 0U;

    for (index = 0U; index < UART1_RX_BUFFER_SIZE; index++)
    {
        Uart1_RxBuffer[index] = 0U;
    }

    Uart1_RxByte = 0U;
    Uart1_RxWriteIndex = 0U;
    Uart1_RxReadIndex = 0U;
    Uart1_RxCount = 0U;
    Uart1_Initialized = 1U;
    Uart1_ReceiveInterruptStart();
}

/**
 * @brief 发送一段数据。
 */
Uart1_ResultType Uart1_Send(const uint8_t *Data, uint16_t Length)
{
    HAL_StatusTypeDef hal_result = HAL_ERROR;
    Uart1_ResultType result = UART1_RESULT_OK;

    if (Uart1_Initialized == 0U)
    {
        result = UART1_RESULT_NOT_INITIALIZED;
    }
    else if ((Data == (const uint8_t *)0) || (Length == 0U))
    {
        result = UART1_RESULT_INVALID_PARAMETER;
    }
    else
    {
        hal_result = HAL_UART_Transmit(&huart1, (uint8_t *)Data, Length, HAL_MAX_DELAY);
        if (hal_result != HAL_OK)
        {
            result = UART1_RESULT_HAL_ERROR;
        }
        else
        {
            result = UART1_RESULT_OK;
        }
    }

    return result;
}

/**
 * @brief 从 UART1 接收环形缓冲区读取一个字节。
 */
Uart1_ResultType Uart1_ReceiveByte(uint8_t *Data)
{
    Uart1_ResultType result = UART1_RESULT_OK;

    if (Uart1_Initialized == 0U)
    {
        result = UART1_RESULT_NOT_INITIALIZED;
    }
    else if (Data == (uint8_t *)0)
    {
        result = UART1_RESULT_INVALID_PARAMETER;
    }
    else if (Uart1_RxCount == 0U)
    {
        result = UART1_RESULT_NO_DATA;
    }
    else
    {
        *Data = Uart1_RxBuffer[Uart1_RxReadIndex];
        Uart1_RxReadIndex++;
        if (Uart1_RxReadIndex >= UART1_RX_BUFFER_SIZE)
        {
            Uart1_RxReadIndex = 0U;
        }
        Uart1_RxCount--;
        result = UART1_RESULT_OK;
    }

    return result;
}

/**
 * @brief 获取 UART1 接收缓冲区中的字节数。
 */
uint16_t Uart1_GetReceivedCount(void)
{
    uint16_t count = 0U;

    count = Uart1_RxCount;

    return count;
}

/**
 * @brief 启动 UART1 单字节接收中断。
 */
static void Uart1_ReceiveInterruptStart(void)
{
    (void)HAL_UART_Receive_IT(&huart1, &Uart1_RxByte, 1U);
}

/**
 * @brief HAL UART 接收完成回调。
 *
 * @param[in] UartHandle UART 外设句柄。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    uint16_t next_index = 0U;

    if ((UartHandle != (UART_HandleTypeDef *)0) && (UartHandle->Instance == USART1))
    {
        next_index = Uart1_RxWriteIndex + 1U;
        if (next_index >= UART1_RX_BUFFER_SIZE)
        {
            next_index = 0U;
        }

        if (Uart1_RxCount < UART1_RX_BUFFER_SIZE)
        {
            Uart1_RxBuffer[Uart1_RxWriteIndex] = Uart1_RxByte;
            Uart1_RxWriteIndex = next_index;
            Uart1_RxCount++;
        }
        else
        {
            /* 缓冲区满时丢弃当前字节，保留已有数据。 */
        }

        Uart1_ReceiveInterruptStart();
    }
    else
    {
        /* 非 USART1 接收事件不在本模块处理。 */
    }
}

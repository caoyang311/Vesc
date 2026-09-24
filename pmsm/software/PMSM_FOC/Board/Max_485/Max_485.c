#include "Max_485.h"

#include "Dio.h"

#define MAX_485_RX_BUFFER_SIZE (256U)

static uint8_t Max_485_RxByte = 0U;
static uint8_t Max_485_RxBuffer[MAX_485_RX_BUFFER_SIZE] = {0U};
static volatile uint16_t Max_485_RxWriteIndex = 0U;
static volatile uint16_t Max_485_RxReadIndex = 0U;
static volatile uint16_t Max_485_RxCount = 0U;
static volatile uint8_t Max_485_Initialized = 0U;

static void Max_485_StartReceiveInterrupt(void)
{
    (void)HAL_UART_Receive_IT(&huart3, &Max_485_RxByte, 1U);
}

void Max_485_Init(void)
{
    uint16_t index = 0U;

    for (index = 0U; index < MAX_485_RX_BUFFER_SIZE; index++)
    {
        Max_485_RxBuffer[index] = 0U;
    }

    Max_485_RxByte = 0U;
    Max_485_RxWriteIndex = 0U;
    Max_485_RxReadIndex = 0U;
    Max_485_RxCount = 0U;
    Max_485_Initialized = 1U;

    Dio_WriteChannel(DIO_CHANNEL_MAX_485_EN, STD_LOW);
    Max_485_StartReceiveInterrupt();
}

Max_485_ResultType Max_485_Send(const uint8_t *Data, uint16_t Length)
{
    HAL_StatusTypeDef halResult = HAL_ERROR;
    Max_485_ResultType result = MAX_485_RESULT_HAL_ERROR;

    if (Max_485_Initialized == 0U)
    {
        result = MAX_485_RESULT_NOT_INITIALIZED;
    }
    else if ((Data == (const uint8_t *)0) || (Length == 0U))
    {
        result = MAX_485_RESULT_INVALID_PARAMETER;
    }
    else
    {
        Dio_WriteChannel(DIO_CHANNEL_MAX_485_EN, STD_HIGH);
        halResult = HAL_UART_Transmit(&huart3, (uint8_t *)Data, Length, HAL_MAX_DELAY);
        Dio_WriteChannel(DIO_CHANNEL_MAX_485_EN, STD_LOW);

        if (halResult == HAL_OK)
        {
            result = MAX_485_RESULT_OK;
        }
        else if (halResult == HAL_BUSY)
        {
            result = MAX_485_RESULT_BUSY;
        }
        else
        {
            result = MAX_485_RESULT_HAL_ERROR;
        }
    }

    return result;
}

Max_485_ResultType Max_485_ReceiveByte(uint8_t *Data)
{
    Max_485_ResultType result = MAX_485_RESULT_OK;

    if (Max_485_Initialized == 0U)
    {
        result = MAX_485_RESULT_NOT_INITIALIZED;
    }
    else if (Data == (uint8_t *)0)
    {
        result = MAX_485_RESULT_INVALID_PARAMETER;
    }
    else if (Max_485_RxCount == 0U)
    {
        result = MAX_485_RESULT_NO_DATA;
    }
    else
    {
        *Data = Max_485_RxBuffer[Max_485_RxReadIndex];
        Max_485_RxReadIndex++;
        if (Max_485_RxReadIndex >= MAX_485_RX_BUFFER_SIZE)
        {
            Max_485_RxReadIndex = 0U;
        }
        Max_485_RxCount--;
    }

    return result;
}

uint16_t Max_485_GetReceivedCount(void)
{
    return Max_485_RxCount;
}

void Max_485_HandleRxComplete(UART_HandleTypeDef *UartHandle)
{
    uint16_t nextIndex = 0U;

    if ((UartHandle != (UART_HandleTypeDef *)0) && (UartHandle->Instance == USART3))
    {
        nextIndex = Max_485_RxWriteIndex + 1U;
        if (nextIndex >= MAX_485_RX_BUFFER_SIZE)
        {
            nextIndex = 0U;
        }

        if (Max_485_RxCount < MAX_485_RX_BUFFER_SIZE)
        {
            Max_485_RxBuffer[Max_485_RxWriteIndex] = Max_485_RxByte;
            Max_485_RxWriteIndex = nextIndex;
            Max_485_RxCount++;
        }
        else
        {
            /* 缓冲区满时丢弃当前字节。 */
        }

        Max_485_StartReceiveInterrupt();
    }
    else
    {
        /* 非 USART3 事件不在本模块处理。 */
    }
}

void Max_485_HandleUartRxComplete(UART_HandleTypeDef *UartHandle)
{
    Max_485_HandleRxComplete(UartHandle);
}

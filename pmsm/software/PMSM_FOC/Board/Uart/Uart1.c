/**
 * @file Uart1.c
 * @brief USART1 DMA + IDLE 接收驱动实现。
 */
#include "Uart1.h"

#include "usart.h"
#include "Max_485.h"

#define UART1_DMA_RX_BUFFER_SIZE       (256U)
#define UART1_RX_MESSAGE_QUEUE_SIZE    (8U)

#if (UART1_DMA_RX_BUFFER_SIZE > UINT16_MAX)
#error "UART1_DMA_RX_BUFFER_SIZE exceeds uint16_t range"
#endif

#if (UART1_RX_MESSAGE_QUEUE_SIZE > UINT16_MAX)
#error "UART1_RX_MESSAGE_QUEUE_SIZE exceeds uint16_t range"
#endif



static uint8_t Uart1_DmaRxBuffer[UART1_DMA_RX_BUFFER_SIZE] = {0U};
static Uart1_MessageType Uart1_MessageQueue[UART1_RX_MESSAGE_QUEUE_SIZE] = {0};
static volatile uint16_t Uart1_QueueWriteIndex = 0U;
static volatile uint16_t Uart1_QueueReadIndex = 0U;
static volatile uint16_t Uart1_QueueCount = 0U;
static volatile uint8_t Uart1_Initialized = 0U;

static Uart1_ResultType Uart1_StartDmaReceive(void);
static void Uart1_ResetDmaBuffer(void);
static void Uart1_EnqueueMessage(const uint8_t *Data, uint16_t Length);

void Uart1_Init(void)
{
    uint16_t index = 0U;

    for (index = 0U; index < UART1_DMA_RX_BUFFER_SIZE; index++)
    {
        Uart1_DmaRxBuffer[index] = 0U;
    }

    for (index = 0U; index < UART1_RX_MESSAGE_QUEUE_SIZE; index++)
    {
        Uart1_MessageQueue[index].Length = 0U;
    }

    Uart1_QueueWriteIndex = 0U;
    Uart1_QueueReadIndex = 0U;
    Uart1_QueueCount = 0U;
    Uart1_Initialized = 1U;
    (void)Uart1_StartDmaReceive();
}

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

Uart1_ResultType Uart1_ReceiveMessage(Uart1_MessageType *Message)
{
    Uart1_ResultType result = UART1_RESULT_OK;
    uint16_t index = 0U;

    if (Uart1_Initialized == 0U)
    {
        result = UART1_RESULT_NOT_INITIALIZED;
    }
    else if (Message == (Uart1_MessageType *)0)
    {
        result = UART1_RESULT_INVALID_PARAMETER;
    }
    else if (Uart1_QueueCount == 0U)
    {
        result = UART1_RESULT_NO_DATA;
    }
    else
    {
        Message->Length = Uart1_MessageQueue[Uart1_QueueReadIndex].Length;
        for (index = 0U; index < Message->Length; index++)
        {
            Message->Data[index] = Uart1_MessageQueue[Uart1_QueueReadIndex].Data[index];
        }
        Uart1_MessageQueue[Uart1_QueueReadIndex].Length = 0U;
        Uart1_QueueReadIndex++;
        if (Uart1_QueueReadIndex >= UART1_RX_MESSAGE_QUEUE_SIZE)
        {
            Uart1_QueueReadIndex = 0U;
        }
        Uart1_QueueCount--;
    }

    return result;
}

uint16_t Uart1_GetReceivedMessageCount(void)
{
    return Uart1_QueueCount;
}

static Uart1_ResultType Uart1_StartDmaReceive(void)
{
    Uart1_ResultType result = UART1_RESULT_OK;
    HAL_StatusTypeDef hal_result = HAL_ERROR;

    hal_result = HAL_UARTEx_ReceiveToIdle_DMA(
        &huart1,
        Uart1_DmaRxBuffer,
        UART1_DMA_RX_BUFFER_SIZE);

    if (hal_result != HAL_OK)
    {
        result = UART1_RESULT_HAL_ERROR;
    }
    else
    {
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }

    return result;
}

static void Uart1_ResetDmaBuffer(void)
{
    uint16_t index = 0U;

    (void)HAL_UART_AbortReceive(&huart1);
    for (index = 0U; index < UART1_DMA_RX_BUFFER_SIZE; index++)
    {
        Uart1_DmaRxBuffer[index] = 0U;
    }
    (void)Uart1_StartDmaReceive();
}

static void Uart1_EnqueueMessage(const uint8_t *Data, uint16_t Length)
{
    uint16_t index = 0U;

    if ((Data != (const uint8_t *)0) && (Length > 0U) &&
        (Length <= UART1_DMA_RX_BUFFER_SIZE))
    {
        if (Uart1_QueueCount < UART1_RX_MESSAGE_QUEUE_SIZE)
        {
            Uart1_MessageQueue[Uart1_QueueWriteIndex].Length = Length;
            for (index = 0U; index < Length; index++)
            {
                Uart1_MessageQueue[Uart1_QueueWriteIndex].Data[index] = Data[index];
            }
            Uart1_QueueWriteIndex++;
            if (Uart1_QueueWriteIndex >= UART1_RX_MESSAGE_QUEUE_SIZE)
            {
                Uart1_QueueWriteIndex = 0U;
            }
            Uart1_QueueCount++;
        }
        else
        {
            /* 队列满时丢弃当前消息，保留已有消息。 */
        }
    }
    else
    {
        /* 忽略空消息或超长消息。 */
    }
}

void Uart1_HandleRxEvent(UART_HandleTypeDef *UartHandle, uint16_t Length)
{
    if ((Uart1_Initialized != 0U) &&
        (UartHandle != (UART_HandleTypeDef *)0) &&
        (UartHandle->Instance == USART1))
    {
        Uart1_EnqueueMessage(Uart1_DmaRxBuffer, Length);
        Uart1_ResetDmaBuffer();
    }
    else
    {
        /* 非 USART1 事件不在本模块处理。 */
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *UartHandle, uint16_t Size)
{
    Uart1_HandleRxEvent(UartHandle, Size);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    Max_485_HandleUartRxComplete(UartHandle);
}

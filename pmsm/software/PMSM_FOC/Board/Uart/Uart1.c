/**
 * @file Uart1.c
 * @brief USART1 DMA + IDLE 接收驱动实现。
 */
#include "Uart1.h"
#include <string.h>
#include "usart.h"

#define UART1_RX_MESSAGE_QUEUE_SIZE    (8U)



static uint8_t Uart1_DmaRxBuffer[UART1_RX_MESSAGE_MAX_LENGTH] = {0U}; /** @brief DMA 接收缓冲区 */
static Uart1_MessageType Uart1_MessageQueue[UART1_RX_MESSAGE_QUEUE_SIZE] = {0}; /** @brief 消息队列 */
static volatile uint16_t Uart1_QueueWriteIndex = 0U; /** @brief 写入队列索引 */
static volatile uint16_t Uart1_QueueReadIndex = 0U; /** @brief 读取队列索引 */
static volatile uint16_t Uart1_QueueCount = 0U; /** @brief 队列消息数量 */
static volatile uint8_t Uart1_Initialized = 0U; /** @brief 初始化标志位 */

static Uart1_ResultType Uart1_StartDmaReceive(void);
static void Uart1_ResetDmaBuffer(void);
static void Uart1_EnqueueMessage(const uint8_t *Data, uint16_t Length);
/**
 * @brief 初始化 UART1 驱动并启动 DMA + UART 空闲接收。
 */
void Uart1_Init(void)
{
    uint16_t index = 0U;

    for (index = 0U; index < UART1_RX_MESSAGE_MAX_LENGTH; index++)
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
/**
 * @brief 发送一段数据。
 *
 * @param[in] Data 数据缓冲区。
 * @param[in] Length 数据长度，单位为字节。
 * @return UART1_RESULT_OK 表示发送成功。
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
 * @brief 从消息队列取出一帧 DMA 接收数据。
 * @param[out] Message 消息输出缓冲区。
 * @return UART1_RESULT_OK 表示读取成功；UART1_RESULT_NO_DATA 表示当前无消息。
 */
Uart1_ResultType Uart1_ReceiveMessage(Uart1_MessageType *Message)
{
    Uart1_ResultType result = UART1_RESULT_OK;

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
        (void)memcpy(Message->Data,Uart1_MessageQueue[Uart1_QueueReadIndex].Data,Message->Length);
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
/**
 * @brief 获取当前接收消息队列中的消息数量。
 * @return 消息数量。
 */
uint16_t Uart1_GetReceivedMessageCount(void)
{
    return Uart1_QueueCount;
}
/**
 * @brief 启动 UART1 DMA 空闲接收。
 */
static Uart1_ResultType Uart1_StartDmaReceive(void)
{
    Uart1_ResultType result = UART1_RESULT_OK;
    HAL_StatusTypeDef hal_result = HAL_ERROR;

    hal_result = HAL_UARTEx_ReceiveToIdle_DMA( &huart1,Uart1_DmaRxBuffer,UART1_RX_MESSAGE_MAX_LENGTH);

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
/**
 * @brief 重置 UART1 DMA 接收缓冲区。
 */
static void Uart1_ResetDmaBuffer(void)
{
    uint16_t index = 0U;

    (void)HAL_UART_AbortReceive(&huart1);
    for (index = 0U; index < UART1_RX_MESSAGE_MAX_LENGTH; index++)
    {
        Uart1_DmaRxBuffer[index] = 0U;
    }
    (void)Uart1_StartDmaReceive();
}
/**
 * @brief 将一帧 DMA 接收数据入队。
 * @param[in] Data 数据缓冲区。
 * @param[in] Length 数据长度，单位为字节。
 */
static void Uart1_EnqueueMessage(const uint8_t *Data, uint16_t Length)
{
    if ((Data != (const uint8_t *)0) && (Length > 0U) &&
        (Length <= UART1_RX_MESSAGE_MAX_LENGTH))
    {
        if (Uart1_QueueCount < UART1_RX_MESSAGE_QUEUE_SIZE)
        {
            Uart1_MessageQueue[Uart1_QueueWriteIndex].Length = Length;
            (void)memcpy(Uart1_MessageQueue[Uart1_QueueWriteIndex].Data,Data,Length);
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
/**
 * @brief 处理 UART1 接收事件。
 * @param[in] UartHandle UART 处理句柄。
 * @param[in] Length 接收数据长度，单位为字节。
 */
static void Uart1_HandleRxEvent(UART_HandleTypeDef *UartHandle, uint16_t Length)
{
    if ((Uart1_Initialized != 0U) &&
        (UartHandle != (UART_HandleTypeDef *)0) &&
        (UartHandle->Instance == USART1))
    {
        Uart1_EnqueueMessage(Uart1_DmaRxBuffer, Length); /* 数据入队 */
        Uart1_ResetDmaBuffer(); /* 重置 DMA 接收缓冲区 */
    }
    else
    {
        /* 非 USART1 事件不在本模块处理。 */
    }
}
/**
 * @brief 处理 UART1 接收事件回调函数。
 * @param[in] UartHandle UART 处理句柄。
 * @param[in] Size 接收数据长度，单位为字节。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *UartHandle, uint16_t Size)
{
    Uart1_HandleRxEvent(UartHandle, Size); /* 处理接收事件 */
}

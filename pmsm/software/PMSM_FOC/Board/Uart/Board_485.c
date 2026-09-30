#include "Board_485.h"
#include <string.h>

static UART_HandleTypeDef *rs485Huart = &RS485_UART_HANDLE;
static uint8_t s_rxDmaBuf[RS485_RX_BUF_SIZE];
static uint8_t s_txBuf[RS485_TX_BUFFER_SIZE];
static volatile uint16_t s_rxLen;
static volatile uint8_t s_rxReady;
static volatile uint8_t s_txBusy;
/**
 * @brief 设置为发送模式
 */
static void Rs485_SetTxMode(void)
{
    HAL_GPIO_WritePin(
        RS485_DE_PORT,
        RS485_DE_PIN,
        GPIO_PIN_SET);
}
/**
 * @brief 设置为接收模式
 */
static void Rs485_SetRxMode(void)
{
    HAL_GPIO_WritePin(
        RS485_DE_PORT,
        RS485_DE_PIN,
        GPIO_PIN_RESET);
}
/**
 * @brief 启动接收 DMA
 */
static void Rs485_StartRxDma(void)
{
    Rs485_SetRxMode();

    (void)HAL_UARTEx_ReceiveToIdle_DMA(
        rs485Huart,
        s_rxDmaBuf,
        RS485_RX_BUF_SIZE);

    if (rs485Huart->hdmarx != (DMA_HandleTypeDef *)0)
    {
        __HAL_DMA_DISABLE_IT(rs485Huart->hdmarx, DMA_IT_HT);
    }
}
/**
 * @brief 初始化 RS485 模块
 */
void Rs485_Init(void)
{
    s_rxLen = 0U;
    s_rxReady = 0U;
    s_txBusy = 0U;
    Rs485_StartRxDma();
}
/**
 * @brief 发送一帧数据
 * @param data 数据指针
 * @param len 数据长度
 * @return rs485_status_t 状态码
 */
rs485_status_t Rs485_SendFrame(const uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef result;

    if ((data == (const uint8_t *)0) ||
        (len == 0U) ||
        (len > RS485_TX_BUFFER_SIZE) ||
        (s_txBusy != 0U))
    {
        return RS485_ERR_PARAM;
    }

    (void)memcpy(s_txBuf, data, len);
    s_txBusy = 1U;
    Rs485_SetTxMode();
    (void)HAL_UART_DMAStop(rs485Huart);
    result = HAL_UART_Transmit_IT(rs485Huart, s_txBuf, len);

    if (result != HAL_OK)
    {
        s_txBusy = 0U;
        Rs485_StartRxDma();
        return RS485_ERR_TIMEOUT;
    }

    return RS485_OK;
}
/**
 * @brief 检查是否有新帧可用
 * @return uint8_t 是否有新帧
 */
uint8_t Rs485_FrameAvailable(void)
{
    return s_rxReady;
}
/**
 * @brief 获取当前接收缓冲区中的帧
 * @param out_buf 接收缓冲区指针
 * @param out_len 接收缓冲区长度指针
 * @return rs485_status_t 状态码
 */
rs485_status_t Rs485_GetFrame(uint8_t *out_buf, uint16_t *out_len)
{
    uint16_t len;

    if ((out_buf == (uint8_t *)0) || (out_len == (uint16_t *)0))
    {
        return RS485_ERR_PARAM;
    }

    if (s_rxReady == 0U)
    {
        return RS485_ERR_NO_FRAME;
    }

    len = s_rxLen;
    (void)memcpy(out_buf, s_rxDmaBuf, len);
    *out_len = len;
    s_rxReady = 0U;

    return RS485_OK;
}
/**
 * @brief 丢弃当前接收缓冲区中的帧（复位接收状态）
 */
void Rs485_FlushRx(void)
{
    s_rxLen = 0U;
    s_rxReady = 0U;
    (void)memset(s_rxDmaBuf, 0, sizeof(s_rxDmaBuf));
}
/**
 * @brief 处理 UART DMA 接收事件
 * @param size DMA 当前接收位置
 */
void Rs485_RxEventHandler(uint16_t size)
{
    if ((size > 0U) && (size <= RS485_RX_BUF_SIZE))
    {
        s_rxLen = size;
        s_rxReady = 1U;
    }
}
/**
 * @brief 处理 UART DMA 发送完成事件
 * @param huart UART 处理柄
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart != (UART_HandleTypeDef *)0) &&
        (huart->Instance == rs485Huart->Instance))
    {
        s_txBusy = 0U;
        Rs485_StartRxDma();/* 重新启动接收 DMA */
    }
}

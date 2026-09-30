#include "Board_Uart1.h"
#include <string.h>

/**
 * @brief 入队 TX FIFO
 * @param f TX FIFO 结构体指针
 * @param d 要入队的字节
 * @return true 成功入队，false 队满
 */
static bool TxFifo_Enqueue(UartTxFifo_t *f, uint8_t d) {
    uint16_t next = (f->head + 1) & (UART_TX_FIFO_SIZE - 1);
    if (next == f->tail) return false;
    f->buffer[f->head] = d;
    f->head = next;
    return true;
}

/**
 * @brief 出队 TX FIFO
 * @param f TX FIFO 结构体指针
 * @param d 要出队的字节指针
 * @return true 成功出队，false 队空
 */
static bool TxFifo_Dequeue(UartTxFifo_t *f, uint8_t *d) {
    if (f->head == f->tail) return false;
    *d = f->buffer[f->tail];
    f->tail = (f->tail + 1) & (UART_TX_FIFO_SIZE - 1);
    return true;
}

/**
 * @brief 入队 RX FIFO
 * @param f RX FIFO 结构体指针
 * @param d 要入队的字节
 * @return true 成功入队，false 队满
 */
static bool RxFifo_Enqueue(UartRxFifo_t *f, uint8_t d) {
    uint16_t next = (f->head + 1) & (UART_RX_FIFO_SIZE - 1);
    if (next == f->tail) return false;
    f->buffer[f->head] = d;
    f->head = next;
    return true;
}

/**
 * @brief 出队 RX FIFO
 * @param f RX FIFO 结构体指针
 * @param d 要出队的字节指针
 * @return true 成功出队，false 队空
 */
static bool RxFifo_Dequeue(UartRxFifo_t *f, uint8_t *d) {
    if (f->head == f->tail) return false;
    *d = f->buffer[f->tail];
    f->tail = (f->tail + 1) & (UART_RX_FIFO_SIZE - 1);
    return true;
}

/**
 * @brief 启动 TX DMA传输
 * @param t UART 传输结构体指针
 */
void UartTransport_StartTx(UartTransport_t *t) {
    if (t->huart->hdmatx->State != HAL_DMA_STATE_READY) {
        return;
    }
    uint16_t cnt = 0;
    while (cnt < UART_TX_CHUNK_SIZE && TxFifo_Dequeue(&t->tx_fifo, &t->tx_chunk[cnt])) {
        cnt++;
    }
    if (cnt > 0) {
        HAL_UART_Transmit_DMA(t->huart, t->tx_chunk, cnt);
    }
}
/**
 * @brief 发送字节序列
 * @param t UART 传输结构体指针
 * @param data 要发送的字节序列指针
 * @param len 要发送的字节数
 */
void UartTransport_SendBytes(UartTransport_t *t, const uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        TxFifo_Enqueue(&t->tx_fifo, data[i]);
    }
}


/**
 * @brief TX DMA传输完成回调函数
 * @param huart UART句柄指针
 */
/**
 * @brief 初始化 UART 传输结构体
 * @param t UART 传输结构体指针
 * @param huart UART句柄指针
 */
void UartTransport_Init(UartTransport_t *t, UART_HandleTypeDef *huart) {
    memset(t, 0, sizeof(UartTransport_t));
    t->huart = huart;

    HAL_UARTEx_ReceiveToIdle_DMA(
        huart,
        t->dma_rx_buf,
        UART_DMA_RX_SIZE);
}

/**
 * @brief 处理空闲中断
 * @param t UART 传输结构体指针
 */
void UartTransport_RxEventHandler(UartTransport_t *t, uint16_t size)
{
    uint16_t curr_pos = size;

    if (curr_pos >= UART_DMA_RX_SIZE)
    {
        curr_pos = 0U;
    }

    while (t->dma_last_pos != curr_pos)
    {
        RxFifo_Enqueue(&t->rx_fifo, t->dma_rx_buf[t->dma_last_pos]);
        t->dma_last_pos = (t->dma_last_pos + 1) & (UART_DMA_RX_SIZE - 1);
    }
}

/**
 * @brief 从 RX FIFO 读取字节
 * @param t UART 传输结构体指针
 * @param byte 要读取的字节指针
 * @return true 成功读取，false 队空
 */
bool UartTransport_ReadByte(UartTransport_t *t, uint8_t *byte) {
    return RxFifo_Dequeue(&t->rx_fifo, byte);
}

/**
 * @brief 获取 TX FIFO 中需要发送的字节数
 * @param t UART 传输结构体指针
 * @return 可用字节数
 */
uint16_t UartTransport_Available(UartTransport_t *t) {
    return (t->tx_fifo.head - t->tx_fifo.tail) & (UART_TX_FIFO_SIZE - 1);
}
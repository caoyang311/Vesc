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
#include "Fifo.h"

/* ============ Configuration ============ */
#define UART_DMA_RX_SIZE       256      /* DMA 接收缓冲区大小 */
#define UART_TX_CHUNK_SIZE     128      /* 发送块大小 */
#define UART_TX_FIFO_SIZE      1024     /* 发送 FIFO 大小 */
#define UART_RX_FIFO_SIZE      1024     /* 接收 FIFO 大小 */

/* RX FIFO structure (independent instance) */
typedef struct {
    uint8_t  buffer[UART_RX_FIFO_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} UartRxFifo_t;

/* TX FIFO structure (independent instance) */
typedef struct {
    uint8_t  buffer[UART_TX_FIFO_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} UartTxFifo_t;

/* Transport handle */
typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t            dma_rx_buf[UART_DMA_RX_SIZE];
    uint16_t           dma_last_pos;
    UartRxFifo_t       rx_fifo;
    UartTxFifo_t       tx_fifo;
    uint8_t            tx_chunk[UART_TX_CHUNK_SIZE];
} UartTransport_t;


/* ---- API ---- */
void UartTransport_Init(UartTransport_t *t, UART_HandleTypeDef *huart);
void UartTransport_RxEventHandler(UartTransport_t *t, uint16_t size);
void UartTransport_SendBytes(UartTransport_t *t, const uint8_t *data, uint16_t len);
bool UartTransport_ReadByte(UartTransport_t *t, uint8_t *byte);
uint16_t UartTransport_Available(UartTransport_t *t);
void UartTransport_StartTx(UartTransport_t *t);


#ifdef __cplusplus
}
#endif

#endif /* BOARD_UART1_H */

/**
 * @file Board_485.h
 * @brief USART3 RS485 driver interface.
 */
#ifndef BOARD_485_H
#define BOARD_485_H

#include "stm32f4xx_hal.h"
#include "usart.h"
#include <stdint.h>

#define RS485_UART_HANDLE      huart3
#define RS485_DE_PORT          GPIOI
#define RS485_DE_PIN           GPIO_PIN_11
#define RS485_RX_BUF_SIZE      (256U)
#define RS485_TX_BUFFER_SIZE   (256U)

typedef enum
{
    RS485_OK = 0,
    RS485_ERR_PARAM,
    RS485_ERR_TIMEOUT,
    RS485_ERR_NO_FRAME
} rs485_status_t;

void Rs485_Init(void);
rs485_status_t Rs485_SendFrame(const uint8_t *data, uint16_t len);
uint8_t Rs485_FrameAvailable(void);
rs485_status_t Rs485_GetFrame(uint8_t *out_buf, uint16_t *out_len);
void Rs485_FlushRx(void);
void Rs485_RxEventHandler(uint16_t size);

#endif /* BOARD_485_H */

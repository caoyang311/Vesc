/**
 * *****************************************************************************
 * @file        uart.h
 * @brief       串口底层驱动
 * @author       (caoyang)
 * @date        2023-11-17
 * @copyright   欢阳技有限公司
 * *****************************************************************************
 */


#ifndef UART_H 
#define UART_H 

/*----------------------------------include-----------------------------------*/
#include "misc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stdio.h"
/*-----------------------------------macro------------------------------------*/
/*-----------------------------------USART1_Config----------------------------*/
#define USART1_BAUDRATE      (4000000)                     //串口波特率

#define USART1_DR			((uint32_t)USART1+0x04)       //发送数据寄存器地址

#define USART1_DMA_TX_STREAM   DMA2_Stream7
#define USART1_DMA_RX_STREAM   DMA2_Stream5

#define USART1_TX_DMA_CHANNEL  DMA_Channel_4
#define USART1_RX_DMA_CHANNEL  DMA_Channel_4

#define USART1_DMA_TX_IRQn               DMA2_Stream7_IRQn   
#define USART1_DMA_RX_IRQn               DMA2_Stream5_IRQn  

#define USART1_DMA_TX_IRQHandler         DMA2_Stream7_IRQHandler   
#define USART1_DMA_RX_IRQHandler         DMA2_Stream5_IRQHandler  

#define USART1_DMA_TC_FLAG               DMA_FLAG_TCIF7   
#define USART1_DMA_RC_FLAG               DMA_FLAG_TCIF5   
/*-----------------------------------USART3_Config----------------------------*/
#define USART3_BAUDRATE      (2500000)                     //串口波特率

#define USART3_DR			((uint32_t)USART3+0x04)       //发送数据寄存器地址

#define USART3_DMA_TX_STREAM   DMA1_Stream3
#define USART3_DMA_RX_STREAM   DMA1_Stream1

#define USART3_TX_DMA_CHANNEL  DMA_Channel_4
#define USART3_RX_DMA_CHANNEL  DMA_Channel_4

#define USART3_MAX_RX_LEN      (16)
#define USART3_MAX_TX_LEN      (1)
/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void uart1_init(void);
void uart3_init(void);
void Uart1_DMASendConfig(u8 *buffer, u32 size);
void Uart3_DMASendConfig(u8 *buffer, u32 size);
/*------------------------------------test------------------------------------*/


#endif	/* UART_H */

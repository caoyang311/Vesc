/**
 * @file Com.h
 * @brief 日志记录器。
 */
#ifndef __COM_H
#define __COM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "Protocol.h"
#include "Justfloat.h"
#include "Board_Uart1.h"


/* ============ 调试开关 ============ */
#define APP_ENABLE_JUSTFLOAT    1   /* 1: 调试阶段开启，0: 发布时关闭 */


void Com_Uart_Init(UART_HandleTypeDef *huart);
void Com_RX_MainFunction(void);                /* 主循环调用 */
void Com_TX_MainFunction(void);                /* 主循环调用 */

/* ============ 发送接口 ============ */

/* 业务通道：发送自定义帧协议（参数下发、控制指令、应答等） */
void Com_Uart_SendFrame(const uint8_t *data, uint16_t len);

#if APP_ENABLE_JUSTFLOAT
/* 调试通道：发送 JustFloat 波形数据 */
void Com_Uart_SendJustFloat(const float *ch_data, uint8_t ch_count);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __LOG_H */

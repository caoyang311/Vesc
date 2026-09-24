/**
 * @file VescPacket.h
 * @brief VESC Packet 串口协议接口。
 */
#ifndef PROTOCOL_VESC_PACKET_H
#define PROTOCOL_VESC_PACKET_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    VESC_PACKET_RESULT_OK = 0U,
    VESC_PACKET_RESULT_NOT_INITIALIZED,
    VESC_PACKET_RESULT_INVALID_PARAMETER,
    VESC_PACKET_RESULT_NO_DATA,
    VESC_PACKET_RESULT_FRAME_ERROR,
    VESC_PACKET_RESULT_BUFFER_ERROR,
    VESC_PACKET_RESULT_UART_ERROR
} VescPacket_ResultType;

/** @brief 初始化 VESC Packet 协议状态。 */
void VescPacket_Init(void);

/**
 * @brief 处理 UART1 中的全部待处理字节。
 * @return 本次处理结果。
 */
VescPacket_ResultType VescPacket_Task(void);

/**
 * @brief 输入一个串口字节并驱动协议状态机。
 * @param[in] Byte 输入字节。
 * @return 字节处理结果。
 */
VescPacket_ResultType packet_process_byte(uint8_t Byte);

/**
 * @brief 计算 VESC Packet CRC16-CCITT。
 * @param[in] Data 数据区。
 * @param[in] Length 数据长度。
 * @return CRC16 校验值。
 */
uint16_t VescPacket_Crc16(const uint8_t *Data, uint16_t Length);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_VESC_PACKET_H */

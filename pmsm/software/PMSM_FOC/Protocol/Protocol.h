/**
 * @file Protocol.h
 * @brief 协议接口。
 */
#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ================= Frame Format =================
 * +-------+-------+--------+---------+-------+-------+-------+-------+
 * | HEAD1 | HEAD2 | LENGTH |  DATA   | CRC_L | CRC_H | TAIL1 | TAIL2 |
 * | 0xAA  | 0x55  | 1 byte | N bytes | 1 byte| 1 byte| 0x0D  | 0x0A  |
 * +-------+-------+--------+---------+-------+-------+-------+-------+
 * ================================================ */

#define PROTO_HEAD1             0xAA
#define PROTO_HEAD2             0x55
#define PROTO_TAIL1             0x0D
#define PROTO_TAIL2             0x0A

#define PROTO_DATA_MAX_LEN      20
#define PROTO_FRAME_OVERHEAD    6   /* HEAD1+HEAD2+LEN+CRC(2)+TAIL1+TAIL2 */
#define PROTO_FRAME_MAX_SIZE    (PROTO_DATA_MAX_LEN + PROTO_FRAME_OVERHEAD)

/* Parser states */
typedef enum {
    PROTO_STATE_HEAD1 = 0,
    PROTO_STATE_HEAD2,
    PROTO_STATE_LENGTH,
    PROTO_STATE_DATA,
    PROTO_STATE_CRC_LOW,
    PROTO_STATE_CRC_HIGH,
    PROTO_STATE_TAIL1,
    PROTO_STATE_TAIL2
} ProtoState_t;

/* Parsed frame */
typedef struct {
    uint8_t  data[PROTO_DATA_MAX_LEN];
    uint16_t length;
    uint16_t crc;
} ProtoFrame_t;

/* Parser context */
typedef struct {
    ProtoState_t state;
    ProtoFrame_t frame;
    uint8_t      data_index;
    uint16_t     crc_received;
    uint16_t     crc_calculated;
} ProtoParser_t;

/* ---- API ---- */
void     Proto_Init(ProtoParser_t *parser);
bool     Proto_ParseByte(ProtoParser_t *parser, uint8_t byte);
uint16_t Proto_BuildFrame(const uint8_t *data, uint16_t len,
                          uint8_t *out_buf, uint16_t out_size);
uint16_t Proto_CRC16(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_H */

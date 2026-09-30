#include "Protocol.h"
#include <string.h>

/**
 * @brief 计算 CRC16 校验和。
 * @param data 数据指针。
 * @param len 数据长度。
 * @return CRC16 校验和。
 */
uint16_t Proto_CRC16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief 初始化协议解析器。
 * @param parser 协议解析器指针。
 */
void Proto_Init(ProtoParser_t *parser) {
    memset(parser, 0, sizeof(ProtoParser_t));
    parser->state = PROTO_STATE_HEAD1;
}

/**
 * @brief 解析协议字节。
 * @param parser 协议解析器指针。
 * @param byte 待解析字节。
 * @return true 如果解析到完整帧，false 否则。
 */
bool Proto_ParseByte(ProtoParser_t *parser, uint8_t byte) {
    switch (parser->state) {

    case PROTO_STATE_HEAD1:
        if (byte == PROTO_HEAD1) {
            parser->state = PROTO_STATE_HEAD2;
        }
        break;

    case PROTO_STATE_HEAD2:
        if (byte == PROTO_HEAD2) {
            parser->state = PROTO_STATE_LENGTH;
        } else if (byte != PROTO_HEAD1) {
            parser->state = PROTO_STATE_HEAD1;
        }
        break;

    case PROTO_STATE_LENGTH:
        if (byte == 0 || byte > PROTO_DATA_MAX_LEN) {
            parser->state = PROTO_STATE_HEAD1;
            break;
        }
        parser->frame.length = byte;
        parser->data_index = 0;
        parser->state = PROTO_STATE_DATA;
        break;

    case PROTO_STATE_DATA:
        parser->frame.data[parser->data_index++] = byte;
        if (parser->data_index >= parser->frame.length) {
            parser->state = PROTO_STATE_CRC_LOW;
        }
        break;

    case PROTO_STATE_CRC_LOW:
        parser->crc_received = byte;
        parser->state = PROTO_STATE_CRC_HIGH;
        break;

    case PROTO_STATE_CRC_HIGH:
        parser->crc_received |= ((uint16_t)byte << 8);
        parser->crc_calculated = Proto_CRC16(parser->frame.data,
                                             parser->frame.length);
        if (parser->crc_calculated != parser->crc_received) {
            parser->state = PROTO_STATE_HEAD1;
        } else {
            parser->state = PROTO_STATE_TAIL1;
        }
        break;

    case PROTO_STATE_TAIL1:
        parser->state = (byte == PROTO_TAIL1)
                        ? PROTO_STATE_TAIL2
                        : PROTO_STATE_HEAD1;
        break;

    case PROTO_STATE_TAIL2:
        parser->state = PROTO_STATE_HEAD1;
        if (byte == PROTO_TAIL2) {
            parser->frame.crc = parser->crc_received;
            return true;    /* Complete frame */
        }
        break;

    default:
        parser->state = PROTO_STATE_HEAD1;
        break;
    }
    return false;
}

/**
 * @brief 构建协议帧。
 * @param data 数据指针。
 * @param len 数据长度。
 * @param out_buf 输出缓冲区指针。
 * @param out_size 输出缓冲区大小。
 * @return 构建的字节数。
 */
uint16_t Proto_BuildFrame(const uint8_t *data, uint16_t len,
                          uint8_t *out_buf, uint16_t out_size) {
    uint16_t total = len + PROTO_FRAME_OVERHEAD;
    if (len == 0 || len > PROTO_DATA_MAX_LEN || total > out_size) {
        return 0;
    }

    uint16_t idx = 0;
    out_buf[idx++] = PROTO_HEAD1;
    out_buf[idx++] = PROTO_HEAD2;
    out_buf[idx++] = (uint8_t)len;
    memcpy(&out_buf[idx], data, len);
    idx += len;

    uint16_t crc = Proto_CRC16(data, len);
    out_buf[idx++] = crc & 0xFF;
    out_buf[idx++] = (crc >> 8) & 0xFF;
    out_buf[idx++] = PROTO_TAIL1;
    out_buf[idx++] = PROTO_TAIL2;

    return idx;
}
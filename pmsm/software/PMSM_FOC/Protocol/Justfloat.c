#include "Justfloat.h"
#include <string.h>
/**
 * @brief 构建 JustFloat 帧
 * @param ch_data � 浮点数据数组
 * @param ch_count 通道数（float 个数）
 * @param out_buf 输出缓冲区，需至少 (ch_count * 4 + 4) 字节
 * @param out_size 输出缓冲区大小
 * @return 实际写入的字节数，0 表示失败
 */
uint16_t JustFloat_Build(const float *ch_data, uint8_t ch_count,
                         uint8_t *out_buf, uint16_t out_size)
{
    if (ch_data == NULL || out_buf == NULL || ch_count == 0) {
        return 0;
    }
    if (ch_count > JUSTFLOAT_MAX_CHANNELS) {
        return 0;
    }

    uint16_t data_len = (uint16_t)ch_count * sizeof(float);
    uint16_t total_len = data_len + JUSTFLOAT_TAIL_SIZE;

    if (total_len > out_size) {
        return 0;
    }

    /* 直接内存拷贝（Cortex-M 为小端，无需字节序转换） */
    memcpy(out_buf, ch_data, data_len);

    /* 写入帧尾 */
    out_buf[data_len + 0] = JUSTFLOAT_TAIL_0;
    out_buf[data_len + 1] = JUSTFLOAT_TAIL_1;
    out_buf[data_len + 2] = JUSTFLOAT_TAIL_2;
    out_buf[data_len + 3] = JUSTFLOAT_TAIL_3;

    return total_len;
}
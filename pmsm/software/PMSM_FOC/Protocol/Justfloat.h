/**
 * @file Justfloat.h
 * @brief Justfloat 类型定义。
 */
#ifndef __JUSTFLOATS_H
#define __JUSTFLOATS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ================= JustFloat Frame Format =================
 * +--------------+--------------+-----+--------------+---------------------+
 * | float ch[0]  | float ch[1]  | ... | float ch[N-1]| Tail (4 bytes)       |
 * | 4 bytes      | 4 bytes      |     | 4 bytes      | 0x00 0x00 0x80 0x7F |
 * +--------------+--------------+-----+--------------+---------------------+
 *
 * - float 必须为小端字节序（ARM Cortex-M 默认小端）
 * - 通道数 = (帧总字节数 - 4) / 4
 * - VOFA+ 根据帧长自动推断通道数
 * ============================================================ */

#define JUSTFLOAT_TAIL_SIZE     4
#define JUSTFLOAT_TAIL_0        0x00
#define JUSTFLOAT_TAIL_1        0x00
#define JUSTFLOAT_TAIL_2        0x80
#define JUSTFLOAT_TAIL_3        0x7F

/* 默认最大通道数（可根据需要调整） */
#define JUSTFLOAT_MAX_CHANNELS  16

/**
 * @brief  构建 JustFloat 帧
 * @param  ch_data   浮点数据数组
 * @param  ch_count  通道数（float 个数）
 * @param  out_buf   输出缓冲区，需至少 (ch_count * 4 + 4) 字节
 * @param  out_size  输出缓冲区大小
 * @return 实际写入的字节数，0 表示失败
 */
uint16_t JustFloat_Build(const float *ch_data, uint8_t ch_count,
                         uint8_t *out_buf, uint16_t out_size);



#ifdef __cplusplus
}
#endif

#endif /* __JUSTFLOATS_H */

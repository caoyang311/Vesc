/**
 * @file FocMovingAverage.h
 * @brief FOC滑动平均滤波器接口声明。
 */

#ifndef FOC_MOVING_AVERAGE_H
#define FOC_MOVING_AVERAGE_H

#include <stdbool.h>
#include <stdint.h>

#define FOC_MOVING_AVERAGE_WINDOW_SIZE (10U)

/**
 * @brief 窗口长度为10的单精度滑动平均滤波器上下文。
 */
typedef struct
{
    float buffer[FOC_MOVING_AVERAGE_WINDOW_SIZE]; /**< 历史采样缓冲区。 */
    float sum;                                    /**< 当前窗口采样和。 */
    uint8_t index;                                /**< 下一次写入位置。 */
    uint8_t count;                                /**< 当前有效采样数量。 */
} FocMovingAverage_ContextType;

/**
 * @brief 初始化滑动平均滤波器。
 *
 * @param[out] context 滤波器上下文，指针不得为NULL。
 *
 * @post 参数有效时清空缓冲区、累加和、索引及有效采样数量。
 */
void FocMovingAverage_Init(FocMovingAverage_ContextType * context);

/**
 * @brief 向滑动平均滤波器输入一个采样并获取滤波结果。
 *
 * 窗口填满前使用当前有效采样数量计算平均值；窗口填满后固定使用最近
 * 10个采样计算平均值。算法每次调用执行时间有界。
 *
 * @param[in,out] context 滤波器上下文，指针不得为NULL。
 * @param[in] sample 新输入的单精度浮点采样。
 * @param[out] output 滤波结果，指针不得为NULL。
 *
 * @return 参数有效并完成滤波时返回true，否则返回false。
 */
void FocMovingAverage_Update(FocMovingAverage_ContextType * context,
                             float sample,
                             float * output);

#endif /* FOC_MOVING_AVERAGE_H */

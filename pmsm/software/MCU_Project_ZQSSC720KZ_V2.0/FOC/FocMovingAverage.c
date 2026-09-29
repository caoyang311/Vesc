/**
 * @file FocMovingAverage.c
 * @brief FOC滑动平均滤波器实现。
 */

#include "FocMovingAverage.h"

#include <stddef.h>

/**
 * @brief 初始化滑动平均滤波器。
 *
 * @param[out] context 滤波器上下文，指针不得为NULL。
 *
 * @post 参数有效时清空全部内部状态。
 */
void FocMovingAverage_Init(FocMovingAverage_ContextType * context)
{
    uint8_t index = 0U;

    if (context != NULL)
    {
        for (index = 0U; index < FOC_MOVING_AVERAGE_WINDOW_SIZE; index++)
        {
            context->buffer[index] = 0.0F;
        }

        context->sum = 0.0F;
        context->index = 0U;
        context->count = 0U;
    }
    else
    {
        /* 无效上下文指针不执行初始化。 */
    }
}

/**
 * @brief 更新滑动平均滤波器并输出当前平均值。
 *
 * 通过从累加和中减去最旧采样并加入新采样实现固定执行时间更新，避免
 * 每次调用重新遍历整个窗口。
 *
 * @param[in,out] context 滤波器上下文，指针不得为NULL。
 * @param[in] sample 新输入的单精度浮点采样。
 * @param[out] output 滤波结果，指针不得为NULL。
 *
 * @return 参数有效时返回true，否则返回false。
 */
void FocMovingAverage_Update(FocMovingAverage_ContextType * context,
                             float sample,
                             float * output)
{
    float result = 0.0F;

    if ((context->index < FOC_MOVING_AVERAGE_WINDOW_SIZE) &&
        (context->count <= FOC_MOVING_AVERAGE_WINDOW_SIZE))
    {
        context->sum -= context->buffer[context->index];
        context->buffer[context->index] = sample;
        context->sum += sample;

        context->index++;
        if (context->index >= FOC_MOVING_AVERAGE_WINDOW_SIZE)
        {
            context->index = 0U;
        }
        else
        {
            /* 写入索引尚未到达缓冲区末尾。 */
        }

        if (context->count < FOC_MOVING_AVERAGE_WINDOW_SIZE)
        {
            context->count++;
        }
        else
        {
            /* 窗口已满，有效采样数量保持不变。 */
        }

        result = context->sum / (float)context->count;
        *output = result;
    }
    else
    {
        /* 参数或上下文状态无效时不修改输出。 */
    }

}

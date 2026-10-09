#include "FocFilter.h"

/**
 * @brief 初始化低通滤波器。
 *
 * @param f 滤波器结构体指针。
 * @param alpha 滤波系数 α (0 < alpha <= 1)。
 */
void FocLpf_Init(LPF_Filter *f, float alpha)
{
    f->alpha = alpha;
    f->y = 0.0f;   // 初始输出，可按需设为第一个采样值
}

/**
 * @brief 更新低通滤波器的输出。
 *
 * @param f 滤波器结构体指针。
 * @param x 输入采样值。
 * @return float 滤波后的输出。
 */
float FocLpf_Update(LPF_Filter *f, float x)
{
    f->y = f->alpha * x + (1.0f - f->alpha) * f->y;
    return f->y;
}
#ifndef FOC_FILTER_H
#define FOC_FILTER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float alpha;   // 滤波系数 α (0 < alpha <= 1)
    float y;       // 上一次的输出 y[n-1]
} LPF_Filter;

void FocLpf_Init(LPF_Filter *f, float alpha);
float FocLpf_Update(LPF_Filter *f, float x);

#endif /* FOC_FILTER_H */

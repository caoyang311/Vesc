#ifndef FOC_TRIG_TABLE_H
#define FOC_TRIG_TABLE_H

#include <stdint.h>

#include "FocAlgorithm.h"

/**
 * @brief 根据完整电周期相位获取正弦和余弦值。
 *
 * 使用四分之一周期257点正弦表、象限映射和线性插值。函数执行时间
 * 有界，不访问硬件、不分配内存且不调用数学库三角函数。
 *
 * @param[in] phase 完整电周期相位，0x0000至0xFFFF对应0至接近360度。
 * @param[out] trig 正弦和余弦输出指针；允许为NULL，NULL时不执行写操作。
 *
 * @return None.
 */
void FocTrig_GetSinCos(uint16_t phase, Foc_TrigType * trig);

#endif /* FOC_TRIG_TABLE_H */

#ifndef RESET_H
#define RESET_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

typedef enum
{
  RESET_CAUSE_UNKNOWN = 0U, /* 未知复位 */
  RESET_CAUSE_IWDG, /* IWDG 复位 */
  RESET_CAUSE_WWDG, /* WWDG 复位 */
  RESET_CAUSE_SOFTWARE, /* 软件复位 */
  RESET_CAUSE_POWER_ON, /* 上电复位 */
  RESET_CAUSE_BROWN_OUT, /* 低电压复位 */
  RESET_CAUSE_EXTERNAL, /* 外部复位 */
  RESET_CAUSE_LOW_POWER /* 低功耗复位 */
} ResetCause;

/**
 * @brief 初始化重置模块
 * 
 * 解码重置标志位，设置复位原因
 */
void Reset_Init(void);
/**
 * @brief 获取当前重置原因
 * 
 * @return ResetCause 当前重置原因
 */
ResetCause Reset_GetCause(void);
/**
 * @brief 清除重置标志位
 * 
 * 清除RCC寄存器中的重置标志位，准备下一次重置
 */
void Reset_ClearFlags(void);

#ifdef __cplusplus
}
#endif

#endif /* RESET_H */

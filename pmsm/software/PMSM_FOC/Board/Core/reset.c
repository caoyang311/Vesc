#include "reset.h"

static uint32_t reset_raw_flags = 0U; /* 原始重置标志位 */
static ResetCause reset_cause = RESET_CAUSE_UNKNOWN; /* 复位原因 */
/**
 * @brief 解码重置标志位，返回复位原因
 * 
 * @param flags 重置标志位
 * @return ResetCause 复位原因
 */
static ResetCause Reset_DecodeCause(uint32_t flags)
{
  ResetCause cause = RESET_CAUSE_UNKNOWN;

  if ((flags & RCC_FLAG_IWDGRST) != 0U)
  {
    cause = RESET_CAUSE_IWDG; /* IWDG 复位 */
  }
  else if ((flags & RCC_FLAG_WWDGRST) != 0U)
  {
    cause = RESET_CAUSE_WWDG; /* WWDG 复位 */
  }
  else if ((flags & RCC_FLAG_SFTRST) != 0U)
  {
    cause = RESET_CAUSE_SOFTWARE;/* 软件复位 */
  }
  else if ((flags & RCC_FLAG_BORRST) != 0U)
  {
    cause = RESET_CAUSE_BROWN_OUT; /* 低电压复位 */
  }
  else if ((flags & RCC_FLAG_PORRST) != 0U)
  {
    cause = RESET_CAUSE_POWER_ON; /* 上电复位 */
  }
  else if ((flags & RCC_FLAG_PINRST) != 0U)
  {
    cause = RESET_CAUSE_EXTERNAL;/* 外部复位 */
  }
  else if ((flags & RCC_FLAG_LPWRRST) != 0U)
  {
    cause = RESET_CAUSE_LOW_POWER;/* 低功耗复位 */
  }
  else
  {
    cause = RESET_CAUSE_UNKNOWN;/* 未知复位 */
  }

  return cause;
}
/**
 * @brief 初始化重置模块
 * 
 * 解码重置标志位，设置复位原因
 */
void Reset_Init(void)
{
  reset_raw_flags = RCC->CSR;
  reset_cause = Reset_DecodeCause(reset_raw_flags);
}

/**
 * @brief 获取当前重置原因
 * 
 * @return ResetCause 当前重置原因
 */
ResetCause Reset_GetCause(void)
{
  return reset_cause;
}
/**
 * @brief 清除重置标志位
 * 
 * 清除RCC寄存器中的重置标志位，准备下一次重置
 */
void Reset_ClearFlags(void)
{
  __HAL_RCC_CLEAR_RESET_FLAGS();
}

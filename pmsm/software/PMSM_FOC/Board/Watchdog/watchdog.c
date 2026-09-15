#include "watchdog.h"

/**
 * @brief 使能IWDG狗
 * 
 */
void Watchdog_Enable(void)
{
  (void)__HAL_IWDG_START (&hiwdg);
}

/**
 * @brief 看门狗刷新
 * 
 */
void Watchdog_Refresh(void)
{
  (void)HAL_IWDG_Refresh (&hiwdg);
}

/**
 * @brief 重新设置IWDG狗参数
 * 
 * @param timeout_ms 超时时间，单位毫秒
 */
void Watchdog_Reload(uint32_t timeout_ms)
{
  // 1. 先喂狗，重置计数器（防止修改期间超时复位）
  HAL_IWDG_Refresh(&hiwdg);

  // 2. 解锁 PR 和 RLR 寄存器（向 KR 写入 0x5555）
  IWDG->KR = 0x5555;

  // 3. 等待状态寄存器 SR 的 PVU 和 RVU 位清零（表示寄存器可更新）
  while (IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU));

  // 4. 写入新的分频值和重装载值
  IWDG->PR = IWDG_PRESCALER_32;  // 例如改为 32 分频
  IWDG->RLR = timeout_ms;        // 例如重装载值为 timeout_ms

  // 5. 再次喂狗，让新配置的装载值生效
  HAL_IWDG_Refresh(&hiwdg);
}

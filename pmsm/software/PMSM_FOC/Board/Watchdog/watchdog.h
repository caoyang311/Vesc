#ifndef WATCHDOG_H
#define WATCHDOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "iwdg.h"

/**
 * @brief 使能IWDG狗
 * 
 */
void Watchdog_Enable(void);
/**
 * @brief 看门狗刷新
 * 
 */
void Watchdog_Refresh(void);

/**
 * @brief 重新设置IWDG狗参数
 * 
 * @param timeout_ms 超时时间，单位毫秒
 */
void Watchdog_Reload(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* WATCHDOG_H */

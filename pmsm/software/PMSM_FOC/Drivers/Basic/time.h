#ifndef TIME_H
#define TIME_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

uint32_t Time_GetTickMs(void);
uint32_t Time_GetElapsedMs(uint32_t start_tick);
uint8_t Time_IsElapsed(uint32_t start_tick, uint32_t period_ms);

#ifdef __cplusplus
}
#endif

#endif /* TIME_H */

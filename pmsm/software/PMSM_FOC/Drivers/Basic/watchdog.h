#ifndef WATCHDOG_H
#define WATCHDOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "iwdg.h"

typedef enum
{
  WATCHDOG_SERVICE_MAIN_LOOP = 0U,
  WATCHDOG_SERVICE_TIME_BASE,
  WATCHDOG_SERVICE_DMA,
  WATCHDOG_SERVICE_COUNT
} WatchdogServiceId;

#define WATCHDOG_SERVICE_MASK(id) (1UL << (id))

void Watchdog_Init(void);
void Watchdog_RegisterHeartbeat(uint32_t service_id);
void Watchdog_Alive(uint32_t service_id);
void Watchdog_MainFunction(void);
void Watchdog_Suspend(void);
uint32_t Watchdog_GetEnabledServices(void);
uint32_t Watchdog_GetAliveServices(void);

#ifdef __cplusplus
}
#endif

#endif /* WATCHDOG_H */

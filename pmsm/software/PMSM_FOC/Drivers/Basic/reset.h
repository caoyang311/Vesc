#ifndef RESET_H
#define RESET_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

typedef enum
{
  RESET_CAUSE_UNKNOWN = 0U,
  RESET_CAUSE_IWDG,
  RESET_CAUSE_WWDG,
  RESET_CAUSE_SOFTWARE,
  RESET_CAUSE_POWER_ON,
  RESET_CAUSE_BROWN_OUT,
  RESET_CAUSE_EXTERNAL,
  RESET_CAUSE_LOW_POWER
} ResetCause;

void Reset_Init(void);
uint32_t Reset_GetRawFlags(void);
ResetCause Reset_GetCause(void);
void Reset_ClearFlags(void);

#ifdef __cplusplus
}
#endif

#endif /* RESET_H */

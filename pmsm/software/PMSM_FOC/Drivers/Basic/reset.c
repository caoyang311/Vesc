#include "reset.h"

static uint32_t reset_raw_flags = 0U;
static ResetCause reset_cause = RESET_CAUSE_UNKNOWN;

static ResetCause Reset_DecodeCause(uint32_t flags)
{
  ResetCause cause = RESET_CAUSE_UNKNOWN;

  if ((flags & RCC_FLAG_IWDGRST) != 0U)
  {
    cause = RESET_CAUSE_IWDG;
  }
  else if ((flags & RCC_FLAG_WWDGRST) != 0U)
  {
    cause = RESET_CAUSE_WWDG;
  }
  else if ((flags & RCC_FLAG_SFTRST) != 0U)
  {
    cause = RESET_CAUSE_SOFTWARE;
  }
  else if ((flags & RCC_FLAG_BORRST) != 0U)
  {
    cause = RESET_CAUSE_BROWN_OUT;
  }
  else if ((flags & RCC_FLAG_PORRST) != 0U)
  {
    cause = RESET_CAUSE_POWER_ON;
  }
  else if ((flags & RCC_FLAG_PINRST) != 0U)
  {
    cause = RESET_CAUSE_EXTERNAL;
  }
  else if ((flags & RCC_FLAG_LPWRRST) != 0U)
  {
    cause = RESET_CAUSE_LOW_POWER;
  }
  else
  {
    cause = RESET_CAUSE_UNKNOWN;
  }

  return cause;
}

void Reset_Init(void)
{
  reset_raw_flags = RCC->CSR;
  reset_cause = Reset_DecodeCause(reset_raw_flags);
}

uint32_t Reset_GetRawFlags(void)
{
  return reset_raw_flags;
}

ResetCause Reset_GetCause(void)
{
  return reset_cause;
}

void Reset_ClearFlags(void)
{
  __HAL_RCC_CLEAR_RESET_FLAGS();
}

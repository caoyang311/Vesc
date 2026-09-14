#include "time.h"

uint32_t Time_GetTickMs(void)
{
  return HAL_GetTick();
}

uint32_t Time_GetElapsedMs(uint32_t start_tick)
{
  uint32_t current_tick = Time_GetTickMs();

  return (current_tick - start_tick);
}

uint8_t Time_IsElapsed(uint32_t start_tick, uint32_t period_ms)
{
  uint8_t elapsed = 0U;

  if (Time_GetElapsedMs(start_tick) >= period_ms)
  {
    elapsed = 1U;
  }
  else
  {
    elapsed = 0U;
  }

  return elapsed;
}

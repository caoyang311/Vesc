#include "watchdog.h"

static uint32_t watchdog_enabled_services = 0U;
static uint32_t watchdog_alive_services = 0U;
static uint8_t watchdog_suspended = 0U;

void Watchdog_Init(void)
{
  watchdog_enabled_services = 0U;
  watchdog_alive_services = 0U;
  watchdog_suspended = 0U;
}

void Watchdog_RegisterHeartbeat(uint32_t service_id)
{
  if (service_id < (uint32_t)WATCHDOG_SERVICE_COUNT)
  {
    watchdog_enabled_services |= WATCHDOG_SERVICE_MASK(service_id);
  }
  else
  {
    /* Invalid service identifiers are ignored. */
  }
}

void Watchdog_Alive(uint32_t service_id)
{
  if (service_id < (uint32_t)WATCHDOG_SERVICE_COUNT)
  {
    watchdog_alive_services |= WATCHDOG_SERVICE_MASK(service_id);
  }
  else
  {
    /* Invalid service identifiers are ignored. */
  }
}

void Watchdog_MainFunction(void)
{
  if (watchdog_suspended == 0U)
  {
    if (watchdog_alive_services == watchdog_enabled_services)
    {
      (void)HAL_IWDG_Refresh(&hiwdg);
      watchdog_alive_services = 0U;
    }
    else
    {
      /* At least one enabled service has not reported its heartbeat. */
    }
  }
  else
  {
    /* Watchdog refresh is intentionally suspended. */
  }
}

void Watchdog_Suspend(void)
{
  watchdog_suspended = 1U;
}

uint32_t Watchdog_GetEnabledServices(void)
{
  return watchdog_enabled_services;
}

uint32_t Watchdog_GetAliveServices(void)
{
  return watchdog_alive_services;
}

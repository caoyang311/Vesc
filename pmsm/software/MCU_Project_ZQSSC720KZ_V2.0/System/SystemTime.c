#include "SystemTime.h"
#include "tim.h"

static volatile uint32_t SystemTime_TickMs;

/**
 * @brief Initializes the millisecond system time.
 *
 * @post SystemTime_TickMs is zero.
 */
void SystemTime_Init(void)
{
    SystemTime_TickMs = 0U;
}

/**
 * @brief Starts the TIM6 update interrupt used as the 1 ms time base.
 *
 * @return true if the HAL starts TIM6 successfully; otherwise false.
 *
 * @pre TIM6 has been initialized by the hardware initialization layer.
 */
bool SystemTime_Start(void)
{
    (void)HAL_TIM_Base_Start_IT(&htim1);
    return HAL_TIM_Base_Start_IT(&htim6) == HAL_OK;
}

/**
 * @brief Returns an atomic snapshot of the millisecond system time.
 *
 * @return Milliseconds elapsed since initialization, modulo 2^32.
 *
 * @note This relies on atomic aligned 32-bit access on the Cortex-M4 target.
 */
uint32_t SystemTime_GetTimeMs(void)
{
    return SystemTime_TickMs;
}

/**
 * @brief Advances the system time by one millisecond.
 *
 * This single-writer function executes in TIM6 interrupt context, is bounded,
 * and does not block. The counter wraps naturally modulo 2^32.
 *
 * @pre SystemTime_Init() has completed.
 */
void SystemTime_Notification1ms(void)
{
    ++SystemTime_TickMs;
}

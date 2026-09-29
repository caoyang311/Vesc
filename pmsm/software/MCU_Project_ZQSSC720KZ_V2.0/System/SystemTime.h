#ifndef SYSTEM_TIME_H
#define SYSTEM_TIME_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initializes the millisecond system time.
 *
 * @post The system time value is zero.
 */
void SystemTime_Init(void);

/**
 * @brief Starts the TIM6 update interrupt used as the 1 ms time base.
 *
 * @return true if the timer interrupt starts successfully; otherwise false.
 *
 * @pre SystemTime_Init() has completed and TIM6 has been configured.
 */
bool SystemTime_Start(void);

/**
 * @brief Returns the current system time in milliseconds.
 *
 * @return Milliseconds elapsed since initialization, modulo 2^32.
 *
 * @note The target shall support atomic aligned 32-bit reads shared with the
 *       TIM6 interrupt context.
 */
uint32_t SystemTime_GetTimeMs(void);

/**
 * @brief Advances the system time by one millisecond.
 *
 * This function is called only from the TIM6 interrupt callback. It is
 * bounded and non-blocking. The counter wraps naturally modulo 2^32.
 *
 * @pre SystemTime_Init() has completed.
 */
void SystemTime_Notification1ms(void);

#endif /* SYSTEM_TIME_H */

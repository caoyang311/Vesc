#include "Led.h"
#include "Dio.h"

static uint8_t LedBlink_Tick;

/**
 * @brief Initializes the LED blinking application.
 *
 * Resets the private blink counter and drives the configured LED channel low.
 *
 * @post LedBlink_Tick is zero and the LED channel is set to STD_LOW.
 */
void Led_Init(void)
{
    LedBlink_Tick = 0U;
    Dio_WriteChannel(DIO_CHANNEL_LED,STD_LOW);
}

/**
 * @brief Executes the 10 ms LED blinking task.
 *
 * Increments the private blink counter and reverses the configured LED level
 * every 50 calls. This main-loop function is non-blocking and non-reentrant.
 *
 * @pre Led_Init() has completed and calls occur at 10 ms intervals.
 */
void Led_Task(void)
{
    LedBlink_Tick++;
    if (LedBlink_Tick >= 50U)
    {
        LedBlink_Tick = 0U;
        Dio_FlipChannel(DIO_CHANNEL_LED);
    }
}

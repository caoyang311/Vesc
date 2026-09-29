#ifndef LED_H
#define LED_H

/**
 * @brief Initializes the LED blinking application.
 *
 * Resets the blink counter and drives the configured LED channel low.
 *
 * @post The blink counter is zero and the LED channel is set to STD_LOW.
 */
void Led_Init(void);

/**
 * @brief Executes the periodic LED blinking task.
 *
 * This function shall be called every 10 ms from the cooperative main-loop
 * scheduler. It reverses the LED level after 50 calls, producing a 500 ms
 * level duration. The function is non-blocking and shall not be called
 * concurrently or from interrupt context.
 *
 * @pre Led_Init() has completed.
 */
void Led_Task(void);

#endif /* LED_H */

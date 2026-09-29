#include "MotorApp.h"

#include "Key.h"
#include "Led.h"
#include "Motor.h"

#define MOTOR_APP_LED_BLINK_PERIOD_MS (60U)

static uint8_t MotorApp_PreviousKeyStates = 0U;
static uint8_t MotorApp_PreviousMotorRunning = 0U;

static void MotorApp_HandleKeys(void);
static void MotorApp_HandleLed(void);
/**
 * @brief 检查指定按键是否被按下。
 *
 * @param[in] CurrentKeyStates 当前按键状态掩码。
 * @param[in] KeyMask 指键掩码。
 * @return uint8_t 1U 表示按键被按下，0U 表示按键未被按下。
 */
static uint8_t MotorApp_IsKeyPressed(
    uint8_t CurrentKeyStates,
    uint8_t KeyMask)
{
    return (((CurrentKeyStates & KeyMask) != 0U) &&
            ((MotorApp_PreviousKeyStates & KeyMask) == 0U)) ?
           1U : 0U;
}
/**
 * @brief 初始化电机应用。
 */
void MotorApp_Init(void)
{
    MotorApp_PreviousKeyStates = Key_GetStateMask();
    MotorApp_PreviousMotorRunning = 0U;
    (void)Led_Off(LED_ID_0);
}
/**
 * @brief 处理按键的逻辑。
 */
static void MotorApp_HandleKeys(void)
{
    uint8_t current_key_states = Key_GetStateMask();

    if (MotorApp_IsKeyPressed(current_key_states, KEY_STATE_MASK(KEY_ID_0)) != 0U)
    {
        Motor_Start();
    }
    else
    {
        /* KEY0 当前没有新的按下事件。 */
    }

    if (MotorApp_IsKeyPressed(current_key_states, KEY_STATE_MASK(KEY_ID_1)) != 0U)
    {
        Motor_Stop();
    }
    else
    {
        /* KEY1 当前没有新的按下事件。 */
    }

    if (MotorApp_IsKeyPressed(current_key_states, KEY_STATE_MASK(KEY_ID_2)) != 0U)
    {

            Motor_ModeType next_mode =(Motor_ModeType)(Motor_GetMode() + 1U);
            if (next_mode >= MOTOR_MODE_COUNT)
            {
                next_mode = MOTOR_MODE_VF;
            }
            else
            {
                /* 当前模式切换有效。 */
            }
            Motor_SetMode(next_mode);
    }
    else
    {
        /* KEY2 当前没有新的按下事件。 */
    }

    MotorApp_PreviousKeyStates = current_key_states;
}
/**
 * @brief 处理LED的逻辑。
 */
static void MotorApp_HandleLed(void)
{
    uint8_t motor_is_running = Motor_GetIsRunning() ? 1U : 0U;

    if (motor_is_running != 0U)
    {
        if (MotorApp_PreviousMotorRunning == 0U)
        {
            (void)Led_Blink(LED_ID_0, MOTOR_APP_LED_BLINK_PERIOD_MS);
        }
        else
        {
            /* LED0 已经处于周期闪烁状态。 */
        }
    }
    else
    {
        (void)Led_Off(LED_ID_0);
    }

    MotorApp_PreviousMotorRunning = motor_is_running;
}
/**
 * @brief 主函数，处理电机应用的逻辑。
 */
void MotorApp_MainFunction(void)
{
    MotorApp_HandleKeys();
    MotorApp_HandleLed();
}

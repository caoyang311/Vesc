#include "Led.h"

#include "Dio.h"
#include "SystemTime.h"

static const Dio_ChannelType Led_DioChannels[LED_ID_COUNT] =
{
    DIO_CHANNEL_LED0,
    DIO_CHANNEL_LED1
};

static uint8_t Led_States[LED_ID_COUNT] = {LED_STATE_OFF};
static uint8_t Led_BlinkEnabled[LED_ID_COUNT] = {0U};
static uint32_t Led_BlinkPeriods[LED_ID_COUNT] = {0U};
static uint32_t Led_NextToggleTimes[LED_ID_COUNT] = {0U};

static uint8_t Led_IsTimeReached(uint32_t Now, uint32_t Target)
{
    return ((int32_t)(Now - Target) >= 0) ? 1U : 0U;
}

static Led_ResultType Led_CheckId(Led_IdType LedId)
{
    return (LedId < LED_ID_COUNT) ?
           LED_RESULT_OK : LED_RESULT_INVALID_PARAMETER;
}

static void Led_Write(Led_IdType LedId, uint8_t State)
{
    Led_States[LedId] = State;
    Dio_WriteChannel(
        Led_DioChannels[LedId],
        (State == LED_STATE_ON) ? STD_LOW : STD_HIGH);
}

void Led_Init(void)
{
    uint8_t index = 0U;

    for (index = 0U; index < LED_ID_COUNT; index++)
    {
        Led_BlinkEnabled[index] = 0U;
        Led_BlinkPeriods[index] = 0U;
        Led_NextToggleTimes[index] = 0U;
        Led_Write((Led_IdType)index, LED_STATE_OFF);
    }
}

Led_ResultType Led_On(Led_IdType LedId)
{
    Led_ResultType result = Led_CheckId(LedId);

    if (result == LED_RESULT_OK)
    {
        Led_BlinkEnabled[LedId] = 0U;
        Led_Write(LedId, LED_STATE_ON);
    }
    else
    {
        /* LED 标识无效。 */
    }

    return result;
}

Led_ResultType Led_Off(Led_IdType LedId)
{
    Led_ResultType result = Led_CheckId(LedId);

    if (result == LED_RESULT_OK)
    {
        Led_BlinkEnabled[LedId] = 0U;
        Led_Write(LedId, LED_STATE_OFF);
    }
    else
    {
        /* LED 标识无效。 */
    }

    return result;
}

Led_ResultType Led_Blink(Led_IdType LedId, uint32_t PeriodMs)
{
    Led_ResultType result = Led_CheckId(LedId);

    if (result == LED_RESULT_OK)
    {
        if (PeriodMs == 0U)
        {
            result = LED_RESULT_INVALID_PARAMETER;
        }
        else
        {
            Led_BlinkEnabled[LedId] = 1U;
            Led_BlinkPeriods[LedId] = PeriodMs;
            Led_NextToggleTimes[LedId] =
                SystemTime_GetTimeMs() + PeriodMs;
        }
    }
    else
    {
        /* LED 标识无效。 */
    }

    return result;
}

void Led_MainFunction(void)
{
    uint32_t now = SystemTime_GetTimeMs();
    uint8_t index = 0U;

    for (index = 0U; index < LED_ID_COUNT; index++)
    {
        if ((Led_BlinkEnabled[index] != 0U) &&
            (Led_IsTimeReached(now, Led_NextToggleTimes[index]) != 0U))
        {
            Led_Write(
                (Led_IdType)index,
                (Led_States[index] == LED_STATE_ON) ?
                LED_STATE_OFF : LED_STATE_ON);
            Led_NextToggleTimes[index] =
                now + Led_BlinkPeriods[index];
        }
        else
        {
            /* 当前 LED 未到切换时间。 */
        }
    }
}

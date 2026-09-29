#include "Key.h"

#include "Dio.h"

static const Dio_ChannelType Key_DioChannels[KEY_ID_COUNT] =
{
    DIO_CHANNEL_KEY0,
    DIO_CHANNEL_KEY1,
    DIO_CHANNEL_KEY2
};

static uint8_t Key_FilterCounters[KEY_ID_COUNT] = {0U};
static uint8_t Key_RawStates = 0U;
static uint8_t Key_StableStates = 0U;

/**
 * @brief 初始化按键模块。
 */
void Key_Init(void)
{
    uint8_t index = 0U;

    Key_RawStates = 0U;
    Key_StableStates = 0U;

    for (index = 0U; index < KEY_ID_COUNT; index++)
    {
        Key_FilterCounters[index] = 0U;
    }
}
/**
 * @brief 扫描按键状态。
 */
void Key_Scan(void)
{
    uint8_t index = 0U;
    uint8_t raw_state = 0U;

    for (index = 0U; index < KEY_ID_COUNT; index++)
    {
        raw_state = (Dio_ReadChannel(Key_DioChannels[index]) == STD_LOW) ?
                     KEY_STATE_PRESSED : KEY_STATE_RELEASED;

        if (raw_state != ((Key_RawStates >> index) & 0x01U))
        {
            Key_FilterCounters[index] = 0U;
            if (raw_state == KEY_STATE_PRESSED)
            {
                Key_RawStates |= KEY_STATE_MASK(index);
            }
            else
            {
                Key_RawStates &= (uint8_t)(~KEY_STATE_MASK(index));
            }
        }
        else if (Key_FilterCounters[index] < KEY_FILTER_COUNT)
        {
            Key_FilterCounters[index]++;
        }
        else
        {
            if (raw_state == KEY_STATE_PRESSED)
            {
                Key_StableStates |= KEY_STATE_MASK(index);
            }
            else
            {
                Key_StableStates &= (uint8_t)(~KEY_STATE_MASK(index));
            }
        }
    }
}
/**
 * @brief 获取按键状态。
 *
 * @param[in] KeyId 按键 ID。
 * @return Key_StateType 按键状态。
 */
Key_StateType Key_GetState(Key_IdType KeyId)
{
    if (KeyId < KEY_ID_COUNT)
    {
        return (Key_StateType)((Key_StableStates >> KeyId) & 0x01U);
    }
    else
    {
        return KEY_STATE_RELEASED;
    }
}
/**
 * @brief 获取按键状态掩码。
 *
 * @return uint8_t 按键状态掩码。
 */
uint8_t Key_GetStateMask(void)
{
    return Key_StableStates;
}

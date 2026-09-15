#include "Dio.h"
#include "Dio_Config.h"
/**
 * @brief DIO channel configuration structure.
 */
typedef struct
{
    GPIO_TypeDef *Port;
    uint16_t Pin;
} Dio_ChannelConfigType;
/**
 * @brief DIO channel configuration table.
 */
static const Dio_ChannelConfigType Dio_ChannelConfig[DIO_CONFIGURED_CHANNEL_COUNT] =
{
    [DIO_CHANNEL_LED0] =
    {
        DIO_CHANNEL_LED0_PORT,
        DIO_CHANNEL_LED0_PIN,
    },
    [DIO_CHANNEL_LED1] =
    {
        DIO_CHANNEL_LED1_PORT,
        DIO_CHANNEL_LED1_PIN,
    },
    [DIO_CHANNEL_PM1_CTRL_SD] =
    {
        DIO_CHANNEL_PM1_CTRL_SD_PORT,
        DIO_CHANNEL_PM1_CTRL_SD_PIN,
    },
    [DIO_CHANNEL_KEY0] =
    {
        DIO_CHANNEL_KEY0_PORT,
        DIO_CHANNEL_KEY0_PIN,
    },
    [DIO_CHANNEL_KEY1] =
    {
        DIO_CHANNEL_KEY1_PORT,
        DIO_CHANNEL_KEY1_PIN,
    },
    [DIO_CHANNEL_KEY2] =
    {
        DIO_CHANNEL_KEY2_PORT,
        DIO_CHANNEL_KEY2_PIN,
    }
};

_Static_assert((sizeof(Dio_ChannelConfig) / sizeof(Dio_ChannelConfig[0])) ==
                   DIO_CONFIGURED_CHANNEL_COUNT,
               "Dio channel configuration count mismatch");

/**
 * @brief Reads the logical level of a configured DIO channel.
 *
 * @param[in] ChannelId Logical channel identifier.
 *
 * @return STD_HIGH when the physical pin is set; otherwise STD_LOW. An
 *         invalid channel identifier returns STD_LOW without hardware access.
 */
Dio_LevelType Dio_ReadChannel(Dio_ChannelType ChannelId)
{
    if (ChannelId >= DIO_CONFIGURED_CHANNEL_COUNT)
    {
        return STD_LOW;
    }

    return (HAL_GPIO_ReadPin(Dio_ChannelConfig[ChannelId].Port,
                            Dio_ChannelConfig[ChannelId].Pin) == GPIO_PIN_SET)
               ? STD_HIGH
               : STD_LOW;
}

/**
 * @brief Writes a logical level to a configured DIO channel.
 *
 * @param[in] ChannelId Logical channel identifier.
 * @param[in] Level Requested logical level. STD_HIGH writes a high level;
 *                  every other value writes a low level.
 *
 * @note An invalid channel identifier is ignored without hardware access.
 */
void Dio_WriteChannel(Dio_ChannelType ChannelId, Dio_LevelType Level)
{
    if (ChannelId >= DIO_CONFIGURED_CHANNEL_COUNT)
    {
        return;
    }

    HAL_GPIO_WritePin(Dio_ChannelConfig[ChannelId].Port,
                      Dio_ChannelConfig[ChannelId].Pin,
                      (Level == STD_HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief Reverses the logical level of a configured DIO channel.
 *
 * @param[in] ChannelId Logical channel identifier.
 *
 * @return The logical level written after reversal. An invalid channel
 *         identifier returns STD_LOW.
 */
Dio_LevelType Dio_FlipChannel(Dio_ChannelType ChannelId)
{
    Dio_LevelType newLevel = STD_LOW;

    if (ChannelId < DIO_CONFIGURED_CHANNEL_COUNT)
    {
        newLevel = (Dio_ReadChannel(ChannelId) == STD_HIGH) ? STD_LOW : STD_HIGH;
        Dio_WriteChannel(ChannelId, newLevel);
    }

    return newLevel;
}

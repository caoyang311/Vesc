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
    [DIO_CHANNEL_LED] =
    {
        DIO_CHANNEL_LED_PORT,
        DIO_CHANNEL_LED_PIN,
    },
    [DIO_CHANNEL_DSP_P] =
    {
        DIO_CHANNEL_DSP_P_PORT,
        DIO_CHANNEL_DSP_P_PIN,
    },
    [DIO_CHANNEL_DSP_SS] =
    {
        DIO_CHANNEL_DSP_SS_PORT,
        DIO_CHANNEL_DSP_SS_PIN,
    },
    [DIO_CHANNEL_DSP_SDL] =
    {
        DIO_CHANNEL_DSP_SDL_PORT,
        DIO_CHANNEL_DSP_SDL_PIN,
    },
    [DIO_CHANNEL_DSP_SDH] =
    {
        DIO_CHANNEL_DSP_SDH_PORT,
        DIO_CHANNEL_DSP_SDH_PIN,
    },
    [DIO_CHANNEL_DSP_FDS] =
    {
        DIO_CHANNEL_DSP_FDS_PORT,
        DIO_CHANNEL_DSP_FDS_PIN,
    },
    [DIO_CHANNEL_DSP_XH] =
    {
        DIO_CHANNEL_DSP_XH_PORT,
        DIO_CHANNEL_DSP_XH_PIN,
    },
    [DIO_CHANNEL_DSP_PUSH] =
    {
        DIO_CHANNEL_DSP_PUSH_PORT,
        DIO_CHANNEL_DSP_PUSH_PIN,
    },
    [DIO_CHANNEL_DSP_SIF] =
    {
        DIO_CHANNEL_DSP_SIF_PORT,
        DIO_CHANNEL_DSP_SIF_PIN,
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

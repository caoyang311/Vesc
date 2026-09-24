#ifndef DIO_H
#define DIO_H

#include <stdint.h>

typedef enum
{
    DIO_CHANNEL_LED0 = 0,
    DIO_CHANNEL_LED1 = 1,
    DIO_CHANNEL_PM1_CTRL_SD = 2,
    DIO_CHANNEL_KEY0 = 3,
    DIO_CHANNEL_KEY1 = 4,
    DIO_CHANNEL_KEY2 = 5,
    DIO_CHANNEL_BEEP = 6,
    DIO_CHANNEL_MAX_485_EN = 7,
    DIO_CHANNEL_COUNT
} Dio_ChannelType;

typedef uint8_t Dio_LevelType;

#define STD_LOW  ((Dio_LevelType)0U)
#define STD_HIGH ((Dio_LevelType)1U)
/**
 * @brief Reads the logical level of a configured DIO channel.
 *
 * @param[in] ChannelId Logical channel identifier. The value shall be less
 *                      than DIO_CONFIGURED_CHANNEL_COUNT.
 *
 * @return STD_HIGH when the physical pin is high; otherwise STD_LOW.
 *
 * @note An invalid channel identifier returns STD_LOW.
 */
Dio_LevelType Dio_ReadChannel(Dio_ChannelType ChannelId);

/**
 * @brief Writes a logical level to a configured DIO channel.
 *
 * @param[in] ChannelId Logical channel identifier. The value shall be less
 *                      than DIO_CONFIGURED_CHANNEL_COUNT.
 * @param[in] Level Requested level. STD_HIGH writes high; all other values
 *                  write low.
 *
 * @note An invalid channel identifier causes no hardware access.
 */
void Dio_WriteChannel(Dio_ChannelType ChannelId, Dio_LevelType Level);

/**
 * @brief Reverses the logical level of a configured DIO channel.
 *
 * @param[in] ChannelId Logical channel identifier. The value shall be less
 *                      than DIO_CONFIGURED_CHANNEL_COUNT.
 *
 * @return The logical level written after reversal. An invalid channel
 *         identifier returns STD_LOW and causes no hardware access.
 */
Dio_LevelType Dio_FlipChannel(Dio_ChannelType ChannelId);

#endif /* DIO_H */

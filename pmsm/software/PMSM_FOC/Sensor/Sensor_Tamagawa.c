#include "Sensor_Tamagawa.h"
#include "Board_485.h"
#include <string.h>

static uint32_t Sensor_Tamagawa_RawCount; /**< 原始计数 */
static bool Sensor_Tamagawa_DataValid; /**< 数据有效标志 */
static bool Sensor_Tamagawa_RequestPending; /**< 请求待处理标志 */

/**
 * @brief 计算8位CRC校验和
 * @param data 输入数据指针
 * @param length 数据长度
 * @return uint8_t CRC�验和值
 */
static uint8_t Sensor_Tamagawa_Crc8(
    const uint8_t *data,
    uint16_t length)
{
    uint8_t crc = 0U;
    uint16_t index;
    uint8_t bit;

    for (index = 0U; index < length; index++)
    {
        crc ^= data[index];

        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x01U) != 0U)
            {
                crc = (uint8_t)((crc >> 1U) ^ 0x80U);
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}
/**
 * @brief 解析Tamagawa传感器响应
 * @param data 输入数据指针
 * @param length 数据长度
 * @return bool 是否成功解析
 */
static bool Sensor_Tamagawa_ParseResponse(
    const uint8_t *data,
    uint16_t length)
{
    uint32_t raw_count;

    if ((length != SENSOR_TAMAGAWA_RESPONSE_LENGTH) ||
        (data[0] != SENSOR_TAMAGAWA_READ_COMMAND) ||
        (Sensor_Tamagawa_Crc8(data, length - 1U) != data[length - 1U]))
    {
        return false;
    }

    raw_count = ((uint32_t)data[4] << 16U) |
                ((uint32_t)data[3] << 8U) |
                (uint32_t)data[2];

    Sensor_Tamagawa_RawCount =
        raw_count & SENSOR_TAMAGAWA_COUNT_MASK;
    Sensor_Tamagawa_DataValid = true;

    return true;
}
/**
 * @brief 初始化Tamagawa传感器
 */
void Sensor_Tamagawa_Init(void)
{
    Sensor_Tamagawa_RawCount = 0U;
    Sensor_Tamagawa_DataValid = false;
    Sensor_Tamagawa_RequestPending = false;
}
/**
 * @brief 处理角度数据
 */
void Sensor_Tamagawa_ProcessAngle(void)
{
    uint8_t response[SENSOR_TAMAGAWA_RESPONSE_LENGTH];
    uint16_t response_length = 0U;

    if (Rs485_FrameAvailable() != 0U)
    {
        if (Rs485_GetFrame(response, &response_length) == RS485_OK)
        {
            (void)Sensor_Tamagawa_ParseResponse(
                response,
                response_length);
        }

        Sensor_Tamagawa_RequestPending = false;
    }
}
/**
 * @brief 请求角度数据
 */
void Sensor_Tamagawa_RequestAngle(void)
{
    static const uint8_t request = SENSOR_TAMAGAWA_READ_COMMAND;

    if ((Sensor_Tamagawa_RequestPending == false) &&
        (Rs485_SendFrame(&request, 1U) == RS485_OK))
    {
        Sensor_Tamagawa_RequestPending = true;
    }
}
/**
 * @brief 获取原始计数
 * @return uint32_t 原始计数
 */
uint32_t Sensor_Tamagawa_GetRawCount(void)
{
    return Sensor_Tamagawa_RawCount;
}
/**
 * @brief 获取机械角度（弧度）
 * @return float 机械角度（弧度）
 */
float Sensor_Tamagawa_GetMechanicalAngleRad(void)
{
    return ((float)Sensor_Tamagawa_RawCount *
            SENSOR_TAMAGAWA_TWO_PI) /
           (float)SENSOR_TAMAGAWA_RESOLUTION;
}
/**
 * @brief 获取电气角度（弧度）
 * @param pole_pairs 极对数
 * @return float 电气角度（弧度）
 */
float Sensor_Tamagawa_GetElectricalAngleRad(uint8_t pole_pairs)
{
    float electrical_angle;

    electrical_angle = Sensor_Tamagawa_GetMechanicalAngleRad() *
                       (float)pole_pairs;

    while (electrical_angle >= SENSOR_TAMAGAWA_TWO_PI)
    {
        electrical_angle -= SENSOR_TAMAGAWA_TWO_PI;
    }

    return electrical_angle;
}
uint16_t Sensor_Tamagawa_GetElectricalPhase(uint8_t pole_pairs)
{
    uint32_t electrical_count;

    electrical_count = Sensor_Tamagawa_RawCount *
                       (uint32_t)pole_pairs;

    return (uint16_t)((electrical_count * 65536ULL) /
                      SENSOR_TAMAGAWA_RESOLUTION);
}
/**
 * @brief 检查数据是否有效
 * @return bool 是否有效
 */
bool Sensor_Tamagawa_IsDataValid(void)
{
    return Sensor_Tamagawa_DataValid;
}
/**
 * @brief 清除数据有效标志
 */
void Sensor_Tamagawa_ClearDataValid(void)
{
    Sensor_Tamagawa_DataValid = false;
}

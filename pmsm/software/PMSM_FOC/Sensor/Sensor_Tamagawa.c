#include "Sensor_Tamagawa.h"
#include "Board_485.h"
#include <stdint.h>

#define SENSOR_TAMAGAWA_PLL_BANDWIDTH_HZ  (50.0F) /**< PLL自然频率，单位Hz */
#define SENSOR_TAMAGAWA_PLL_DAMPING_RATIO  (1.0F) /**< PLL阻尼比，1.0为临界阻尼 */
#define SENSOR_TAMAGAWA_PLL_NATURAL_FREQ_RAD_S \
    (SENSOR_TAMAGAWA_TWO_PI * SENSOR_TAMAGAWA_PLL_BANDWIDTH_HZ)
#define SENSOR_TAMAGAWA_PLL_KP \
    (2.0F * SENSOR_TAMAGAWA_PLL_DAMPING_RATIO * \
     SENSOR_TAMAGAWA_PLL_NATURAL_FREQ_RAD_S) /**< PLL比例增益 */
#define SENSOR_TAMAGAWA_PLL_KI \
    (SENSOR_TAMAGAWA_PLL_NATURAL_FREQ_RAD_S * \
     SENSOR_TAMAGAWA_PLL_NATURAL_FREQ_RAD_S) /**< PLL积分增益 */

static volatile uint32_t ST_RawCount = 0U; /**< 原始角度数据 */
static volatile bool ST_NewData = false; /**< 新角度数据标志 */
static bool ST_RequestPending = false; /**< 角度数据请求待标志 */

static uint16_t ST_Electrical_Theta_Offset = 0U; /**< 电气零位偏移角度 */
static bool ST_Electrical_Theta_OffsetValid = false; /**< 电气零位校准标志 */


static float ST_Electrical_Theta = 0.0F; /**< 电角度 单位rad */
static float ST_Electrical_We = 0.0F; /**< 电角速度 单位rad/s */
static float ST_SpeedIntegral = 0.0F; /**< 速度积分项 */
static uint8_t ST_PreviousValid = false; /**< 上一次数据是否有效 */

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

    ST_RawCount = raw_count & SENSOR_TAMAGAWA_COUNT_MASK;
    ST_NewData = true;

    return true;
}

/**
 * @brief 获取新角度数据标志
 * @return bool 是否有新数据
 */
static bool Sensor_Tamagawa_ConsumeNewData(void)
{
    bool new_data = ST_NewData;

    ST_NewData = false;

    return new_data;
}
/**
 * @brief 包装角度弧度值到[-pi, pi]范围
 * @param phase_rad 输入角度弧度值
 * @return float 包装后的角度弧度值
 */
static float Sensor_Tamagawa_WrapPhaseRad(float phase_rad)
{
    while (phase_rad > (SENSOR_TAMAGAWA_TWO_PI * 0.5F))
    {
        phase_rad -= SENSOR_TAMAGAWA_TWO_PI;
    }

    while (phase_rad < -(SENSOR_TAMAGAWA_TWO_PI * 0.5F))
    {
        phase_rad += SENSOR_TAMAGAWA_TWO_PI;
    }

    return phase_rad;
}
/**
 * @brief 初始化Tamagawa传感器
 */
void Sensor_Tamagawa_Init(void)
{
    ST_RawCount = 0U;
    ST_Electrical_Theta_Offset = 0U;
    ST_Electrical_Theta_OffsetValid = false;
    ST_NewData = false;
    ST_RequestPending = false;
    ST_Electrical_Theta = 0.0F;
    ST_Electrical_We = 0.0F;
    ST_SpeedIntegral = 0.0F;
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

        ST_RequestPending = false;
    }
}
/**
 * @brief 请求角度数据
 */
void Sensor_Tamagawa_RequestAngle(void)
{
    static const uint8_t request = SENSOR_TAMAGAWA_READ_COMMAND;

    if ((ST_RequestPending == false) &&
        (Rs485_SendFrame(&request, 1U) == RS485_OK))
    {
        ST_RequestPending = true;
    }
}
/**
 * @brief 更新PLL电气角度和角速度。
 * @param sample_period_s FOC更新周期，单位s。
 * @param pole_pairs 电机极对数。
 */
void Sensor_Tamagawa_SampleSpeed(float sample_period_s,
                                 uint8_t pole_pairs)
{
    float current_theta = 0.0F;
    float theta_error = 0.0F;
    float proportional_speed = 0.0F;

    if ((sample_period_s > 0.0F) && (pole_pairs > 0U))
    {
        ST_Electrical_Theta += ST_Electrical_We * sample_period_s; /*当前电角度 = 上一次电角度 + 电角速度 * 时间周期（Δt） */
        ST_Electrical_Theta = Sensor_Tamagawa_WrapPhaseRad(ST_Electrical_Theta);/*限制电角度在[-pi, pi]范围内*/

        if (Sensor_Tamagawa_ConsumeNewData() == true)/*如果有新数据*/
        {
            current_theta =((float)Sensor_Tamagawa_GetElectricalPhaseWithOffset( pole_pairs) * SENSOR_TAMAGAWA_TWO_PI) / 65536.0F;/*获取当前电角度 单位：弧度*/
            theta_error = Sensor_Tamagawa_WrapPhaseRad( current_theta - ST_Electrical_Theta);/*计算电角度误差 单位：弧度*/

            if (ST_PreviousValid == false)/*如果是第一次更新*/
            {
                ST_Electrical_Theta = current_theta;
                ST_SpeedIntegral = 0.0F;
                ST_Electrical_We = 0.0F;
                ST_PreviousValid = true;
            }
            else
            {
                ST_SpeedIntegral +=SENSOR_TAMAGAWA_PLL_KI * theta_error *SENSOR_TAMAGAWA_UPDATE_PERIOD_S; /*积分项*/
                proportional_speed =SENSOR_TAMAGAWA_PLL_KP * theta_error; /*比例项*/
                ST_Electrical_We =ST_SpeedIntegral +proportional_speed; /*当前电角速度 = 积分项 + 比例项*/
            }
        }
        else
        {
        }
    }
    else
    {
        ST_Electrical_Theta = 0.0F;
        ST_SpeedIntegral = 0.0F;
        ST_Electrical_We = 0.0F;
        ST_PreviousValid = false;
        ST_NewData = false;
    }
}
/**
 * @brief 角度补偿
 * @param compensation_time_s 补偿时间，单位s
 * @return uint16_t 调整后的电气相位（65536分）
 */
uint16_t Sensor_Tamagawa_GetElectricalPhaseWithCompensation(
    float compensation_time_s)
{
    float compensated_phase_rad = 0.0F;

    compensated_phase_rad = ST_Electrical_Theta +(ST_Electrical_We * compensation_time_s); /*当前电角度 = 上一次电角度 + 电角速度 * 时间周期（Δt） */
    compensated_phase_rad = Sensor_Tamagawa_WrapPhaseRad(compensated_phase_rad);/*限制电角度在[-pi, pi]范围内*/
    if (compensated_phase_rad < 0.0F) /*角度限制在[0, 2pi]范围内*/
    {
        compensated_phase_rad += SENSOR_TAMAGAWA_TWO_PI;
    }
    else
    {
    }

    return (uint16_t)((compensated_phase_rad * 65536.0F) /SENSOR_TAMAGAWA_TWO_PI);/*将角度转换为65536分*/
}
/**
 * @brief 获取PLL电气角速度
 * @return float PLL电气角速度，单位rad/s
 */
float Sensor_Tamagawa_GetElectricalAngularSpeedRadS(void)
{
    return ST_Electrical_We;
}
/**
 * @brief 获取电机转速
 * @param pole_pairs 极对数
 * @param electrical_speed 电气角速度，单位rad/s
 * @return float 电机转速，单位RPM，正负表示旋转方向
 */
float Sensor_Tamagawa_GetMotorSpeedRpm(uint8_t pole_pairs,float electrical_speed)
{
    float motor_speed_rpm = 0.0F;

    if (pole_pairs > 0U)
    {
        motor_speed_rpm = (electrical_speed * 60.0F) /
                          (SENSOR_TAMAGAWA_TWO_PI * (float)pole_pairs);
    }
    else
    {
    }

    return motor_speed_rpm;
}
/**
 * @brief 获取电气相位（65536分）
 * @param pole_pairs 极对数
 * @return uint16_t 电气相位（65536分）
 */
uint16_t Sensor_Tamagawa_GetElectricalPhase(uint8_t pole_pairs)
{
    uint32_t electrical_count = 0U;
    uint16_t electrical_phase = 0U;

    electrical_count = ST_RawCount *(uint32_t)pole_pairs; 

    electrical_phase = (uint16_t)((electrical_count * 65536ULL) / SENSOR_TAMAGAWA_RESOLUTION);

    return electrical_phase;
}
/**
 * @brief 获取带零点补偿的电气相位。
 * @param pole_pairs 电机极对数。
 * @return uint16_t 补偿后的电气相位（65536分）。
 */
uint16_t Sensor_Tamagawa_GetElectricalPhaseWithOffset( uint8_t pole_pairs)
{
    uint16_t electrical_phase = 0U;
    uint16_t corrected_phase = 0U;

    electrical_phase = Sensor_Tamagawa_GetElectricalPhase(pole_pairs);
    corrected_phase = (uint16_t)(electrical_phase - ST_Electrical_Theta_Offset);
    return corrected_phase;
}
/**
 * @brief 设置电气相位偏移量
 * @param offset 偏移量（65536分）
 */
void Sensor_Tamagawa_SetElectricalPhaseOffset(uint16_t offset)
{
    ST_Electrical_Theta_Offset = offset;
    ST_Electrical_Theta_OffsetValid = true;
}
/**
 * @brief 检查电气相位偏移量是否有效
 * @return bool 是否有效
 */
bool Sensor_Tamagawa_IsElectricalOffsetValid(void)
{
    return ST_Electrical_Theta_OffsetValid;
}

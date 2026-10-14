#include "Sensor_Tamagawa.h"
#include "Board_485.h"
#include "FocFilter.h"
#include <string.h>
#include "Com.h"

#define SENSOR_TAMAGAWA_SPEED_FILTER_ALPHA (0.2F) /**< 速度滤波器系数 */
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

static volatile uint32_t Sensor_Tamagawa_RawCount = 0U; /**< 原始计数 */
static volatile bool Sensor_Tamagawa_NewData = false; /**< 差分新角度数据标志 */
static volatile bool Sensor_Tamagawa_PllNewData = false; /**< PLL新角度数据标志 */

static uint16_t Sensor_Tamagawa_ElectricalPhaseOffset = 0U; /**< 电气零位偏移 */
static bool Sensor_Tamagawa_ElectricalOffsetValid = false; /**< 电气零位有效 */
static bool Sensor_Tamagawa_RequestPending = false; /**< 请求待处理标志 */

static float Sensor_Tamagawa_MechanicalAngularSpeedRadS = 0.0F; /**< 机械角速度 */
static float Sensor_Tamagawa_ElectricalAngularSpeedRadS = 0.0F; /**< 电气角速度 */
static float Sensor_Tamagawa_MotorSpeedRpm = 0.0F; /**< 电机转速 */

static uint32_t Sensor_Tamagawa_PreviousCount = 0U; /**< 上次角度计数 */
static bool Sensor_Tamagawa_PreviousValid = false; /**< 上次角度有效标志 */

static LPF_Filter Sensor_Tamagawa_MechanicalSpeedFilter; /**< 机械角速度滤波器 */
static float Sensor_Tamagawa_PllElectricalPhaseRad = 0.0F; /**< PLL电气角度 */
static float Sensor_Tamagawa_PllElectricalSpeedRadS = 0.0F; /**< PLL电气角速度 */
static float Sensor_Tamagawa_PllSpeedIntegral = 0.0F; /**< PLL速度积分项 */
static bool Sensor_Tamagawa_PllValid = false; /**< PLL有效标志 */

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

    Sensor_Tamagawa_RawCount = raw_count & SENSOR_TAMAGAWA_COUNT_MASK;
    Sensor_Tamagawa_NewData = true;
    Sensor_Tamagawa_PllNewData = true;

    return true;
}

/**
 * @brief 获取新角度数据标志
 * @return bool 是否有新数据
 */
static bool Sensor_Tamagawa_ConsumeNewData(void)
{
    bool new_data = Sensor_Tamagawa_NewData;

    Sensor_Tamagawa_NewData = false;

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
 * @brief 获取PLL新角度数据标志
 * @return bool 是否有新数据
 */
static bool Sensor_Tamagawa_ConsumePllData(void)
{
    bool new_data = Sensor_Tamagawa_PllNewData;

    Sensor_Tamagawa_PllNewData = false;

    return new_data;
}
/**
 * @brief 初始化Tamagawa传感器
 */
void Sensor_Tamagawa_Init(void)
{
    Sensor_Tamagawa_RawCount = 0U;
    Sensor_Tamagawa_ElectricalPhaseOffset = 0U;
    Sensor_Tamagawa_ElectricalOffsetValid = false;
    Sensor_Tamagawa_NewData = false;
    Sensor_Tamagawa_PllNewData = false;
    Sensor_Tamagawa_RequestPending = false;
    Sensor_Tamagawa_MechanicalAngularSpeedRadS = 0.0F;
    Sensor_Tamagawa_ElectricalAngularSpeedRadS = 0.0F;
    Sensor_Tamagawa_MotorSpeedRpm = 0.0F;
    Sensor_Tamagawa_PllElectricalPhaseRad = 0.0F;
    Sensor_Tamagawa_PllElectricalSpeedRadS = 0.0F;
    Sensor_Tamagawa_PllSpeedIntegral = 0.0F;
    Sensor_Tamagawa_PllValid = false;
    FocLpf_Init(&Sensor_Tamagawa_MechanicalSpeedFilter,
                SENSOR_TAMAGAWA_SPEED_FILTER_ALPHA);
    Sensor_Tamagawa_PreviousCount = 0U;
    Sensor_Tamagawa_PreviousValid = false;
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
//    Rs485_SendFrame(&request, 1U);
}
/**
 * @brief 根据角度采样周期更新速度
 * @param sample_period_s 角度采样周期，单位为秒
 * @param pole_pairs 电机极对数
 */
void Sensor_Tamagawa_SampleSpeed(float sample_period_s, uint8_t pole_pairs)
{
    int32_t count_delta = 0;
    float mechanical_angle_delta_rad = 0.0F;

    if ((sample_period_s > 0.0F) && (pole_pairs > 0U))
    {
        if (Sensor_Tamagawa_ConsumeNewData() == false)
        {
            return;
        }
        if (Sensor_Tamagawa_PreviousValid == true)
        {
            count_delta = (int32_t)Sensor_Tamagawa_RawCount -
                          (int32_t)Sensor_Tamagawa_PreviousCount;

            if (count_delta > ((int32_t)SENSOR_TAMAGAWA_RESOLUTION / 2))
            {
                count_delta -= (int32_t)SENSOR_TAMAGAWA_RESOLUTION;
            }
            else if (count_delta < -((int32_t)SENSOR_TAMAGAWA_RESOLUTION / 2))
            {
                count_delta += (int32_t)SENSOR_TAMAGAWA_RESOLUTION;
            }
            else
            {
            }

            mechanical_angle_delta_rad =
                ((float)count_delta * SENSOR_TAMAGAWA_TWO_PI) /
                (float)SENSOR_TAMAGAWA_RESOLUTION;

            Sensor_Tamagawa_MechanicalAngularSpeedRadS =
                FocLpf_Update(&Sensor_Tamagawa_MechanicalSpeedFilter,
                              mechanical_angle_delta_rad /
                              sample_period_s);
            Sensor_Tamagawa_ElectricalAngularSpeedRadS =
                Sensor_Tamagawa_MechanicalAngularSpeedRadS *
                (float)pole_pairs;
            Sensor_Tamagawa_MotorSpeedRpm =
                (Sensor_Tamagawa_MechanicalAngularSpeedRadS * 60.0F) /
                SENSOR_TAMAGAWA_TWO_PI;
        }
        else
        {
            Sensor_Tamagawa_PreviousValid = true;
        }

        Sensor_Tamagawa_PreviousCount = Sensor_Tamagawa_RawCount;
    }
    else
    {
        Sensor_Tamagawa_MechanicalAngularSpeedRadS = 0.0F;
        Sensor_Tamagawa_ElectricalAngularSpeedRadS = 0.0F;
        Sensor_Tamagawa_MotorSpeedRpm = 0.0F;
        FocLpf_Init(&Sensor_Tamagawa_MechanicalSpeedFilter,
                    SENSOR_TAMAGAWA_SPEED_FILTER_ALPHA);
        Sensor_Tamagawa_PreviousValid = false;
    }
}
/**
 * @brief 更新PLL电气角度和角速度。
 * @param sample_period_s FOC更新周期，单位s。
 * @param pole_pairs 电机极对数。
 */
void Sensor_Tamagawa_UpdatePll(float sample_period_s,
                               uint8_t pole_pairs)
{
    float sensor_phase_rad = 0.0F;
    float phase_error_rad = 0.0F;
    float proportional_speed_rad_s = 0.0F;

    if ((sample_period_s > 0.0F) && (pole_pairs > 0U))
    {
        Sensor_Tamagawa_PllElectricalPhaseRad +=
            Sensor_Tamagawa_PllElectricalSpeedRadS * sample_period_s;
        Sensor_Tamagawa_PllElectricalPhaseRad =
            Sensor_Tamagawa_WrapPhaseRad(
                Sensor_Tamagawa_PllElectricalPhaseRad);

        if (Sensor_Tamagawa_ConsumePllData() == true)
        {
            sensor_phase_rad =
                ((float)Sensor_Tamagawa_GetElectricalPhaseWithOffset(
                    pole_pairs) * SENSOR_TAMAGAWA_TWO_PI) /
                65536.0F;
            phase_error_rad = Sensor_Tamagawa_WrapPhaseRad(
                sensor_phase_rad - Sensor_Tamagawa_PllElectricalPhaseRad);

            if (Sensor_Tamagawa_PllValid == false)
            {
                Sensor_Tamagawa_PllElectricalPhaseRad = sensor_phase_rad;
                Sensor_Tamagawa_PllSpeedIntegral = 0.0F;
                Sensor_Tamagawa_PllElectricalSpeedRadS = 0.0F;
                Sensor_Tamagawa_PllValid = true;
            }
            else
            {
                Sensor_Tamagawa_PllSpeedIntegral +=
                    SENSOR_TAMAGAWA_PLL_KI * phase_error_rad *
                    SENSOR_TAMAGAWA_UPDATE_PERIOD_S;
                proportional_speed_rad_s =
                    SENSOR_TAMAGAWA_PLL_KP * phase_error_rad;
                Sensor_Tamagawa_PllElectricalSpeedRadS =
                    Sensor_Tamagawa_PllSpeedIntegral +
                    proportional_speed_rad_s;
            }
        }
        else
        {
        }
    }
    else
    {
        Sensor_Tamagawa_PllElectricalPhaseRad = 0.0F;
        Sensor_Tamagawa_PllElectricalSpeedRadS = 0.0F;
        Sensor_Tamagawa_PllSpeedIntegral = 0.0F;
        Sensor_Tamagawa_PllValid = false;
        Sensor_Tamagawa_PllNewData = false;
    }
}
/**
 * @brief 清除新数据标志
 */
void Sensor_Tamagawa_ClearNewData(void)
{
    Sensor_Tamagawa_NewData = false;
    Sensor_Tamagawa_PllNewData = false;
}
/**
 * @brief 获取PLL电气相位（包含偏移量和补偿）
 * @param compensation_time_s �ens偿时间，单位s
 * @return uint16_t 调整后的电气相位（65536分）
 */
uint16_t Sensor_Tamagawa_GetPllElectricalPhaseWithOffsetAndLead(
    float compensation_time_s)
{
    float compensated_phase_rad = 0.0F;

    compensated_phase_rad = Sensor_Tamagawa_PllElectricalPhaseRad +
                            (Sensor_Tamagawa_PllElectricalSpeedRadS *
                             compensation_time_s);
    compensated_phase_rad = Sensor_Tamagawa_WrapPhaseRad(
        compensated_phase_rad);
    if (compensated_phase_rad < 0.0F)
    {
        compensated_phase_rad += SENSOR_TAMAGAWA_TWO_PI;
    }
    else
    {
    }

    return (uint16_t)((compensated_phase_rad * 65536.0F) /
                      SENSOR_TAMAGAWA_TWO_PI);
}
/**
 * @brief 获取PLL电气角速度
 * @return float PLL电气角速度，单位rad/s
 */
float Sensor_Tamagawa_GetPllElectricalAngularSpeedRadS(void)
{
    return Sensor_Tamagawa_PllElectricalSpeedRadS;
}
/**
 * @brief 获取PLL电机转速
 * @param pole_pairs 极对数
 * @return float PLL电机转速，单位RPM，正负表示旋转方向
 */
float Sensor_Tamagawa_GetPllMotorSpeedRpm(uint8_t pole_pairs)
{
    float motor_speed_rpm = 0.0F;

    if (pole_pairs > 0U)
    {
        motor_speed_rpm =
            (Sensor_Tamagawa_PllElectricalSpeedRadS * 60.0F) /
            (SENSOR_TAMAGAWA_TWO_PI * (float)pole_pairs);
    }
    else
    {
    }

    return motor_speed_rpm;
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
 * @brief 获取机械角速度
 * @return float 机械角速度，单位rad/s
 */
float Sensor_Tamagawa_GetMechanicalAngularSpeedRadS(void)
{
    return Sensor_Tamagawa_MechanicalAngularSpeedRadS;
}
/**
 * @brief 获取电气角速度
 * @return float 电气角速度，单位rad/s
 */
float Sensor_Tamagawa_GetElectricalAngularSpeedRadS(void)
{
    return Sensor_Tamagawa_ElectricalAngularSpeedRadS;
}
/**
 * @brief 获取电机转速
 * @return float 电机转速，单位RPM，正负表示旋转方向
 */
float Sensor_Tamagawa_GetMotorSpeedRpm(void)
{
    return Sensor_Tamagawa_MotorSpeedRpm;
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
/**
 * @brief 获取电气相位（65536分）
 * @param pole_pairs 极对数
 * @return uint16_t 电气相位（65536分）
 */
uint16_t Sensor_Tamagawa_GetElectricalPhase(uint8_t pole_pairs)
{
    uint32_t electrical_count = 0U;

    electrical_count = Sensor_Tamagawa_RawCount *
                       (uint32_t)pole_pairs;

    return (uint16_t)((electrical_count * 65536ULL) /
                      SENSOR_TAMAGAWA_RESOLUTION);
}
/**
 * @brief 获取电气相位（65536分）并应用偏移量
 * @param pole_pairs 极对数
 * @return uint16_t 电气相位（65536分）并应用偏移量
 */
uint16_t Sensor_Tamagawa_GetElectricalPhaseWithOffset(uint8_t pole_pairs)
{
    return Sensor_Tamagawa_GetElectricalPhaseWithOffsetAndLead(
        pole_pairs,
        0.0F);
}
/**
 * @brief 获取带速度前馈补偿的电气相位。
 * @param pole_pairs 电机极对数。
 * @param compensation_time_s 等效角度延迟，单位s。
 * @return uint16_t 补偿后的电气相位（65536分）。
 */
uint16_t Sensor_Tamagawa_GetElectricalPhaseWithOffsetAndLead(
    uint8_t pole_pairs,
    float compensation_time_s)
{
    uint16_t electrical_phase = 0U;
    uint16_t corrected_phase = 0U;
    int32_t phase_lead = 0;
    int32_t compensated_phase = 0;

    electrical_phase = Sensor_Tamagawa_GetElectricalPhase(pole_pairs);
    corrected_phase = (uint16_t)(electrical_phase -
                                 Sensor_Tamagawa_ElectricalPhaseOffset);
    if ((pole_pairs > 0U) && (compensation_time_s > 0.0F))
    {
        phase_lead = (int32_t)(
            (Sensor_Tamagawa_ElectricalAngularSpeedRadS *
             compensation_time_s * 65536.0F) /
            SENSOR_TAMAGAWA_TWO_PI);
        compensated_phase = (int32_t)corrected_phase + phase_lead;
        corrected_phase = (uint16_t)compensated_phase;
    }
    else
    {
    }
    return corrected_phase;
}
/**
 * @brief 设置电气相位偏移量
 * @param offset 偏移量（65536分）
 */
void Sensor_Tamagawa_SetElectricalPhaseOffset(uint16_t offset)
{
    Sensor_Tamagawa_ElectricalPhaseOffset = offset;
    Sensor_Tamagawa_ElectricalOffsetValid = true;
}
/**
 * @brief 检查电气相位偏移量是否有效
 * @return bool 是否有效
 */
bool Sensor_Tamagawa_IsElectricalOffsetValid(void)
{
    return Sensor_Tamagawa_ElectricalOffsetValid;
}

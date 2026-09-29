/**
 * @file EncoderService.c
 * @brief TIM2增量式编码器位置与速度服务实现。
 */

#include "EncoderService.h"

#include <limits.h>
#include <stddef.h>

#include "tim.h"
#include "FocMovingAverage.h"

#define ENCODER_SERVICE_PHASE_SCALE       (65536ULL)
#define ENCODER_SERVICE_SECONDS_PER_MINUTE (60.0F)
#define ENCODER_SERVICE_TWO_PI            (6.2831853072F)

static EncoderService_ConfigType EncoderService_Config = {0};
static volatile EncoderService_OutputType EncoderService_Output = {0};
static uint32_t EncoderService_PositionPreviousCount = 0U;
static uint32_t EncoderService_SpeedPreviousCount = 0U;
static FocMovingAverage_ContextType EncoderService_SpeedFilter = {0};
static bool EncoderService_IsInitialized = false;
static bool EncoderService_IsRunning = false;

/**
 * @brief 将32位无符号模运算差值转换为有符号计数差。
 *
 * @param[in] current_count 当前计数值。
 * @param[in] previous_count 上一次计数值。
 *
 * @return 最短方向的有符号计数差。
 * @note 单次采样的实际计数变化绝对值必须小于2^31。
 */
static int32_t EncoderService_GetDelta(uint32_t current_count,
                                       uint32_t previous_count);

/**
 * @brief 根据有符号增量更新一机械转内的归一化计数。
 *
 * @param[in] mechanical_count 当前机械计数，必须小于每转计数。
 * @param[in] delta_count 本次有符号计数增量。
 * @param[in] counts_per_revolution 每机械转计数，必须大于零。
 *
 * @return 更新后位于一机械转内的计数。
 */
static uint32_t EncoderService_UpdateMechanicalCount(
    uint32_t mechanical_count,
    int32_t delta_count,
    uint32_t counts_per_revolution);

/**
 * @brief 将32位无符号模运算差值转换为有符号计数差。
 *
 * 本实现显式处理负差值，避免依赖不可表示的无符号到有符号转换行为。
 *
 * @param[in] current_count 当前计数值。
 * @param[in] previous_count 上一次计数值。
 *
 * @return 最短方向的有符号计数差。
 */
static int32_t EncoderService_GetDelta(uint32_t current_count,
                                       uint32_t previous_count)
{
    uint32_t unsigned_delta = current_count - previous_count;
    uint32_t magnitude = 0U;
    int32_t signed_delta = 0;

    if (unsigned_delta <= (uint32_t)INT32_MAX)
    {
        signed_delta = (int32_t)unsigned_delta;
    }
    else
    {
        magnitude = (UINT32_MAX - unsigned_delta) + 1U;
        if (magnitude <= (uint32_t)INT32_MAX)
        {
            signed_delta = -(int32_t)magnitude;
        }
        else
        {
            signed_delta = INT32_MIN;
        }
    }

    return signed_delta;
}

/**
 * @brief 根据有符号增量更新一机械转内的归一化计数。
 *
 * @param[in] mechanical_count 当前机械计数，必须小于每转计数。
 * @param[in] delta_count 本次有符号计数增量。
 * @param[in] counts_per_revolution 每机械转计数，必须大于零。
 *
 * @return 更新后位于一机械转内的计数。
 */
static uint32_t EncoderService_UpdateMechanicalCount(
    uint32_t mechanical_count,
    int32_t delta_count,
    uint32_t counts_per_revolution)
{
    int64_t normalized_count = 0;

    normalized_count = (int64_t)mechanical_count + (int64_t)delta_count;
    normalized_count %= (int64_t)counts_per_revolution;

    if (normalized_count < 0)
    {
        normalized_count += (int64_t)counts_per_revolution;
    }
    else
    {
        /* 计数已经位于非负范围。 */
    }

    return (uint32_t)normalized_count;
}

/**
 * @brief 初始化编码器服务。
 *
 * @param[in] config 配置指针，不得为NULL；计数、极对数和周期必须大于零。
 *
 * @return 配置有效时返回true，否则返回false。
 */
bool EncoderService_Init(const EncoderService_ConfigType * config)
{
    bool is_valid = false;

    EncoderService_IsRunning = false;
    EncoderService_IsInitialized = false;
    EncoderService_Output.status = ENCODER_STATUS_UNINITIALIZED;

    if ((config != NULL) &&
        (config->counts_per_revolution > 0U) &&
        (config->pole_pairs > 0U) &&
        (config->speed_sample_period_s > 0.0F))
    {
        EncoderService_Config = *config;
        EncoderService_Output.raw_count = config->initial_count;
        EncoderService_Output.delta_count = 0;
        EncoderService_Output.mechanical_count = 0U;
        EncoderService_Output.mechanical_angle_phase = 0U;
        EncoderService_Output.electrical_angle_phase = 0U;
        EncoderService_Output.electrical_speed_rad_s = 0.0F;
        EncoderService_Output.mechanical_speed_rpm = 0.0F;
        EncoderService_Output.direction = ENCODER_DIRECTION_STOP;
        EncoderService_Output.status = ENCODER_STATUS_UNINITIALIZED;
        EncoderService_PositionPreviousCount = config->initial_count;
        EncoderService_SpeedPreviousCount = config->initial_count;
        FocMovingAverage_Init(&EncoderService_SpeedFilter);
        EncoderService_IsInitialized = true;
        is_valid = true;
    }
    else
    {
        EncoderService_Output.status = ENCODER_STATUS_CONFIG_ERROR;
    }

    return is_valid;
}

/**
 * @brief 启动TIM2的A/B双通道编码器接口。
 *
 * @return HAL启动成功时返回true，否则返回false。
 */
bool EncoderService_Start(void)
{
    bool is_started = false;

    if (EncoderService_IsInitialized == true)
    {
        __HAL_TIM_SET_COUNTER(&htim2, EncoderService_Config.initial_count);
        if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) == HAL_OK)
        {
            EncoderService_PositionPreviousCount =
                __HAL_TIM_GET_COUNTER(&htim2);
            EncoderService_SpeedPreviousCount =
                EncoderService_PositionPreviousCount;
            EncoderService_Output.raw_count =
                EncoderService_PositionPreviousCount;
            EncoderService_Output.mechanical_speed_rpm = 0.0F;
            EncoderService_Output.electrical_speed_rad_s = 0.0F;
            FocMovingAverage_Init(&EncoderService_SpeedFilter);
            EncoderService_Output.status = ENCODER_STATUS_RUNNING;
            EncoderService_IsRunning = true;
            is_started = true;
        }
        else
        {
            EncoderService_Output.status = ENCODER_STATUS_HARDWARE_ERROR;
        }
    }
    else
    {
        /* 未初始化时不访问TIM2。 */
    }

    return is_started;
}

/**
 * @brief 更新机械角度和电角度。
 *
 * 本函数供10 kHz快速控制中断调用，执行时间有界且不阻塞。
 */
void EncoderService_UpdatePosition(void)
{
    uint32_t current_count = 0U;
    uint32_t mechanical_count = 0U;
    uint64_t phase_numerator = 0ULL;
    uint64_t electrical_count = 0ULL;
    int32_t delta_count = 0;

    if (EncoderService_IsRunning == true)
    {
        current_count = __HAL_TIM_GET_COUNTER(&htim2);
        delta_count = EncoderService_GetDelta(
            current_count, EncoderService_PositionPreviousCount);
        mechanical_count = EncoderService_UpdateMechanicalCount(
            EncoderService_Output.mechanical_count,
            delta_count,
            EncoderService_Config.counts_per_revolution);

        phase_numerator = (uint64_t)mechanical_count *
                          ENCODER_SERVICE_PHASE_SCALE;
        electrical_count = ((uint64_t)mechanical_count *
                            (uint64_t)EncoderService_Config.pole_pairs) %
                           (uint64_t)EncoderService_Config.counts_per_revolution;

        EncoderService_Output.raw_count = current_count;
        EncoderService_Output.mechanical_count = mechanical_count;
        EncoderService_Output.mechanical_angle_phase =
            (uint16_t)(phase_numerator /
                       (uint64_t)EncoderService_Config.counts_per_revolution);
        EncoderService_Output.electrical_angle_phase =
            (uint16_t)((electrical_count * ENCODER_SERVICE_PHASE_SCALE) /
                       (uint64_t)EncoderService_Config.counts_per_revolution);
        EncoderService_PositionPreviousCount = current_count;
    }
    else
    {
        /* 服务未运行时不读取TIM2。 */
    }
}

/**
 * @brief 更新机械转速、电角速度和旋转方向。
 *
 * 本函数必须由固定5 ms协作任务调用一次。机械转速和电角速度使用最近
 * 10个速度采样的滑动平均结果，窗口填满前按当前有效采样数量计算。
 */
void EncoderService_UpdateSpeed(void)
{
    uint32_t current_count = 0U;
    int32_t delta_count = 0;
    float denominator = 0.0F;
    float mechanical_speed_rpm = 0.0F;
    float filtered_speed_rpm = 0.0F;

    if (EncoderService_IsRunning == true)
    {
        current_count = __HAL_TIM_GET_COUNTER(&htim2);
        delta_count = EncoderService_GetDelta(
            current_count, EncoderService_SpeedPreviousCount);
        denominator = (float)EncoderService_Config.counts_per_revolution *
                      EncoderService_Config.speed_sample_period_s;
        mechanical_speed_rpm =
            ((float)delta_count * ENCODER_SERVICE_SECONDS_PER_MINUTE) /
            denominator;

        EncoderService_Output.delta_count = delta_count;
        FocMovingAverage_Update(&EncoderService_SpeedFilter,
                                    mechanical_speed_rpm,
                                    &filtered_speed_rpm);
            EncoderService_Output.mechanical_speed_rpm = filtered_speed_rpm;
            EncoderService_Output.electrical_speed_rad_s =
                (filtered_speed_rpm *
                 (float)EncoderService_Config.pole_pairs *
                 ENCODER_SERVICE_TWO_PI) /
                ENCODER_SERVICE_SECONDS_PER_MINUTE;

        if (delta_count > 0)
        {
            EncoderService_Output.direction = ENCODER_DIRECTION_COUNTERCLOCKWISE;
        }
        else if (delta_count < 0)
        {
            EncoderService_Output.direction = ENCODER_DIRECTION_CLOCKWISE;
        }
        else
        {
            EncoderService_Output.direction = ENCODER_DIRECTION_STOP;
        }

        EncoderService_SpeedPreviousCount = current_count;
    }
    else
    {
        /* 服务未运行时不更新速度。 */
    }
}

/**
 * @brief 获取编码器计算结果的一致性快照。
 *
 * @param[out] output 输出结构体指针，不得为NULL。
 *
 * @return 服务正在运行且输出指针有效时返回true，否则返回false。
 */
bool EncoderService_GetOutput(EncoderService_OutputType * output)
{
    bool is_available = false;

    if ((output != NULL) && (EncoderService_IsRunning == true))
    {
        *output = EncoderService_Output;
        is_available = true;
    }
    else
    {
        /* 输出无效或服务未运行时不修改调用方数据。 */
    }

    return is_available;
}

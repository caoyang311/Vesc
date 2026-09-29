/**
 * @file FocPiController.c
 * @brief D轴和Q轴并联型PI电流控制器实现。
 */

#include "FocPiController.h"

#include <stddef.h>

#define FOC_PI_TWO_PI                 (6.2831853072F)
#define FOC_PI_ONE_OVER_SQRT_THREE    (0.5773502692F)
#define FOC_PI_STAR_PHASE_FACTOR      (0.5F)
#define FOC_PI_LINE_RESISTANCE_OHM    (1.02F)
#define FOC_PI_LINE_INDUCTANCE_H      (0.00059F)
#define FOC_PI_RATED_VOLTAGE_V        (24.0F)
#define FOC_PI_LOOP_BANDWIDTH_HZ      (1000.0F)
#define FOC_PI_LOOP_FREQUENCY_HZ      (10000.0F)
#define FOC_PI_LOOP_OMEGA_RAD_S       (FOC_PI_TWO_PI * FOC_PI_LOOP_BANDWIDTH_HZ)
#define FOC_PI_PROPORTIONAL_GAIN      (FOC_PI_LINE_INDUCTANCE_H * FOC_PI_STAR_PHASE_FACTOR * FOC_PI_LOOP_OMEGA_RAD_S)
#define FOC_PI_INTEGRAL_GAIN          (FOC_PI_LINE_RESISTANCE_OHM * FOC_PI_STAR_PHASE_FACTOR * FOC_PI_LOOP_OMEGA_RAD_S)
#define FOC_PI_SAMPLE_PERIOD_S        (1.0F / FOC_PI_LOOP_FREQUENCY_HZ)
#define FOC_PI_INTEGRAL_LIMIT_V       (FOC_PI_RATED_VOLTAGE_V * FOC_PI_ONE_OVER_SQRT_THREE)

/** @brief 单轴并联型PI控制器内部配置。 */
typedef struct
{
    float proportional_gain;
    float integral_gain;
    float sample_period_s;
    float integral_limit_v;
} FocPiController_ConfigType;

static FocPiController_ConfigType FocPiController_Config = {0.0F, 0.0F, 0.0F, 0.0F};
static float FocPiController_IntegralD = 0.0F;
static float FocPiController_IntegralQ = 0.0F;
static bool FocPiController_IsInitialized = false;

/**
 * @brief 将输入限制在对称边界内。
 *
 * @param[in] value 待限制数值。
 * @param[in] absolute_limit 正的绝对限幅。
 *
 * @return 限制后的数值。
 */
static float FocPiController_Clamp(float value, float absolute_limit);

/**
 * @brief 执行一次单轴并联型PI控制。
 *
 * @param[in] feedback_current_a 反馈电流，单位A。
 * @param[in] reference_current_a 参考电流，单位A。
 * @param[in] dc_bus_voltage_v 母线电压，单位V，必须大于0。
 * @param[in,out] integral_v 该轴积分状态，单位V，不得为NULL。
 * @param[out] output_voltage_v 该轴输出电压，单位V，不得为NULL。
 *
 * @return 参数和控制器状态有效时返回true，否则返回false。
 */
static void FocPiController_UpdateAxis(float feedback_current_a,
                                       float reference_current_a,
                                       float dc_bus_voltage_v,
                                       float * integral_v,
                                       float * output_voltage_v);

/**
 * @brief 将输入限制在对称边界内。
 *
 * @param[in] value 待限制数值。
 * @param[in] absolute_limit 正的绝对限幅。
 *
 * @return 限制后的数值。
 */
static float FocPiController_Clamp(float value, float absolute_limit)
{
    float limited_value = value;

    if (value > absolute_limit)
    {
        limited_value = absolute_limit;
    }
    else if (value < -absolute_limit)
    {
        limited_value = -absolute_limit;
    }
    else
    {
        /* 输入位于允许范围内。 */
    }

    return limited_value;
}

/**
 * @brief 执行一次单轴并联型PI控制。
 *
 * @param[in] feedback_current_a 反馈电流，单位A。
 * @param[in] reference_current_a 参考电流，单位A。
 * @param[in] dc_bus_voltage_v 母线电压，单位V，必须大于0。
 * @param[in,out] integral_v 该轴积分状态，单位V，不得为NULL。
 * @param[out] output_voltage_v 该轴输出电压，单位V，不得为NULL。
 *
 * @return 参数和控制器状态有效时返回true，否则返回false。
 */
static void FocPiController_UpdateAxis(float feedback_current_a,
                                       float reference_current_a,
                                       float dc_bus_voltage_v,
                                       float * integral_v,
                                       float * output_voltage_v)
{
    float error_a = 0.0F;
    float voltage_limit_v = 0.0F;
    float voltage_v = 0.0F;


    error_a = reference_current_a - feedback_current_a;
    *integral_v += FocPiController_Config.integral_gain *
                    error_a * FocPiController_Config.sample_period_s;
    *integral_v = FocPiController_Clamp(*integral_v, FocPiController_Config.integral_limit_v);/* 限制积分状态 */

    voltage_v = (FocPiController_Config.proportional_gain * error_a) + *integral_v;/* 计算输出电压 */
    voltage_limit_v = dc_bus_voltage_v * FOC_PI_ONE_OVER_SQRT_THREE;/* 限制输出电压 */
    *output_voltage_v = FocPiController_Clamp(voltage_v, voltage_limit_v);/* 限制输出电压 */
}

/**
 * @brief 初始化D轴和Q轴并联型PI控制器。
 *
 * 根据模块内部的电机参数和电流环参数计算Kp、Ki、采样周期及积分限幅，
 * 并清零D轴和Q轴积分状态。
 */
void FocPiController_Init(void)
{
    FocPiController_Config.proportional_gain = FOC_PI_PROPORTIONAL_GAIN;
    FocPiController_Config.integral_gain = FOC_PI_INTEGRAL_GAIN;
    FocPiController_Config.sample_period_s = FOC_PI_SAMPLE_PERIOD_S;
    FocPiController_Config.integral_limit_v = FOC_PI_INTEGRAL_LIMIT_V;
    FocPiController_IntegralD = 0.0F;
    FocPiController_IntegralQ = 0.0F;
    FocPiController_IsInitialized = true;
}

/**
 * @brief 清零D轴和Q轴PI控制器积分状态。
 */
void FocPiController_Reset(void)
{
    FocPiController_IntegralD = 0.0F;
    FocPiController_IntegralQ = 0.0F;
}

/**
 * @brief 执行一次D轴并联型PI电流控制。
 *
 * @param[in] feedback_current_a D轴反馈电流，单位A。
 * @param[in] reference_current_a D轴参考电流，单位A。
 * @param[in] dc_bus_voltage_v 母线电压，单位V，必须大于0。
 * @param[out] output_voltage_v D轴输出电压，单位V，不得为NULL。
 *
 * @return 控制器和输入有效时返回true，否则返回false。
 */
void FocPiController_UpdateD(float feedback_current_a,
                             float reference_current_a,
                             float dc_bus_voltage_v,
                             float * output_voltage_v)
{
    FocPiController_UpdateAxis(feedback_current_a,
                                      reference_current_a,
                                      dc_bus_voltage_v,
                                      &FocPiController_IntegralD,
                                      output_voltage_v);
}

/**
 * @brief 执行一次Q轴并联型PI电流控制。
 *
 * @param[in] feedback_current_a Q轴反馈电流，单位A。
 * @param[in] reference_current_a Q轴参考电流，单位A。
 * @param[in] dc_bus_voltage_v 母线电压，单位V，必须大于0。
 * @param[out] output_voltage_v Q轴输出电压，单位V，不得为NULL。
 *
 * @return 控制器和输入有效时返回true，否则返回false。
 */
void FocPiController_UpdateQ(float feedback_current_a,
                             float reference_current_a,
                             float dc_bus_voltage_v,
                             float * output_voltage_v)
{
    FocPiController_UpdateAxis(feedback_current_a,
                                      reference_current_a,
                                      dc_bus_voltage_v,
                                      &FocPiController_IntegralQ,
                                      output_voltage_v);
}

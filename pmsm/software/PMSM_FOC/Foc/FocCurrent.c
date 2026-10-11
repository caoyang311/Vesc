#include "FocCurrent.h"
#include "Board_Adc.h"
#include <stddef.h>
#define FOC_CURRENT_ADC_REFERENCE_VOLTAGE_V (3.3F)  /**< ADC参考电压，单位：V */
#define FOC_CURRENT_ADC_MAX_COUNT           (4095.0F) /**< ADC最大计数 */
#define FOC_CURRENT_SHUNT_RESISTANCE_OHM    (0.020F) /**< 电流采样电阻，单位：Ω */
#define FOC_CURRENT_AMPLIFIER_GAIN          (6.0F)   /**< 电流放大器增益 */
#define FOC_CURRENT_BIAS_SAMPLE_COUNT       (64U)    /**< 偏置采样点数 */

static Foc_PhaseCurrentType FocCurrent_Current = {0};
static Foc_PhCurrentBiasValueType FocCurrent_BiasValue = {0};
static Foc_PhCurrentBiasVoltageType FocCurrent_BiasVoltage = {0};   
/**
 * @brief 将ADC值转换为电流值，单位为A。
 *
 * @param adc_value ADC值，范围0-4095。
 * @param bias_value 偏置值，范围0-4095。
 * @return float 电流值，单位：A。
 */
static float FocCurrent_AdcToCurrent(Adc_ValueType adc_value,
                                     uint16_t bias_value)
{
    float adc_voltage = 0.0F;
    float bias_voltage = 0.0F;

    adc_voltage = ((float)adc_value * FOC_CURRENT_ADC_REFERENCE_VOLTAGE_V) /
                  FOC_CURRENT_ADC_MAX_COUNT;
    bias_voltage = ((float)bias_value * FOC_CURRENT_ADC_REFERENCE_VOLTAGE_V) /
                   FOC_CURRENT_ADC_MAX_COUNT;

    return (adc_voltage - bias_voltage) /
           (FOC_CURRENT_SHUNT_RESISTANCE_OHM * FOC_CURRENT_AMPLIFIER_GAIN);
}
/**
 * @brief 初始化相电流采样模块。
 */
void FocCurrent_Init(void)
{
    FocCurrent_Current = (Foc_PhaseCurrentType){0};
    FocCurrent_BiasValue = (Foc_PhCurrentBiasValueType){0};
}
/**
 * @brief 采样零电流偏置电压并更新偏置。
 *
 * @note 调用前必须确保电机处于零电流状态。
 *       函数连续采集64组U/V/W相ADC数据，分别求和后取平均值。
 */
void FocCurrent_CalibrateBiasVoltage(void)
{
    Adc_ValueType adc_u = 0U;
    Adc_ValueType adc_v = 0U;
    Adc_ValueType adc_w = 0U;
    static uint32_t sum_u = 0U;
    static uint32_t sum_v = 0U;
    static uint32_t sum_w = 0U;
    static uint8_t sample_index = 0U;

    
    if(sample_index < FOC_CURRENT_BIAS_SAMPLE_COUNT)
    {
        if ((Adc_ReadChannel(ADC_CH_ID_PHASE_U_CURRENT, &adc_u) == ADC_RESULT_OK) &&
            (Adc_ReadChannel(ADC_CH_ID_PHASE_V_CURRENT, &adc_v) == ADC_RESULT_OK) &&
            (Adc_ReadChannel(ADC_CH_ID_PHASE_W_CURRENT, &adc_w) == ADC_RESULT_OK))
        {
            sum_u += (uint32_t)adc_u;
            sum_v += (uint32_t)adc_v;
            sum_w += (uint32_t)adc_w;
            sample_index++;
        }
    }
    else
    {
        FocCurrent_BiasValue.phase_u_bias_value =
            (uint16_t)(sum_u / FOC_CURRENT_BIAS_SAMPLE_COUNT);
        FocCurrent_BiasValue.phase_v_bias_value =
            (uint16_t)(sum_v / FOC_CURRENT_BIAS_SAMPLE_COUNT);
        FocCurrent_BiasValue.phase_w_bias_value =
            (uint16_t)(sum_w / FOC_CURRENT_BIAS_SAMPLE_COUNT);
        sum_u = 0U;
        sum_v = 0U;
        sum_w = 0U;
        sample_index = 0U;
    }
}

/**
 * @brief 根据当前SVPWM扇区采样两相，并计算第三相电流。
 *
 * 低端导通时间最短的相不参与本周期采样，另外两相直接读取；第三相
 * 根据基尔霍夫电流定律计算：Iu + Iv + Iw = 0。
 *
 * @param[out] phase_current 三相电流输出，指针不得为NULL。
 */
void FocCurrent_SamplePhaseCurrent(Foc_PhaseCurrentType * phase_current)
{
    Adc_ValueType adc_u = 0U;
    Adc_ValueType adc_v = 0U;
    Adc_ValueType adc_w = 0U;
    Foc_PhaseCurrentType result = {0};
    uint8_t sector = 0U;

    sector = FocAlgorithm_GetSector();

    if ((sector == 1U) || (sector == 6U))
    {
        /* Sector 1/6：U相低端导通时间最短，采样V、W相。 */
        if ((Adc_ReadChannel(ADC_CH_ID_PHASE_V_CURRENT, &adc_v) == ADC_RESULT_OK) &&
            (Adc_ReadChannel(ADC_CH_ID_PHASE_W_CURRENT, &adc_w) == ADC_RESULT_OK))
        {
            result.phase_v = -FocCurrent_AdcToCurrent(
                adc_v, FocCurrent_BiasValue.phase_v_bias_value);
            result.phase_w = -FocCurrent_AdcToCurrent(
                adc_w, FocCurrent_BiasValue.phase_w_bias_value);
            result.phase_u = -(result.phase_v + result.phase_w);
            FocCurrent_Current = result;
            *phase_current = result;
        }
        else
        {
        }
    }
    else if ((sector == 2U) || (sector == 3U))
    {
        /* Sector 2/3：V相低端导通时间最短，采样U、W相。 */
        if ((Adc_ReadChannel(ADC_CH_ID_PHASE_U_CURRENT, &adc_u) == ADC_RESULT_OK) &&
            (Adc_ReadChannel(ADC_CH_ID_PHASE_W_CURRENT, &adc_w) == ADC_RESULT_OK))
        {
            result.phase_u = -FocCurrent_AdcToCurrent(
                adc_u, FocCurrent_BiasValue.phase_u_bias_value);
            result.phase_w = -FocCurrent_AdcToCurrent(
                adc_w, FocCurrent_BiasValue.phase_w_bias_value);
            result.phase_v = -(result.phase_u + result.phase_w);
            FocCurrent_Current = result;
            *phase_current = result;
        }
        else
        {
        }
    }
    else if ((sector == 4U) || (sector == 5U))
    {
        /* Sector 4/5：W相低端导通时间最短，采样U、V相。 */
        if ((Adc_ReadChannel(ADC_CH_ID_PHASE_U_CURRENT, &adc_u) == ADC_RESULT_OK) &&
            (Adc_ReadChannel(ADC_CH_ID_PHASE_V_CURRENT, &adc_v) == ADC_RESULT_OK))
        {
            result.phase_u = -FocCurrent_AdcToCurrent(
                adc_u, FocCurrent_BiasValue.phase_u_bias_value);
            result.phase_v = -FocCurrent_AdcToCurrent(
                adc_v, FocCurrent_BiasValue.phase_v_bias_value);
            result.phase_w = -(result.phase_u + result.phase_v);
            FocCurrent_Current = result;
            *phase_current = result;
        }
        else
        {
        }
    }
    else
    {
        /* 扇区无效时保持上一组有效电流。 */
    }
}
/**
 * @brief 获取当前三相偏置电压，单位为V。
 *
 * @param[out] bias_voltage 三相偏置电压输出，指针不得为NULL。
 */
void FocCurrent_GetBiasVoltage(Foc_PhCurrentBiasVoltageType * bias_voltage)
{
    if (bias_voltage != NULL)
    {
        FocCurrent_BiasVoltage.phase_u_bias_voltage = FocCurrent_BiasValue.phase_u_bias_value * FOC_CURRENT_ADC_REFERENCE_VOLTAGE_V / FOC_CURRENT_ADC_MAX_COUNT;
        FocCurrent_BiasVoltage.phase_v_bias_voltage = FocCurrent_BiasValue.phase_v_bias_value * FOC_CURRENT_ADC_REFERENCE_VOLTAGE_V / FOC_CURRENT_ADC_MAX_COUNT;
        FocCurrent_BiasVoltage.phase_w_bias_voltage = FocCurrent_BiasValue.phase_w_bias_value * FOC_CURRENT_ADC_REFERENCE_VOLTAGE_V / FOC_CURRENT_ADC_MAX_COUNT;
    }
    *bias_voltage = FocCurrent_BiasVoltage;
}

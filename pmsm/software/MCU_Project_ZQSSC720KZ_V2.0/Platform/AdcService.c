#include "AdcService.h"
#include "Pwm.h"
#include "adc.h"
#include "tim.h"
#include <stddef.h>

#define ADC_SERVICE_ADC1_DMA_LENGTH (UINT32_C(4))

_Alignas(uint32_t) static volatile uint16_t
    AdcService_Adc1DmaBuffer[ADC_SERVICE_ADC1_DMA_LENGTH];

/**
 * @brief 停止本次启动流程涉及的ADC和定时器资源。
 *
 * 该函数用于启动失败后的回滚，按触发源到ADC的顺序尝试停止全部资源。
 * 所有底层返回值均有意丢弃，因为调用方已经确定主启动流程失败。
 */
static void AdcService_RollbackStart(void);

/**
 * @brief 校准指定ADC模块。
 *
 * @param[in] unit ADC模块标识。
 *
 * @return ADC单端校准成功时返回true，否则返回false。
 *
 * @pre 对应ADC底层初始化已经完成，且ADC转换尚未启动。
 * @note 本接口可能等待硬件校准完成，只允许在系统初始化上下文调用。
 */
bool AdcService_Calibrate(AdcService_UnitType unit)
{
    bool is_calibrated = false;

    switch (unit)
    {
        case ADC_SERVICE_UNIT_1:
            is_calibrated =
                (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) == HAL_OK);
            break;

        case ADC_SERVICE_UNIT_2:
            is_calibrated =
                (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) == HAL_OK);
            break;

        case ADC_SERVICE_UNIT_3:
            is_calibrated =
                (HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED) == HAL_OK);
            break;

        default:
            is_calibrated = false;
            break;
    }

    return is_calibrated;
}

/**
 * @brief 启动全部ADC采集链路及其定时器触发源。
 *
 * 按ADC1循环DMA、ADC2注入组、ADC3注入组、TIM4和TIM1 CH4的顺序
 * 启动。任一步骤失败后执行有界回滚，不保存运行时状态。
 *
 * @return 全部采集链路启动成功时返回true，否则返回false。
 *
 * @pre 三个ADC均已完成底层初始化和校准。
 * @pre TIM1、TIM4已完成底层初始化且CCR4已设置为有效值。
 */
bool AdcService_Start(void)
{
    bool is_started = false;

    /* HAL接口固定要求uint32_t指针；DMA实际按半字写入此对齐缓冲区。
     * 此处移除volatile限定符需要作为HAL边界的MISRA C:2012 Rule 11.8
     * 局部偏差进行评审。 */
    if (HAL_ADC_Start_DMA(&hadc1,
                          (uint32_t *)AdcService_Adc1DmaBuffer,
                          ADC_SERVICE_ADC1_DMA_LENGTH) == HAL_OK)
    {
        if (HAL_ADCEx_InjectedStart(&hadc2) == HAL_OK)
        {
            if (HAL_ADCEx_InjectedStart(&hadc3) == HAL_OK)
            {
                if (HAL_TIM_Base_Start(&htim4) == HAL_OK)
                {
                    Pwm_SetAdcTriggerCompare(3600-5);
                    Pwm_StartAdcTrigger();
                    is_started = true;
                }
            }
        }
    }

    if (is_started == false)
    {
        AdcService_RollbackStart();
    }

    return is_started;
}

/**
 * @brief 读取指定ADC模块的指定逻辑通道原始值。
 *
 * @param[in] unit ADC模块标识。
 * @param[in] channel ADC逻辑通道标识。
 * @param[out] value 指向采样值输出变量的非NULL指针。
 *
 * @return 参数和通道归属有效并完成读取时返回true，否则返回false。
 *
 * @pre ADC采集链路已经启动。
 * @pre 读取ADC2或ADC3时，本控制周期注入转换已经完成。
 */
void AdcService_ReadChannel(AdcService_UnitType unit,
                            AdcService_ChannelType channel,
                            AdcService_ValueType *value)
{
    AdcService_ValueType sampled_value = 0U;

    if (((uint32_t)unit < (uint32_t)ADC_SERVICE_UNIT_COUNT) &&
        ((uint32_t)channel < (uint32_t)ADC_SERVICE_CHANNEL_COUNT))
    {
        switch (channel)
        {
            case ADC_SERVICE_CHANNEL_ADC1_IN6:
                if (unit == ADC_SERVICE_UNIT_1)
                {
                    sampled_value = AdcService_Adc1DmaBuffer[0];
                }
                break;

            case ADC_SERVICE_CHANNEL_ADC1_IN7:
                if (unit == ADC_SERVICE_UNIT_1)
                {
                    sampled_value = AdcService_Adc1DmaBuffer[1];
                }
                break;

            case ADC_SERVICE_CHANNEL_ADC1_IN9:
                if (unit == ADC_SERVICE_UNIT_1)
                {
                    sampled_value = AdcService_Adc1DmaBuffer[2];
                }
                break; 

            case ADC_SERVICE_CHANNEL_ADC1_IN14:
                if (unit == ADC_SERVICE_UNIT_1)
                {
                    sampled_value = AdcService_Adc1DmaBuffer[3];
                }
                break;                               

            case ADC_SERVICE_CHANNEL_ADC2_IN12:
                if (unit == ADC_SERVICE_UNIT_2)
                {
                    sampled_value = (AdcService_ValueType)
                        HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
                }
                break;

            case ADC_SERVICE_CHANNEL_ADC2_IN8:
                if (unit == ADC_SERVICE_UNIT_2)
                {
                    sampled_value = (AdcService_ValueType)
                        HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
                }
                break;

            case ADC_SERVICE_CHANNEL_ADC2_IN5:
                if (unit == ADC_SERVICE_UNIT_2)
                {
                    sampled_value = (AdcService_ValueType)
                        HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3);
                }
                break;

            case ADC_SERVICE_CHANNEL_ADC3_IN1:
                if (unit == ADC_SERVICE_UNIT_3)
                {
                    sampled_value = (AdcService_ValueType)
                        HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1);
                }
                break;

            case ADC_SERVICE_CHANNEL_ADC3_IN12:
                if (unit == ADC_SERVICE_UNIT_3)
                {
                    sampled_value = (AdcService_ValueType)
                        HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_2);
                }
                break;

            default:
                break;
        }
    }

    *value = sampled_value;

}

/**
 * @brief 停止本次启动流程涉及的ADC和定时器资源。
 *
 * 该函数仅用于启动失败回滚，不保存或检查运行状态。重复停止操作的
 * HAL返回值被显式丢弃，确保每个资源都得到一次停止尝试。
 */
static void AdcService_RollbackStart(void)
{
    (void)Pwm_StopAdcTrigger();
    (void)HAL_TIM_Base_Stop(&htim4);
    (void)HAL_ADCEx_InjectedStop(&hadc3);
    (void)HAL_ADCEx_InjectedStop(&hadc2);
    (void)HAL_ADC_Stop_DMA(&hadc1);
}

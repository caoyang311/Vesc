#ifndef FOC_CURRENT_H
#define FOC_CURRENT_H

#include <stdbool.h>
#include <stdint.h>
#include "FocAlgorithm.h"

typedef struct
{
    uint16_t phase_u_bias_value;
    uint16_t phase_v_bias_value;
    uint16_t phase_w_bias_value;
} Foc_PhCurrentBiasValueType;

typedef struct
{
    float phase_u_bias_voltage;
    float phase_v_bias_voltage;
    float phase_w_bias_voltage;
} Foc_PhCurrentBiasVoltageType;

/**
 * @brief 初始化相电流采样模块。
 */
void FocCurrent_Init(void);

/**
 * @brief 采样零电流偏置电压并更新偏置。
 *
 * @note 调用前必须确保电机处于零电流状态。
 *       函数连续采集64组U/V/W相ADC数据，分别求和后取平均值。
 */
void FocCurrent_CalibrateBiasVoltage(void);

/**
 * @brief 更新三相实时电流，单位为A。
 *
 * @param[out] phase_current 三相电流输出，指针不得为NULL。
 */
void FocCurrent_SamplePhaseCurrent(Foc_PhaseCurrentType * phase_current);

/**
 * @brief 获取当前三相偏置电压，单位为V。
 *
 * @param[out] bias_voltage 三相偏置电压输出，指针不得为NULL。
 */
void FocCurrent_GetBiasVoltage(Foc_PhCurrentBiasVoltageType * bias_voltage);

#endif
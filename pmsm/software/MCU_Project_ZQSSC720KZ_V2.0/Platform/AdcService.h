#ifndef ADC_SERVICE_H
#define ADC_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    ADC_SERVICE_UNIT_1 = 0,
    ADC_SERVICE_UNIT_2,
    ADC_SERVICE_UNIT_3,
    ADC_SERVICE_UNIT_COUNT
} AdcService_UnitType;

typedef enum
{
    ADC_SERVICE_CHANNEL_ADC1_IN6 = 0,
    ADC_SERVICE_CHANNEL_ADC1_IN7,
    ADC_SERVICE_CHANNEL_ADC1_IN9,
    ADC_SERVICE_CHANNEL_ADC1_IN14,
    ADC_SERVICE_CHANNEL_ADC2_IN12,
    ADC_SERVICE_CHANNEL_ADC2_IN8,
    ADC_SERVICE_CHANNEL_ADC2_IN5,
    ADC_SERVICE_CHANNEL_ADC3_IN1,
    ADC_SERVICE_CHANNEL_ADC3_IN12,
    ADC_SERVICE_CHANNEL_COUNT
} AdcService_ChannelType;

typedef uint16_t AdcService_ValueType;

/**
 * @brief 校准指定ADC模块。
 *
 * @param[in] unit ADC模块标识，取值范围为ADC_SERVICE_UNIT_1至
 *                 ADC_SERVICE_UNIT_3。
 *
 * @return ADC单端校准成功时返回true，否则返回false。
 *
 * @pre 对应ADC底层初始化已经完成，且ADC转换尚未启动。
 * @note 本接口可能等待硬件校准完成，只允许在系统初始化上下文调用。
 */
bool AdcService_Calibrate(AdcService_UnitType unit);

/**
 * @brief 启动全部ADC采集链路及其定时器触发源。
 *
 * 依次启动ADC1循环DMA、ADC2和ADC3注入组、TIM4触发以及TIM1 CH4
 * ADC触发。任一步骤失败时尝试回滚已启动的步骤。
 *
 * @return 全部采集链路启动成功时返回true，否则返回false。
 *
 * @pre ADC1、ADC2和ADC3底层初始化及校准均已成功完成。
 * @pre TIM1、TIM4底层初始化及TIM1 CH4比较值设置已经完成。
 */
bool AdcService_Start(void);

/**
 * @brief 读取指定ADC模块的指定逻辑通道原始值。
 *
 * ADC1通道读取循环DMA缓冲区；ADC2和ADC3通道直接读取对应注入Rank
 * 的数据寄存器。本接口不启动转换，也不轮询转换完成状态。
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
                            AdcService_ValueType *value);

#endif /* ADC_SERVICE_H */

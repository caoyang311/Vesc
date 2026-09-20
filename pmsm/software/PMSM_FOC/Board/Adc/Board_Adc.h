#ifndef BOARD_ADC_H
#define BOARD_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief ADC 原始采样值类型（12 位分辨率，范围 0..4095）。
 */
typedef uint16_t Adc_ValueType;

/**
 * @brief ADC 逻辑通道 ID。
 *
 * 新增通道时：在此追加枚举项，并在 Adc_Config.h 中追加同名硬件映射宏，
 * 再在 Board_Adc.c 的通道配置表中追加一个表项，无需改动任何对外接口。
 */
typedef enum
{
    ADC_CH_ID_PHASE_U_CURRENT = 0,  /**< U 相电流，ADC2 注入通道 */
    ADC_CH_ID_PHASE_V_CURRENT,      /**< V 相电流，ADC2 注入通道 */
    ADC_CH_ID_PHASE_W_CURRENT,      /**< W 相电流，ADC2 注入通道 */
    ADC_CH_ID_DC_BUS_VOLTAGE,       /**< 母线电压，ADC1 规则通道 */
    ADC_CH_ID_POWER_STAGE_TEMP,     /**< 功率级温度，ADC1 规则通道 */
    ADC_CH_ID_COUNT                 /**< 逻辑通道总数，必须位于末尾 */
} Adc_ChannelType;

/**
 * @brief ADC 实例 ID，对应 STM32 的 ADC1/ADC2。
 */
typedef enum
{
    ADC_INSTANCE_1 = 1U,    /**< ADC1 */
    ADC_INSTANCE_2 = 2U,    /**< ADC2 */
    ADC_INSTANCE_COUNT      /**< 实例总数 */
} Adc_InstanceType;

/**
 * @brief 通道归属的转换类型（注入通道由硬件触发，规则通道由 DMA 搬运）。
 */
typedef enum
{
    ADC_CH_KIND_REGULAR = 0U,   /**< 规则通道，结果经 DMA 搬运 */
    ADC_CH_KIND_INJECTED = 1U   /**< 注入通道，结果经注入中断锁存 */
} Adc_ChannelKindType;

/**
 * @brief ADC 模块接口返回状态。
 */
typedef enum
{
    ADC_RESULT_OK = 0U,             /**< 操作成功 */
    ADC_RESULT_NOT_INITIALIZED,     /**< 模块尚未执行 Adc_Init */
    ADC_RESULT_INVALID_CHANNEL,     /**< 逻辑通道 ID 非法或该通道未使能 */
    ADC_RESULT_INVALID_PARAMETER,   /**< 输出指针为空 */
    ADC_RESULT_NOT_READY,           /**< 该通道尚无有效采样结果 */
    ADC_RESULT_HAL_ERROR            /**< HAL 启动失败或转换过程出错 */
} Adc_Result;

/**
 * @brief 初始化 ADC 模块。
 *
 * @note 只复位模块内部状态并解析通道到 ADC 实例的映射，不访问硬件、
 *       不启动转换。底层外设仍由 CubeMX 生成的 MX_ADCx_Init() 配置。
 */
void Adc_Init(void);

/**
 * @brief 反初始化 ADC 模块，停止转换并复位全部内部状态。
 */
void Adc_DeInit(void);

/**
 * @brief 启动全部已配置通道的转换。
 *
 * @return ADC_RESULT_OK 启动成功；
 *         ADC_RESULT_NOT_INITIALIZED 未初始化；
 *         ADC_RESULT_HAL_ERROR 底层 HAL 启动失败。
 *
 * @note 规则通道以循环 DMA 方式启动，注入通道使能后由 TIM1_CH4 硬件触发。
 *       本接口非幂等：规则组配置为单次转换，上层需在每个采样周期反复调用
 *       以重新装载并触发规则组转换。重复调用时底层 DMA 可能返回 HAL_BUSY，
 *       该情况不视为失败。
 */
Adc_Result Adc_Start(void);

/**
 * @brief 停止全部已配置通道的转换。
 *
 * @return ADC_RESULT_OK 停止成功或本就处于停止状态；
 *         ADC_RESULT_NOT_INITIALIZED 未初始化。
 */
Adc_Result Adc_Stop(void);

/**
 * @brief 读取指定逻辑通道最近一次的原始采样值。
 *
 * @param[in]  ChannelId 逻辑通道 ID，取值小于 ADC_CH_ID_COUNT。
 * @param[out] Value     用于返回原始计数值，不得为空指针。
 *
 * @return ADC_RESULT_OK 读取成功；
 *         ADC_RESULT_INVALID_PARAMETER 输出指针为空；
 *         ADC_RESULT_NOT_INITIALIZED 未初始化；
 *         ADC_RESULT_INVALID_CHANNEL 通道 ID 非法或该通道未使能；
 *         ADC_RESULT_NOT_READY 该通道尚无有效采样结果；
 *         ADC_RESULT_HAL_ERROR 转换过程报告过错误。
 */
Adc_Result Adc_ReadChannel(Adc_ChannelType ChannelId, Adc_ValueType *Value);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_ADC_H */

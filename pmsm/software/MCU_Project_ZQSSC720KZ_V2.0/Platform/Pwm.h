#ifndef PWM_H
#define PWM_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    PWM_CHANNEL_U = 0,
    PWM_CHANNEL_V,
    PWM_CHANNEL_W,
    PWM_CHANNEL_COUNT
} Pwm_ChannelType;

/**
 * @brief 设置一组三相互补PWM输出的比较预装载值。
 *
 * 对应通道的主输出和互补输出共享同一个比较寄存器。写入值通过TIM1
 * 比较预装载机制生效，本接口不改变PWM输出的启停状态。
 *
 * @param[in] channel PWM逻辑通道，取值范围为PWM_CHANNEL_U至
 *                    PWM_CHANNEL_W。
 * @param[in] preload 比较预装载值，允许范围为0至当前TIM1自动重装载值。
 *
 * @return 通道和值有效并完成写入时返回true，否则返回false。
 *
 * @pre MX_TIM1_Init()已经执行完成。
 */
void Pwm_SetCompare(Pwm_ChannelType channel, uint16_t preload);

/**
 * @brief 启动TIM1的六路互补PWM输出。
 *
 * 按CH1、CH1N、CH2、CH2N、CH3、CH3N的顺序启动输出。任一路启动失败
 * 时尝试关闭全部六路输出，避免部分通道保持运行。
 *
 * @return 六路输出全部启动成功时返回true，否则返回false。
 *
 * @pre MX_TIM1_Init()已经执行完成，且CCR1至CCR3已写入安全初值。
 */
void Pwm_StartOutputs(void);

/**
 * @brief 关闭TIM1的六路互补PWM输出。
 *
 * 先关闭CH1N、CH2N和CH3N，再关闭CH1、CH2和CH3。即使某一路关闭
 * 失败，仍继续尝试关闭其余输出。
 *
 * @return 所有底层关闭操作成功时返回true，否则返回false。
 *
 * @pre MX_TIM1_Init()已经执行完成。
 */
void Pwm_StopOutputs(void);

/**
 * @brief 启动TIM1 CH4的ADC注入组触发事件。
 *
 * CH4仅产生内部比较事件，不启用物理GPIO输出。ADC模块必须单独完成
 * 校准、注入组配置和启动。
 *
 * @return TIM1 CH4启动成功时返回true，否则返回false。
 *
 * @pre MX_TIM1_Init()已经执行完成，且CCR4已写入有效采样位置。
 */
void Pwm_StartAdcTrigger(void);

/**
 * @brief 关闭TIM1 CH4的ADC注入组触发事件。
 *
 * 本接口不修改CCR4的当前预装载值，也不停止ADC模块。
 *
 * @return TIM1 CH4关闭成功时返回true，否则返回false。
 *
 * @pre MX_TIM1_Init()已经执行完成。
 */
void Pwm_StopAdcTrigger(void);

/**
 * @brief 设置TIM1 CH4的ADC触发比较预装载值。
 *
 * @param[in] preload ADC触发比较预装载值，允许范围为0至当前TIM1
 *                    自动重装载值。
 *
 * @return 预装载值有效并完成写入时返回true，否则返回false。
 *
 * @pre MX_TIM1_Init()已经执行完成。
 */
void Pwm_SetAdcTriggerCompare(uint16_t preload);

#endif /* PWM_H */

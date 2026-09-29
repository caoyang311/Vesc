#include "Pwm.h"
#include "tim.h"

static const uint32_t Pwm_HalChannels[PWM_CHANNEL_COUNT] =
{
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3
};

_Static_assert((sizeof(Pwm_HalChannels) / sizeof(Pwm_HalChannels[0])) ==
                   PWM_CHANNEL_COUNT,
               "PWM channel configuration count mismatch");

/**
 * @brief 设置一组三相互补PWM输出的比较预装载值。
 *
 * 将预装载值直接写入所选TIM1通道的比较寄存器。该函数执行时间固定、
 * 不阻塞且不保存运行时状态，适合快速控制路径调用。
 *
 * @param[in] channel PWM逻辑通道。
 * @param[in] preload 比较预装载值，允许范围为0至当前TIM1自动重装载值。
 *
 * @return 通道和值有效并完成写入时返回true，否则返回false。
 *
 * @pre MX_TIM1_Init()已经执行完成。
 */
void Pwm_SetCompare(Pwm_ChannelType channel, uint16_t preload)
{
    const uint32_t auto_reload = __HAL_TIM_GET_AUTORELOAD(&htim1);

    if (((uint32_t)channel < (uint32_t)PWM_CHANNEL_COUNT) &&
        ((uint32_t)preload <= auto_reload))
    {
        __HAL_TIM_SET_COMPARE(&htim1,
                              Pwm_HalChannels[(uint32_t)channel],
                              (uint32_t)preload);
    }
}

/**
 * @brief 启动TIM1的六路互补PWM输出。
 *
 * 每组先启动主输出再启动互补输出。启动失败时尝试关闭全部输出，避免
 * 部分桥臂继续运行。该函数不记录输出使能状态。
 *
 * @return 六路输出全部启动成功时返回true，否则返回false。
 *
 * @pre MX_TIM1_Init()已经执行完成，且CCR1至CCR3已写入安全初值。
 */
void Pwm_StartOutputs(void)
{
    uint32_t index = 0U;

    for (index = 0U; index < (uint32_t)PWM_CHANNEL_COUNT; ++index)
    {
         HAL_TIM_PWM_Start(&htim1, Pwm_HalChannels[index]); 
         HAL_TIMEx_PWMN_Start(&htim1, Pwm_HalChannels[index]);
    }
}

/**
 * @brief 关闭TIM1的六路互补PWM输出。
 *
 * 先关闭全部互补输出，再关闭全部主输出。单次HAL操作失败不会阻止
 * 后续通道的关闭尝试。该函数不记录输出使能状态。
 *
 * @return 所有底层关闭操作成功时返回true，否则返回false。
 *
 * @pre MX_TIM1_Init()已经执行完成。
 */
void Pwm_StopOutputs(void)
{
    uint32_t index = 0U;


    for (index = 0U; index < (uint32_t)PWM_CHANNEL_COUNT; ++index)
    {
        HAL_TIMEx_PWMN_Stop(&htim1, Pwm_HalChannels[index]);
        HAL_TIM_PWM_Stop(&htim1, Pwm_HalChannels[index]);   
    }
}

/**
 * @brief 启动TIM1 CH4的ADC注入组触发事件。
 *
 * 当前底层将CH4配置为PWM1模式，因此使用HAL PWM接口使能内部OC4REF
 * 比较事件。该函数不启动ADC，也不记录触发使能状态。
 *
 * @return TIM1 CH4启动成功时返回true，否则返回false。
 *
 * @pre MX_TIM1_Init()已经执行完成，且CCR4已写入有效采样位置。
 */
void Pwm_StartAdcTrigger(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

/**
 * @brief 关闭TIM1 CH4的ADC注入组触发事件。
 *
 * 当前底层将CH4配置为PWM1模式，因此使用HAL PWM接口关闭内部OC4REF
 * 比较事件。本接口不修改CCR4，也不停止ADC。
 *
 * @return TIM1 CH4关闭成功时返回true，否则返回false。
 *
 * @pre MX_TIM1_Init()已经执行完成。
 */
void Pwm_StopAdcTrigger(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
}

/**
 * @brief 设置TIM1 CH4的ADC触发比较预装载值。
 *
 * 该函数直接写入CCR4，不改变CH4启停状态，也不操作ADC模块。
 *
 * @param[in] preload ADC触发比较预装载值，允许范围为0至当前TIM1
 *                    自动重装载值。
 *
 * @return 预装载值有效并完成写入时返回true，否则返回false。
 *
 * @pre MX_TIM1_Init()已经执行完成。
 */
void Pwm_SetAdcTriggerCompare(uint16_t preload)
{
    const uint32_t auto_reload = __HAL_TIM_GET_AUTORELOAD(&htim1);

    if ((uint32_t)preload <= auto_reload)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)preload);
    }
}

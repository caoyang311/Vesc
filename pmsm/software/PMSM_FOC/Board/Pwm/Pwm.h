#ifndef PWM_H
#define PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"


/**
 * @brief 开启PWM互补输出
 * 
 * @return void
 */
void Pwm_Start(void);
/**
 * @brief 关闭PWM互补输出
 * 
 * @return void
 */
void Pwm_Stop(void);
/**
 * @brief 设置3相PWM占空比
 * 
 * @param U_value U相占空比
 * @param V_value V相占空比
 * @param W_value W相占空比
 * @return void
 */
void Pwm_Set_UVW_CCR(uint16_t U_value, uint16_t V_value, uint16_t W_value);
/**
 * @brief 开启ADC触发
 * 
 * @return void
 */
void Pwm_StartAdcTrigger(void);
/**
 * @brief 关闭ADC触发
 * 
 * @return void
 */
void Pwm_StopAdcTrigger(void);

#ifdef __cplusplus
}
#endif

#endif /* PWM_H */

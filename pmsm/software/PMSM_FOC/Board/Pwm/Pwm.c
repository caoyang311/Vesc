#include "Pwm.h"
#include "tim.h"


/**
 * @brief PWM相位标识符
 */
typedef enum
{
  PWM_PHASE_U = 0U, /* U相PWM输出 */
  PWM_PHASE_V = 1U, /* V相PWM输出 */
  PWM_PHASE_W = 2U /* W相PWM输出 */
} Pwm_Phase;


static void Pwm_StartPWM(Pwm_Phase phase);
static void Pwm_StopPWM(Pwm_Phase phase);
static void Pwm_SetCCR(Pwm_Phase phase, uint16_t value);
static void Pwm_Start_U_PWM(void);
static void Pwm_Start_V_PWM(void);
static void Pwm_Start_W_PWM(void);
static void Pwm_Stop_U_PWM(void);
static void Pwm_Stop_V_PWM(void);
static void Pwm_Stop_W_PWM(void);

/*
 * @brief 打开PWM互补输出
 * 
 * @param phase PWM相位
 * @return void
 */
static void Pwm_StartPWM(Pwm_Phase phase)
{
   HAL_TIM_PWM_Start(&htim1, phase);
   HAL_TIMEx_PWMN_Start(&htim1, phase);
}
/*
 * @brief 关闭PWM互补输出
 * 
 * @param phase PWM相位
 * @return void
 */
static void Pwm_StopPWM(Pwm_Phase phase)
{
  HAL_TIM_PWM_Stop(&htim1, phase);
  HAL_TIMEx_PWMN_Stop(&htim1, phase);
}
/*
 * @brief 设置PWM占空比
 * 
 * @param value PWM占空比
 * @return void
 */
static void Pwm_SetCCR(Pwm_Phase phase, uint16_t value)
{
  __HAL_TIM_SET_COMPARE(&htim1, phase, value);
}
/**
 * @brief 打开U相PWM输出
 * 
 * @return void
 */
static void Pwm_Start_U_PWM(void)
{
  Pwm_StartPWM(PWM_PHASE_U);
}
/**
 * @brief 打开V相PWM输出
 * 
 * @return void
 */
static void Pwm_Start_V_PWM(void)
{
  Pwm_StartPWM(PWM_PHASE_V);
}
/**
 * @brief 打开W相PWM输出
 * 
 * @return void
 */
static void Pwm_Start_W_PWM(void)
{
  Pwm_StartPWM(PWM_PHASE_W);
}
/**
 * @brief 关闭U相PWM输出
 * 
 * @return void
 */
static void Pwm_Stop_U_PWM(void)
{
  Pwm_StopPWM(PWM_PHASE_U);
}
/**
 * @brief 关闭V相PWM输出
 * 
 * @return void
 */
static void Pwm_Stop_V_PWM(void)
{
  Pwm_StopPWM(PWM_PHASE_V);
}
/**
 * @brief 关闭W相PWM输出
 * 
 * @return void
 */
static void Pwm_Stop_W_PWM(void)
{
  Pwm_StopPWM(PWM_PHASE_W);
}

/**
 * @brief 开启PWM互补输出
 * 
 * @return void
 */
void Pwm_Start(void)
{
  Pwm_Start_U_PWM();
  Pwm_Start_V_PWM();
  Pwm_Start_W_PWM();
}
/**
 * @brief 关闭PWM互补输出
 * 
 * @return void
 */
void Pwm_Stop(void)
{
  Pwm_Stop_U_PWM();
  Pwm_Stop_V_PWM();
  Pwm_Stop_W_PWM();
}

/**
 * @brief 设置3相PWM占空比
 * 
 * @param value PWM占空比
 * @return void
 */
void Pwm_Set_UVW_CCR(uint16_t U_value, uint16_t V_value, uint16_t W_value)
{
  Pwm_SetCCR(PWM_PHASE_U, U_value);
  Pwm_SetCCR(PWM_PHASE_V, V_value);
  Pwm_SetCCR(PWM_PHASE_W, W_value);
}

/*
 * @brief 开启ADC触发
 * 
 * @return void
 */
void Pwm_StartAdcTrigger(void)
{
   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 8400-5U);
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}
/*
 * @brief 关闭ADC触发
 * 
 * @return void
 */
void Pwm_StopAdcTrigger(void)
{
   HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
}

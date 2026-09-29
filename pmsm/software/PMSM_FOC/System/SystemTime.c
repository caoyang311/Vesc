/**
 * @file SystemTime.c
 * @brief 基于 TIM6 的系统毫秒时间基实现。
 */
#include "SystemTime.h"

#include "tim.h"

/* 系统时间计数器，单位为毫秒。 */
static volatile uint32_t SystemTime_TickMs = 0U;
/**
 * @brief 初始化系统时间基。
 */
void SystemTime_Init(void)
{
    SystemTime_TickMs = 0U;
}
/**
 * @brief 启动系统时间基。
 */
void SystemTime_Start(void)
{ 
    HAL_TIM_Base_Start_IT(&htim6);    
}
/**
 * @brief 获取当前系统时间（毫秒）。
 * @return uint32_t 当前系统时间（毫秒）。
 */
uint32_t SystemTime_GetTimeMs(void)
{
    return SystemTime_TickMs;
}
/**
 * @brief 处理 1ms 时间中断。
 */
static void SystemTime_Notification1ms(void)
{
    SystemTime_TickMs++;
}
/**
 * @brief 处理 TIM6 定时器中断。
 * @param[in] TimHandle 定时器句柄。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *TimHandle)
{
  if ((TimHandle != (TIM_HandleTypeDef *)0) &&
      (TimHandle->Instance == TIM6))
  {
    SystemTime_Notification1ms();
  }
  else
  {
    /* 其他定时器事件不在系统时间模块中处理。 */
  }
}
#include "led.h"
#include "timer.h"
#include "gpio.h"
/**
 * @brief       LED任务
 * 
 * @param       void     
 * @return      void
 */
void LED_Task(void)
{
    static uint32_t led_timer;
    if((Get_Systick_Timer()>0) &&((Get_Systick_Timer()-led_timer >500)))
    {
        led_timer = Get_Systick_Timer();
        GPIO_ToggleBits(LED0_GPIO_Port,LED0_Pin);
				
    }
}



/**
 * @brief       打开LED1:指示电机开始运行
 * 
 * @param       void     
 * @return      void
 */
void LED1_ON(void)
{
	GPIO_ResetBits(LED1_GPIO_Port,LED1_Pin);
}

/**
 * @brief       关闭LED1:指示电机停止运行
 * 
 * @param       void     
 * @return      void
 */
void LED1_OFF(void)
{
	GPIO_SetBits(LED1_GPIO_Port,LED1_Pin);
}


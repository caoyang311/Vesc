#include "gpio.h"
#include "string.h"
#include "timer.h"
#include "key.h"

buttonType button[1];
/**
 * @brief       按键GPIO配置
 * 
 * @param       void     
 * @return      void
 */ 
void Key_Config(void)
{
	button[0].GPIO_Port = KEY0_GPIO_Port;
	button[0].GPIO_Pin = KEY0_Pin;
 
	button[1].GPIO_Port = KEY1_GPIO_Port;
	button[1].GPIO_Pin = KEY1_Pin;
 
	button[2].GPIO_Port = KEY2_GPIO_Port;
	button[2].GPIO_Pin = KEY2_Pin;
  

}
/**
 * @brief       按键参数初始化
 * 
 * @param       button 参数结构体     
 * @return      void
 */  
void Key_ParaInit(buttonType* button)
{
	button->hold_cnt = 0;
	button->press_flag = 0;
	button->release_flag = 0;
}
/**
 * @brief       按键扫描
 * 
 * @param       button 参数结构体     
 * @return      void
 */  
void Key_Scan(buttonType* button)
{
	switch(button->key)
	{
		case KEY_NULL:
		{
			/* if falling edge captured */
			if(GPIO_ReadInputDataBit(button->GPIO_Port,button->GPIO_Pin) == 0)
			{
				button->key = KEY_DOWN;
			}
			else if(GPIO_ReadInputDataBit(button->GPIO_Port,button->GPIO_Pin) == 1)
			{
				button->key = KEY_NULL;
			}		
			/**********************judge***********************/
			/* if high_time_count is longer than LONG_PRESS_TIME, consider BUTTON_LONG_PRESS */
			if(button->hold_cnt > LONG_PRESS_TIME)
			{
				button->buttonAction = BUTTON_LONG_PRESS;
				Key_ParaInit(button);
			}
			/* 
				only the latest press time is in range of [CLICK_MIN_TIME,CLICK_MAX_TIME] can be regarded valid
			*/
			else if((button->hold_cnt > CLICK_MIN_TIME && button->hold_cnt < CLICK_MAX_TIME))
			{
				if(button->press_flag ==1)
				{
					button->buttonAction = BUTTON_SINGLE;
				}
				Key_ParaInit(button);
			}
			else
			{
				Key_ParaInit(button);
			}
			break;
		}
		
		case KEY_DOWN:
		{
			button->key = KEY_PRESS;
			
			/* as long as falling edge occurring,press_flag++ */
			button->press_flag = 1;
			
			button->release_flag = 0; 			/* means that the button has been pressed */
 
			button->hold_cnt = 0;				/* reset hold time count */
			break;
		}
		
		case KEY_PRESS:
		{
			/* when button was kept pressed, hold count++ */
			if(GPIO_ReadInputDataBit(button->GPIO_Port,button->GPIO_Pin) == 0)
			{
				button->key = KEY_PRESS;
				button->hold_cnt++;
			}
			/* when button was released, change state */
			else if(GPIO_ReadInputDataBit(button->GPIO_Port,button->GPIO_Pin) == 1)
			{
				button->key = KEY_UP;
			}
			break;
		}
		
		case KEY_UP:
		{
			button->key = KEY_NULL;
			button->release_flag = 1;			/* means that the button is released */
			break;
		}
		default:
			break;
	}
}
/**
 * @brief       按键扫描任务
 * 
 * @param       void    
 * @return      void
 */ 
void Key_Task(void)
{
    static uint32_t key_timer;
    if((Get_Systick_Timer()>0) &&((Get_Systick_Timer()-key_timer >10)))
    {
        key_timer = Get_Systick_Timer();
        Key_Scan(&button[0]);
    }
}


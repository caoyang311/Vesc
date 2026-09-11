/**
 * *****************************************************************************
 * @file        main.c
 * @brief       锟斤拷锟斤拷锟斤拷
 * @author       (caoyang)
 * @date        2023-11-13
 * @copyright   锟斤拷锟斤拷锟斤拷锟斤拷锟睫癸拷司
 * *****************************************************************************
 */

#include "timer.h"
#include "adc.h"
#include "uart.h"
#include "gpio.h"
#include "dac.h"
#include "i2c.h"
#include "key.h"
#include "motor.h"
#include "led.h"
#include "debug.h"
#include "at24c02.h"
#include "abs_encoder.h"
#include "pid.h"

void disable_all_interrupts(void) 
{
    __disable_irq(); 
}

void enable_all_interrupts(void) 
{
    __enable_irq();
}
float flux;

int main(void)
{
	//hardware init
	disable_all_interrupts();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	gpio_init();
	iic_init();
	adc_init();
 	timer_init();
	uart1_init();
	uart3_init();
	dac_init();
	//application init
	Key_Config();
	Motor_Init();
	PID_init();                           
	enable_all_interrupts();
	
	while(1)
	{
		flux = MOTOR_FLUX;
		LED_Task();	
		Key_Task();	
    Debug_Task();
	}
}

#ifdef USE_FULL_ASSERT
/**
* @brief  assert_failed
*         Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  File: pointer to the source file name
* @param  Line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {}
}
#endif	



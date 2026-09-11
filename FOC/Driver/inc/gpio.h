/**
 * *****************************************************************************
 * @file        gpio.h
 * @brief       GPIO底层驱动
 * @author      caoyang
 * @date        2023-11-13
 * @copyright   欢阳技有限公司
 * *****************************************************************************
 */
#ifndef GPIO_H 
#define GPIO_H 

/*----------------------------------include-----------------------------------*/
#include "misc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
/*-----------------------------------macro------------------------------------*/

//LED GPIO
#define LED0_Pin GPIO_Pin_0
#define LED0_GPIO_Port GPIOE

#define LED1_Pin GPIO_Pin_1
#define LED1_GPIO_Port GPIOE

//KEY GPIO
#define KEY0_Pin GPIO_Pin_2
#define KEY0_GPIO_Port GPIOE

#define KEY1_Pin GPIO_Pin_3
#define KEY1_GPIO_Port GPIOE

#define KEY2_Pin GPIO_Pin_4
#define KEY2_GPIO_Port GPIOE

//电机驱动板使能GPIO 
#define PM1_CTRL_SD_Pin GPIO_Pin_10
#define PM1_CTRL_SD_GPIO_Port GPIOF

//FOC执行速度测试GPIO
#define Test_Pin        GPIO_Pin_7
#define Test_GPIO_Port  GPIOI

//485通信发送接收切换GPIO
#define TX_EN_Pin        GPIO_Pin_10
#define TX_EN_GPIO_Port  GPIOI


/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void gpio_init(void);
/*------------------------------------test------------------------------------*/


#endif	/* GPIO_H */








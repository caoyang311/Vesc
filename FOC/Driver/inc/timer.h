/**
 * *****************************************************************************
 * @file        timer.h
 * @brief       TIM底层驱动
 * @author      caoyang
 * @date        2023-11-13
 * @copyright   欢阳技有限公司
 * *****************************************************************************
 */


#ifndef TIMER_H 
#define TIMER_H 

/*----------------------------------include-----------------------------------*/
#include "misc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
/*-----------------------------------macro------------------------------------*/
//电机PWM GPIO
#define M1_PWM_UH_Pin GPIO_Pin_8
#define M1_PWM_UH_GPIO_Port GPIOA

#define M1_PWM_UL_Pin GPIO_Pin_13
#define M1_PWM_UL_GPIO_Port GPIOB


#define M1_PWM_VH_Pin GPIO_Pin_9
#define M1_PWM_VH_GPIO_Port GPIOA

#define M1_PWM_VL_Pin GPIO_Pin_14
#define M1_PWM_VL_GPIO_Port GPIOB

#define M1_PWM_WH_Pin GPIO_Pin_10
#define M1_PWM_WH_GPIO_Port GPIOA

#define M1_PWM_WL_Pin GPIO_Pin_15
#define M1_PWM_WL_GPIO_Port GPIOB

#define M1_PWM_BRAKE_Pin GPIO_Pin_12
#define M1_PWM_BRAKE_GPIO_Port GPIOB

//编码器 GPIO
#define	M1_ENCODER_A_Pin	GPIO_Pin_6 //A+
#define	M1_ENCODER_B_Pin	GPIO_Pin_7 //B+
#define	M1_ENCODER_AB_Port	GPIOC

#define	M1_ENCODER_Z_Pin	GPIO_Pin_6 //Z+
#define	M1_ENCODER_Z_Port	GPIOE            
/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void timer_init(void);
uint32_t Get_Systick_Timer(void);
/*------------------------------------test------------------------------------*/

#endif	/* TIMER_H */








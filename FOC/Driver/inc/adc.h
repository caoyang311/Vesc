/**
 * *****************************************************************************
 * @file        math.h
 * @brief       adc底层驱动
 * @author      caoyang
 * @date        2023-11-13
 * @copyright   欢阳技有限公司
 * *****************************************************************************
 */
#ifndef ADC_H 
#define ADC_H 

/*----------------------------------include-----------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
/*-----------------------------------macro------------------------------------*/


//U相电流GPIO
#define M1_CURR_AMPL_U_Pin GPIO_Pin_0
#define M1_CURR_AMPL_U_GPIO_Port GPIOB
//V相电流GPIO
#define M1_CURR_AMPL_V_Pin GPIO_Pin_6
#define M1_CURR_AMPL_V_GPIO_Port GPIOA
//W相电流GPIO
#define M1_CURR_AMPL_W_Pin GPIO_Pin_3
#define M1_CURR_AMPL_W_GPIO_Port GPIOA
//母线电压GPIO
#define	M1_BUS_V_Pin	GPIO_Pin_1
#define	M1_BUS_V_GPIO_Port	GPIOB
//板子温度GPIO
#define	M1_TEMP_V_Pin	GPIO_Pin_0
#define	M1_TEMP_V_GPIO_Port	GPIOA

// U V W 三相ADC通道
#define M1_PHASE_A_ADC_CHANNEL		ADC_Channel_8
#define M1_PHASE_B_ADC_CHANNEL		ADC_Channel_6
#define M1_PHASE_C_ADC_CHANNEL		ADC_Channel_3
/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/
extern __IO uint16_t ADCConvertedValue[2];
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void adc_init(void);
/*------------------------------------test------------------------------------*/


#endif	/* ADC_H */

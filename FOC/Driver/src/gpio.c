/**
 * *****************************************************************************
 * @file        gpio.c
 * @brief       gpio 初始化
 * @author       (caoyang)
 * @date        2023-11-13
 * @copyright   欢阳技有限公司
 * *****************************************************************************
 */
#include "gpio.h"

void gpio_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
	//LED GPIO初始化设置
  GPIO_InitStructure.GPIO_Pin = LED0_Pin | LED1_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(LED1_GPIO_Port, &GPIO_InitStructure);
	
	//GPIOF10初始化设置
  GPIO_InitStructure.GPIO_Pin = PM1_CTRL_SD_Pin;
  GPIO_Init(PM1_CTRL_SD_GPIO_Port, &GPIO_InitStructure);
	
	
	
	//GPIOI7初始化设置
  GPIO_InitStructure.GPIO_Pin = Test_Pin;
  GPIO_Init(Test_GPIO_Port, &GPIO_InitStructure);

	//GPIOI10初始化设置
	GPIO_InitStructure.GPIO_Pin = TX_EN_Pin;
  GPIO_Init(TX_EN_GPIO_Port, &GPIO_InitStructure);
	
	//KEY GPIO 初始化设置
  GPIO_InitStructure.GPIO_Pin = KEY0_Pin | KEY1_Pin | KEY2_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(KEY0_GPIO_Port, &GPIO_InitStructure);	
	
	GPIO_SetBits(GPIOE,LED0_Pin | LED1_Pin);
	GPIO_ResetBits(PM1_CTRL_SD_GPIO_Port,PM1_CTRL_SD_Pin);
	
}


/**
 * *****************************************************************************
 * @file        dac.c
 * @brief       dac打印输出
 * @author       (caoyang)
 * @date        2023-11-13
 * @copyright   欢阳技有限公司
 * *****************************************************************************
 */
#include "dac.h"

void dac_init(void)
{
	DAC_InitTypeDef  DAC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;


  /* GPIOA clock enable (to be used with DAC) */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);                         
  /* DAC Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

  /* DAC channel 1 & 2 (DAC_OUT1 = PA.4)(DAC_OUT2 = PA.5) configuration */
  GPIO_InitStructure.GPIO_Pin = DBG_DAC_CH1_Pin | DBG_DAC_CH2_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(DBG_DAC_CH1_GPIO_Port, &GPIO_InitStructure);
  /* DAC channel1 Configuration */
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);
	
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
  DAC_Init(DAC_Channel_2, &DAC_InitStructure);

  /* Enable DAC Channel2 */
  DAC_Cmd(DAC_Channel_2, ENABLE);
 /* Enable DAC Channel1 */
  DAC_Cmd(DAC_Channel_1, ENABLE);
}

//  输出DAC
void dac_output_data(uint16_t data1,uint16_t data2)
{

	DAC->DHR12R1 = data1;
	DAC->DHR12R2 =	data2;
	
}


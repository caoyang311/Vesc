/**
 * *****************************************************************************
 * @file        adc.c
 * @brief       adc三相电流、母线电压、温度采集驱动
 * @author       (caoyang)
 * @date        2023-11-13
 * @copyright   欢阳技有限公司
 * *****************************************************************************
 */
#include "adc.h"
#include "motor.h"
#include "abs_encoder.h"
#include "abz_encoder.h"

__IO uint16_t ADCConvertedValue[2];//[0]-temp [1]-bus voltage

/**
 * @brief ADC1底层驱动      
 * 
 */
void adc_init(void)
{
    ADC_InitTypeDef ADC_InitStruct;
	  ADC_CommonInitTypeDef ADC_CommonInitStructure;
	  GPIO_InitTypeDef GPIO_InitStruct;
    DMA_InitTypeDef DMA_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
    //外设时钟使能
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);//使能ADC1外设时钟
	  RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIOAEN,ENABLE); //使能GPIO_A时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIOBEN,ENABLE); //使能GPIO_B时钟
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE); //使能DMA2外设时钟
	
    // DMA2 数据流0 通道0配置
    DMA_DeInit(DMA2_Stream0);//复位DMA2_Stream0 
    DMA_InitStructure.DMA_Channel = DMA_Channel_0; //通道选择：通道0
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADCConvertedValue; //内存地址：需要把数据搬运的RAM地址（数组首地址）
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);//外设地址 
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//方向：由外设到内存
    DMA_InitStructure.DMA_BufferSize = 2;//DMA缓存大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址自增：不使能
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址自增：使能
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//外设数据宽度：16bit
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//内存数据宽度：16bit
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//设置传输模式：连续模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA优先级
    //由于关闭了FIFO模式，后面四个参数可以忽略
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;//关闭FIFO
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;//传输阈值：1/2
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;      //一次传输数据多少：单个数据
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//一次传输数据多少：单个数据
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);
    //使能DMA2
    DMA_Cmd(DMA2_Stream0, ENABLE);

    //A、B、C三相电流采集GPIO初始化
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;//模拟模式.
    GPIO_InitStruct.GPIO_Pin =  M1_CURR_AMPL_V_Pin |M1_CURR_AMPL_W_Pin;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;//无上下拉.
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;//模拟模式.
    GPIO_InitStruct.GPIO_Pin =  M1_CURR_AMPL_U_Pin;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;//无上下拉.
    GPIO_Init(GPIOB, &GPIO_InitStruct); 

    //母线电压采集GPIO初始化
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStruct.GPIO_Pin = M1_BUS_V_Pin;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    //温度采集GPIO初始化
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStruct.GPIO_Pin = M1_TEMP_V_Pin;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    ADC_DeInit();
    //ADC公共参数初始化
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//采样时间间隔：5个周期
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;// DMA直接访问模式，只有在双重或者三重模式才需要设置
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//ADC1时钟分频：84M/4=21MHZ
    ADC_CommonInit(&ADC_CommonInitStructure); 
    //ADC初始化
    ADC_InitStruct.ADC_ScanConvMode = ENABLE;       //使能扫描模式
	  ADC_InitStruct.ADC_ContinuousConvMode = ENABLE; //使能连续转换 
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right; //数据右对齐
		ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;;//禁止外部触发，使用软件触发模式
    ADC_InitStruct.ADC_NbrOfConversion = 2; //连续转换通道数：2通道
    ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;//ADC分辨率：12bit
    ADC_Init(ADC1, &ADC_InitStruct);
		
		//注入组通道个数：3
		ADC_InjectedSequencerLengthConfig(ADC1,3);
    ADC_ExternalTrigInjectedConvConfig(ADC1,ADC_ExternalTrigInjecConv_T1_CC4);//外部触发：TIM1_CC4
    ADC_ExternalTrigInjectedConvEdgeConfig(ADC1,ADC_ExternalTrigInjecConvEdge_Rising);//触发方式：上升沿		
    // ADC1 注入组通道配置
    ADC_InjectedChannelConfig(ADC1,ADC_Channel_8,1,ADC_SampleTime_15Cycles);//注入通道1对应A相电流
    ADC_InjectedChannelConfig(ADC1,ADC_Channel_6,2,ADC_SampleTime_15Cycles);//注入通道2对应B相电流
    ADC_InjectedChannelConfig(ADC1,ADC_Channel_3,3,ADC_SampleTime_15Cycles);//注入通道3对应C相电流
		
		// ADC1 规则组通道配置
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_15Cycles);//温度
	  ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 2, ADC_SampleTime_15Cycles);//母线电压	
		   
    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE); /* 开启DMA-ADC循环模式 */
    ADC_DMACmd(ADC1, ENABLE);/* 使能 ADC1 DMA */
		
    ADC_ITConfig(ADC1,ADC_IT_JEOC,ENABLE);//注入转换完成中断
		ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);   //清除中断标志位 
	  ADC_Cmd(ADC1, ENABLE);//使能ADC1
    ADC_SoftwareStartConv(ADC1);//开启ADC转换		
		//中断使能
		NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
		NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //子优先级0
		NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
		NVIC_Init(&NVIC_InitStructure); 
   
}

/**
 * @brief       ADC注入组转换完成中断
 * 
 */
void ADC_IRQHandler(void)
{
	if(ADC_GetFlagStatus(ADC1,ADC_FLAG_JEOC) == SET)//ADC中断
	{
		switch (BLDC_Motor.Motor_State)
    {
			case MOTOR_RUN:			
					MCL_HighFrequency_Task();
				break;
			case MOTOR_INIT:
				if(BLDC_Motor.Motor_Init_State == MOTOR_INIT_ENCODER_ALIGNMENT_START)
        {
          ABS_Encoder_Calibration();
        }
        break;
        		
			default:
				break;
		}
		ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);  
	}
}

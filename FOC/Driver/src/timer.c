

/**
 * *****************************************************************************
 * @file        timer_1.c
 * @brief       TIM1_底层驱动_源文件
 * @author       (caoyang)
 * @date        2023-11-13
 * @copyright   欢阳技有限公司
 * *****************************************************************************
 */
#include "timer.h"
#include "key.h"
#include "motor.h"
#include "uart.h"
#include "motor_pmsm.h"

uint32_t systick;
/**
 * @brief   使用TIM1输出三路互补的PWM，带死区和刹车控制，使用TIM1通道4进行ADC触发转换    
 * 
 */
void timer1_init(void)
{		 					 
	// 结构体声明
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRStructure;

	// 时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM1时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//使能PORTA时钟	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//使能PORTB时钟
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//使能PORTE时钟	
	// IO复用
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_TIM1);
	//GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1);	
	// IO配置
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_8 | GPIO_Pin_9 |GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //初始化PA
	// IO配置
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);  	
	//初始化PE
//		GPIO_InitStructure.GPIO_Pin =GPIO_Pin_14;
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
//		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
//		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
//		GPIO_Init(GPIOE,&GPIO_InitStructure);              //初始化PE	
	//刹车配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// 定时器配置： 自动重装载值：5250 ，时钟2倍频=84M*2=168Mhz ,计数频率 = 168000000 /(5250 * 2) = 16Khz 
	TIM_TimeBaseStructure.TIM_Prescaler=0;  
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_CenterAligned1; //中央对齐模式1
	TIM_TimeBaseStructure.TIM_Period=PWM_PERIOD;   //预装载
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV2;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;//重复计数器值
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);//初始化定时器1
	// 定时器比较输出通道配置
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM2; 
	TIM_OCInitStructure.TIM_Pulse=0; 	//占空比：0/5250*100%=0%
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;  //输出极性高
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable; //输出使能
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset; // 输出空闲电平低
	// 互补输出配置
	TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Enable; //互补输出使能
	TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCNPolarity_High;	// 互补端输出极性高
	TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCNIdleState_Reset; //互补输出空闲电平低

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  // OC1
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);  // OC2
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);  // OC3

	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);   //输出比较寄存器预装载
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);   //输出比较寄存器预装载
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);   //输出比较寄存器预装载

	//TIM1_Channel4用于触发ADC采样
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_Pulse=4; //PWM_PERIOD-4;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;  //输出极性高
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OC4Init(TIM1, &TIM_OCInitStructure); 
	TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);   //输出比较寄存器预装载
	// 刹车死区配置
	TIM_BDTRStructure.TIM_OSSIState=TIM_OSSIState_Enable; //设置在运行模式下非工作状态选项
	TIM_BDTRStructure.TIM_OSSRState=TIM_OSSRState_Enable; //设置在运行模式下非工作状态选项 
	TIM_BDTRStructure.TIM_LOCKLevel=TIM_LOCKLevel_1;// 锁电平参数
	TIM_BDTRStructure.TIM_DeadTime=(DEADTIME); //
	TIM_BDTRStructure.TIM_Break=TIM_Break_Disable;//失能刹车输入
	TIM_BDTRStructure.TIM_BreakPolarity=TIM_BreakPolarity_Low; //刹车输入管脚极性低 
	TIM_BDTRStructure.TIM_AutomaticOutput=TIM_AutomaticOutput_Disable;// 自动输出功能关闭
	TIM_BDTRConfig(TIM1,&TIM_BDTRStructure);

	TIM_GenerateEvent(TIM1, TIM_EventSource_Update); // 手动软件触发一个定时器更新事件
	// Clear Update Flag
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);
	
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
} 

/**
 * @brief   系统定时器，定时周期1ms,定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
            Ft=定时器工作频率,单位:Mhz
 * 
 * @param   自动重装值
 * @param   时钟预分频数
 */
void timer7_init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);  ///使能TIM7时钟

	TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 

	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStructure);//初始化TIM7

	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE); //允许定时器7更新中断
	TIM_Cmd(TIM7,ENABLE); //使能定时器7

	NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn; //定时器7中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x03; //抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void timer_init(void)
{
	timer1_init();
	timer7_init(5,8400-1);//定时器时钟84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数10次为1ms
}

/**
 * @brief   定时器7中断服务函数 500us   
 * 
 */
void TIM7_IRQHandler(void)
{
	static uint8_t timer;

	if(TIM_GetITStatus(TIM7,TIM_IT_Update)==SET) //溢出中断
	{
		timer++;
		if(timer>=2)//1ms计数器
		{
			timer =0;
			systick++;
		}
		
		MCL_MediumFrequency_Task();
	}
	TIM_ClearITPendingBit(TIM7,TIM_IT_Update);  //清除中断标志位
}
/**
 * @brief   获取系统时间  
 * 
 */
uint32_t Get_Systick_Timer(void)
{
	return systick;
}





#include "motor.h"
#include "timer.h"
#include "key.h"
#include "gpio.h"
#include "adc.h"
#include "led.h"
#include "debug.h"
#include "motor_pmsm.h"
#include "foc.h"
#include "pid.h"
#include "at24c02.h"
#include "abs_encoder.h"

Motor_Struct BLDC_Motor;

uint16_t phase_A_offset;//无电流通过时采集到的AD值
uint16_t phase_B_offset;//无电流通过时采集到的AD值
uint16_t phase_C_offset;//无电流通过时采集到的AD值
uint32_t phase_A_offset_temp;
uint32_t phase_B_offset_temp;
uint32_t phase_C_offset_temp;	

static void Motor_ClosePwmOutput(void);
static void Motor_OpenPwmOutput(void);
static void SVPWM_CurrentReadingCalibration(void);
static void Motor_Reset(void);
static void Motor_Start(void);
static void Motor_SpeedCtrl(void);
static void Motor_TorqueCtrl(void);
/**
 * @brief       定时器触发ADC采样
 * 
 * @param       void     
 * @return      void
 */
static void Adc_TimerTriggerCurrentCalibration(void)
{
	ADC_InjectedSequencerLengthConfig(ADC1,3);
	ADC_InjectedChannelConfig(ADC1,M1_PHASE_A_ADC_CHANNEL,1,ADC_SampleTime_3Cycles);//注入通道1对应A相电流
	ADC_InjectedChannelConfig(ADC1,M1_PHASE_B_ADC_CHANNEL,2,ADC_SampleTime_3Cycles);//注入通道2对应B相电流
	ADC_InjectedChannelConfig(ADC1,M1_PHASE_C_ADC_CHANNEL,3,ADC_SampleTime_3Cycles);//注入通道3对应C相电流	
	ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_CC4); //trgo 触发 
	ADC_ExternalTrigInjectedConvEdgeConfig(ADC1,ADC_ExternalTrigInjecConvEdge_Rising);//上升沿采样
	ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
}
/**
 * @brief       关闭PWM输出
 * 
 * @param       void     
 * @return      void
 */
static void Motor_ClosePwmOutput(void)
{
	GPIO_ResetBits(PM1_CTRL_SD_GPIO_Port,PM1_CTRL_SD_Pin);//关闭mosfet输出
//	TIM_SetCounter(TIM1,0);
//	TIM_CtrlPWMOutputs(TIM1, DISABLE);
//	TIM_Cmd(TIM1, DISABLE);	
}
/**
 * @brief       打开PWM输出
 * 
 * @param       void     
 * @return      void
 */
static void Motor_OpenPwmOutput(void)
{
	GPIO_SetBits(PM1_CTRL_SD_GPIO_Port,PM1_CTRL_SD_Pin);//使能mosfet输出	
}

/**
 * @brief       三相电流静态偏置采样静态初始化
 * 
 * @param       void     
 * @return      void
 */
void SVPWM_CurrentReadingCalibration(void)
{
	static u16 bIndex;
	ADC_ITConfig(ADC1, ADC_IT_JEOC, DISABLE);//关闭注入组中断
	phase_A_offset =0;//无电流通过时采集到的AD值
	phase_B_offset =0;//无电流通过时采集到的AD值
	phase_C_offset =0;//无电流通过时采集到的AD值
	phase_A_offset_temp=0;
	phase_B_offset_temp=0;
	phase_C_offset_temp=0;	
	ADC_ExternalTrigInjectedConvEdgeConfig(ADC1,ADC_ExternalTrigInjecConvEdge_None);//禁止外部触发
	ADC_InjectedSequencerLengthConfig(ADC1,3);
	ADC_InjectedChannelConfig(ADC1,M1_PHASE_A_ADC_CHANNEL,1,ADC_SampleTime_112Cycles);//注入通道1对应A相电流
	ADC_InjectedChannelConfig(ADC1,M1_PHASE_B_ADC_CHANNEL,2,ADC_SampleTime_112Cycles);//注入通道2对应B相电流
	ADC_InjectedChannelConfig(ADC1,M1_PHASE_C_ADC_CHANNEL,3,ADC_SampleTime_112Cycles);//注入通道3对应C相电流
	ADC_Cmd(ADC1, ENABLE);
	ADC_ClearFlag(ADC1, ADC_FLAG_JEOC); //清除注入转换完成标志位 

    ADC_SoftwareStartInjectedConv(ADC1);//软件启动注入通道转换 
   
  /* ADC Channel used for current reading are read 
     in order to get zero currents ADC values*/ 
  for(bIndex=0; bIndex <NB_CONVERSIONS; bIndex++)//
  {
    while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_JEOC)) { }

	//读取采集结果
	phase_A_offset_temp += ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_1);
	phase_B_offset_temp += ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_2);
	phase_C_offset_temp += ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_3);
		    
    /* Clear the ADC1 JEOC pending flag */
    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);    
    ADC_SoftwareStartInjectedConv(ADC1);//软件启动注入通道转换 
  }	

	phase_A_offset = phase_A_offset_temp >> 4;
	phase_B_offset = phase_B_offset_temp >> 4;
	phase_C_offset = phase_C_offset_temp >> 4;//右移4位  

    Adc_TimerTriggerCurrentCalibration();
}
/**
 * @brief       电机复位
 * 
 * @param       void     
 * @return      void
 */
static void Motor_Reset(void)
{	
	PID_Reset();
	FOC_Reset();                           //FOC参数复位
	ABS_Encoder_Reset();                   //编码器复位
	LED1_OFF();                             	
	Motor_ClosePwmOutput();		
	BLDC_Motor.Motor_State_Prv = BLDC_Motor.Motor_State;
	BLDC_Motor.Motor_State = MOTOR_WAIT;	
}
/**
 * @brief       电机开启运行
 * 
 * @param       void     
 * @return      void
 */
static void Motor_Start(void)
{	
	ABS_Encoder_DMA_SendData(CF_DATA_ID_0,1); //读取偏移角 
	SVPWM_CurrentReadingCalibration();    //三相电流偏置采集
	Motor_OpenPwmOutput();	              //打开PWM输出
	PID_Reset();
	ABS_Encoder.Direction = CCW;
	FOC_Input.speed_ref = -1000.0;
	ABS_Encoder.Current_Capture = ABS_Encoder.Encoder_Value;
	ABS_Encoder.Previous_Capture = ABS_Encoder.Encoder_Value;
}
/**
 * @brief       电机速度模式
 * 
 * @param       void     
 * @return      void
 */
static void Motor_SpeedCtrl(void)
{
	
	Speed_PID_Calc(FOC_Input.speed_ref,ABS_Encoder.AvrMecSpeed,&Speed_InitStructure,&(FOC_Input.Iq_ref));
	FOC_Input.Id_ref= 0.0f;
}
/**
 * @brief       电机力矩模式
 * 
 * @param       void     
 * @return      void
 */
static void Motor_TorqueCtrl(void)
{
	if(ABS_Encoder.Direction == CW)
	{
		FOC_Input.Iq_ref = 1.0f;
		FOC_Input.Id_ref= 0.0f;		
	}
	else
	{
		FOC_Input.Iq_ref = -1.0f;
		FOC_Input.Id_ref= 0.0f;		
	}
}
/**
 * @brief       编码器电机运行状态机
 * 
 * @param       void     
 * @return      void
 */
void MCL_MediumFrequency_Task(void)
{
	Get_Vbus_Voltage(&(FOC_Input.Udc));   //获取母线电压
	if(BLDC_Motor.Motor_State == MOTOR_RUN)
	{
		ABS_Encoder_CalcAvrgMecSpeed(); //获取平均转速
	}
	switch (BLDC_Motor.Motor_State)//电机运行状态机
	{
		case MOTOR_IDLE:                      // Idle 状态 

			if(button[0].buttonAction == BUTTON_SINGLE)//启动按键按下
			{
				button[0].buttonAction = BUTTON_NULL;
				if(BLDC_Motor.Sensor_Type == ENCODER_SENSOR_MODE)//编码器模式下，板子首次上电，需要做零位校准
				{
					if(BLDC_Motor.Motor_Init_State == MOTOR_INIT_ENCODER_ALIGNMENT_NO_START)//转子位置没有对齐
					{
						BLDC_Motor.Motor_State = MOTOR_INIT; //进行转子零位对齐
					}
					else
					{
						BLDC_Motor.Motor_State = MOTOR_START;//对齐之后，直接角度闭环启动
					}	
				}
			}
		break;	
		case MOTOR_INIT://INIT 状态:板子初始上电转子位置对齐                 	
			if(BLDC_Motor.Sensor_Type == ENCODER_SENSOR_MODE)//编码器模式
			{
				if(BLDC_Motor.Motor_Init_State == MOTOR_INIT_ENCODER_ALIGNMENT_NO_START)//转子零位对齐
				{
					Motor_Start();	
					BLDC_Motor.Motor_Init_State = MOTOR_INIT_ENCODER_ALIGNMENT_START;
				}					
				else if(BLDC_Motor.Motor_Init_State == MOTOR_INIT_ENCODER_ALIGNMENT_END)//对齐结束,返回IDLE状态
				{
					BLDC_Motor.Motor_State = MOTOR_STOP;  
				}	
			}
			break;
			case MOTOR_START:                     //START 状态转到RUN 状态 
						Motor_Start(); 
						BLDC_Motor.Motor_State_Prv = BLDC_Motor.Motor_State;		
						BLDC_Motor.Motor_State = MOTOR_RUN;				
			break;
			case MOTOR_RUN: 

				if(BLDC_Motor.Motor_Control_Mode == MOTOR_SPEED_MODE) //转速模式
				{						
					Motor_SpeedCtrl();
				}
				else if(BLDC_Motor.Motor_Control_Mode == MOTOR_TORQUE_MODE)//力矩模式下
				{					
					Motor_TorqueCtrl();                         // Iq_ref = FOC_Structure.Torque_Reference
				}             
				if(button[0].buttonAction == BUTTON_SINGLE)//有按键按下
				{
					button[0].buttonAction = BUTTON_NULL;
					BLDC_Motor.Motor_State_Prv = BLDC_Motor.Motor_State;
					BLDC_Motor.Motor_State = MOTOR_STOP;
				}
				LED1_ON(); 
			break;
			case MOTOR_STOP:   
				Motor_Reset();
			break;
			case MOTOR_WAIT:                               	
				BLDC_Motor.Motor_State_Prv = BLDC_Motor.Motor_State;
				BLDC_Motor.Motor_State = MOTOR_IDLE; 
				break;   														
	}
}

/**
 * @brief       高频任务，FOC计算
 * 
 * @param       void     
 * @return      void
 */
void MCL_HighFrequency_Task(void)
{
		FOC_Algorithm();
}

/**
 * @brief       电机参数初始化
 * 
 * @param       void     
 * @return      void
 */
void Motor_Init(void)
{
	BLDC_Motor.Motor_Control_Mode = MOTOR_SPEED_MODE;
	BLDC_Motor.Sensor_Type = ENCODER_SENSOR_MODE;
	
//	if(BLDC_Motor.Sensor_Type == ENCODER_SENSOR_MODE)//编码器模式
//	{
//		if(ABS_Encoder.Valid_Flag == VALID_VALUE)//编码器已经校验完成
//		{
//			BLDC_Motor.Motor_Init_State = MOTOR_INIT_ENCODER_ALIGNMENT_END;
//		}
//		else
//		{
//			BLDC_Motor.Motor_Init_State = MOTOR_INIT_ENCODER_ALIGNMENT_NO_START;
//		}	
//	}
	BLDC_Motor.Motor_State = MOTOR_IDLE;
}



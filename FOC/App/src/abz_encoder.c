/**
 * *****************************************************************************
 * @file        encoder.c
 * @brief       编码器模块
 * @author       (caoyang)
 * @date        2023-11-13
 * @copyright   欢阳技有限公司
 * *****************************************************************************
 */

#include "abz_encoder.h"
#include "foc.h"
#include "pid.h"
#include "motor.h"
#include "arm_math.h"

ABZ_ENCODER_Handle_t ABZ_Encoder;
uint8_t buf_count_flag;
uint8_t buf_count;
/**
 * @brief       增量式编码器初始化
 * @param       void
 * @return      void
 */
void ABZ_Encoder_Init( void )
{
    uint8_t BufferSize;
    uint8_t Index;
	
    TIM_Cmd(TIM3, ENABLE);
	
    ABZ_Encoder.PulseNumber = M1_ENCODER_PPR * 4.0f;//一圈的脉冲数 
    ABZ_Encoder.SpeedBufferSize = ENC_AVERAGING_FIFO_DEPTH;
    BufferSize =ABZ_Encoder.SpeedBufferSize;
    /* Erase speed buffer */
    for ( Index = 0u; Index < BufferSize; Index++ )
    {
    ABZ_Encoder.DeltaCapturesBuffer[Index] = 0;
    }
    ABZ_Encoder.TimerOverflowNb	= 0;
    buf_count_flag =0;
    buf_count =0;
}
/**
 * @brief       定时器计数值复位
 * @param       void
 * @return      void
 */
void Reset_ABZ_Encoder(void)
{
  TIM3->CNT = 0;
}

/**
 * @brief       获取电机旋转方向
 * @param       TIMx 定时器编号
 * @return      uint32_t  旋转方向
 */
uint32_t TIM_GetDirection(TIM_TypeDef *TIMx)
{
  return (uint32_t)(READ_BIT(TIMx->CR1, TIM_CR1_DIR));
}
/**
 * @brief       增量式编码器计算电机电角度
 * @param       void
 * @return      uint32_t  电角度
 */

float ABZ_Encoder_CalcAngle( void )
{
		int32_t wtemp1;
		int16_t htemp1;
		float E_angle;
	
    wtemp1 = ( int32_t )(( TIM_GetCounter(TIM3))*65535)/ABZ_Encoder.PulseNumber* MOTOR_PP;//电角度= 脉冲计数 *（360°/一圈总脉冲数）*极对数
    htemp1 = (int16_t)wtemp1;
	
    htemp1 += ABZ_Encoder.InitPhaseShift;//加上零点位置偏移
    
		E_angle = ((float)htemp1 / 32768) * PI;
	
		ABZ_Encoder.EAngle = E_angle;
	
    return ( E_angle );
	                                     
}

/**
 * @brief       增量式编码器零点校准
 * @param       void
 * @return      uint32_t  电角度
 */
int16_t ABZ_Encoder_Zero_Calibration( void )
{
  int32_t wtemp1;
  int16_t htemp1;
  
    wtemp1 = ( int32_t )(( TIM_GetCounter(TIM3))*65535)/ABZ_Encoder.PulseNumber* MOTOR_PP;//电角度= 脉冲计数 *（360°/一圈总脉冲数）*极对数
    htemp1 = (int16_t)wtemp1;
    
    return ( htemp1 );
}
/**
 * @brief       M法计算平均机械转速
 * @param       void 
 * @return      uint32_t  机械转速
 */
void ABZ_Encoder_CalcAvrgMecSpeed(void)
{
  uint8_t bBufferIndex = 0u;
  uint32_t OverflowCntSample;
  //获取当前脉冲计数
  ABZ_Encoder.Current_CntCapture = TIM_GetCounter(TIM3);
  //获取计数方向，顺时针为递增，逆时针时递减
  ABZ_Encoder.directionSample =  TIM_GetDirection( TIM3);
  if ( ABZ_Encoder.directionSample== TIM_COUNTERDIRECTION_DOWN )//递减计数
  {
		//看当前捕获是否大于前一次捕获的脉冲数，当前捕获值大于前一次捕获，那么应该是发生了溢出，当前捕获应该是开始了新的一圈
    OverflowCntSample = (ABZ_Encoder.Current_CntCapture > ABZ_Encoder.PreviousCapture ) ? 1 : 0; 
		//计算两次捕获的脉冲的差，求两次捕获的间隔
		//间隔脉冲数 = 当前捕获脉冲数 - 前一次的捕获的脉冲数 - 溢出次数*4000
    ABZ_Encoder.DeltaCapturesBuffer[ENC_AVERAGING_FIFO_DEPTH] =
    (int32_t)(ABZ_Encoder.Current_CntCapture)-(int32_t)(ABZ_Encoder.PreviousCapture)-((int32_t)(OverflowCntSample))*(int32_t)(ABZ_Encoder.PulseNumber);       
  }
  else //如果是递增计数
  {
 		//如果当前捕获小于前一次捕获，说明发生了溢出
    OverflowCntSample = ( ABZ_Encoder.Current_CntCapture < ABZ_Encoder.PreviousCapture ) ? 1 : 0; 
		//计算两次捕获的脉冲计数的差，求间隔
		//间隔脉冲数 = 当前捕获脉冲数 - 前一次的捕获脉冲数 + 溢出次数*4000
    ABZ_Encoder.DeltaCapturesBuffer[ENC_AVERAGING_FIFO_DEPTH] =
    (int32_t)(ABZ_Encoder.Current_CntCapture)-(int32_t)(ABZ_Encoder.PreviousCapture)+((int32_t)(OverflowCntSample))*(int32_t)(ABZ_Encoder.PulseNumber);      
  }
  //更新脉冲计数
  ABZ_Encoder.PreviousCapture = ABZ_Encoder.Current_CntCapture;
  //滑动平均滤波
  for ( bBufferIndex = 0u; bBufferIndex < ENC_AVERAGING_FIFO_DEPTH; bBufferIndex++ )
  {
	  ABZ_Encoder.DeltaCapturesBuffer[bBufferIndex] = ABZ_Encoder.DeltaCapturesBuffer[bBufferIndex+1];
    ABZ_Encoder.wOverallAngleVariation += ABZ_Encoder.DeltaCapturesBuffer[bBufferIndex];
  } 
  //刚开始缓存：计算平均值
	if(buf_count_flag == 0)
  {
		buf_count++;
		ABZ_Encoder.ENC_Period_Avg = ABZ_Encoder.wOverallAngleVariation/buf_count;
		ABZ_Encoder.wOverallAngleVariation = 0;
	}
  //buff缓存满：计算平均值
	else if(buf_count_flag == 1)
  {
		ABZ_Encoder.ENC_Period_Avg = ABZ_Encoder.wOverallAngleVariation/ENC_AVERAGING_FIFO_DEPTH;// 2K采样率下，脉冲计数
		ABZ_Encoder.wOverallAngleVariation = 0;
		
	}
	if(buf_count >= ENC_AVERAGING_FIFO_DEPTH )//buff缓存满标志位
  {
		buf_count_flag = 1;
	}
	//rpm
	ABZ_Encoder.hAvrMecSpeed =  (float)(60.0f*MEDIUM_FREQUENCY_TASK_RATE*ABZ_Encoder.ENC_Period_Avg)/(ABZ_Encoder.PulseNumber);//（60*f*脉冲计数）/总的脉冲数
}
/**
 * @brief       编码器启动
 * @param       void 
 * @return      void
 */
void ABZ_Encoder_Start_Up(void)
{
  static uint32_t wTimebase=0;
	
	wTimebase++;

  if(wTimebase <= T_ALIGNMENT_PWM_STEPS)//开始对齐
  {
        FOC_Input.Id_ref = I_ALIGNMENT * wTimebase / T_ALIGNMENT_PWM_STEPS;
        FOC_Input.Iq_ref = 0;       
        FOC_Input.theta = ALIGNMENT_ANGLE;//对齐电角度
        Get_Vbus_Voltage(&(FOC_Input.Udc));   //获取母线电压
        Get_PhaseCurrentValues(&Current_Iabc);//相电流采样
        Clarke_Transf(Current_Iabc,&Current_Ialpha_beta);        //CLARK 变换
        Angle_To_Cos_Sin(FOC_Input.theta,&Transf_Cos_Sin);     //由角度计算 park变换和 反park变换的 COS SIN值
        Park_Transf(Current_Ialpha_beta,Transf_Cos_Sin,&Current_Idq);  //Park变换，由Ialpha Ibeta 与角度信息，去计算Id Iq  // 由交流信息转化为直流信息，方便PID控制 
        Current_PID_Calc(FOC_Input.Id_ref,Current_Idq.Id,&ID_InitStructure,&Voltage_DQ.Vd);     //D轴电流环PID  根据电流参考与电流反馈去计算 输出电压
        Current_PID_Calc(FOC_Input.Iq_ref,Current_Idq.Iq,&IQ_InitStructure,&Voltage_DQ.Vq);     //Q轴电流环PID  根据电流参考与电流反馈去计算 输出电压
        Rev_Park_Transf(Voltage_DQ,Transf_Cos_Sin,&Voltage_Alpha_Beta);                //反park变换  通过电流环得到的dq轴电压信息结合角度信息，去把直流信息转化为交流信息用于SVPWM的输入
        SVPWM_Calc(Voltage_Alpha_Beta,FOC_Input.Udc,TS);       //SVPWM 计算模块  
  }
  else//对齐结束
  {
      BLDC_Motor.Motor_Init_State = MOTOR_INIT_ENCODER_ALIGNMENT_END;  	
	}
}






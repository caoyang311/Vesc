#include "foc.h"
#include "arm_math.h"
#include "motor_pmsm.h"
#include "abs_encoder.h"
#include "motor.h"
#include "adc.h"
#include "debug.h"
#include "pid.h"

FOC_OUTPUT_DEF FOC_Output;
FOC_INPUT_DEF FOC_Input;
CURRENT_ABC_DEF Current_Iabc;
CURRENT_ALPHA_BETA_DEF Current_Ialpha_beta;
VOLTAGE_ALPHA_BETA_DEF Voltage_Alpha_Beta;
TRANSF_COS_SIN_DEF Transf_Cos_Sin;
CURRENT_DQ_DEF Current_Idq; 
VOLTAGE_DQ_DEF Voltage_DQ;
uint8_t sector;
/**
 * @brief       频率转换成角度
 * 
 * @param       void     
 * @return      void
 */ 
void FreqGenerator(real32_T Freq,real32_T *Theta)
{
  *Theta +=  0.00039269909F* Freq;
	
  if (*Theta > 3.14159274F)
	{
    *Theta -= 6.28318548F;
  } 
	else if (*Theta < -3.14159274F)
	{
		*Theta += 6.28318548F;
  }
}

/***************************************
功能：Clark变换
形参：三相电流以及alpha_beta电流
说明：由三相互差120度变换到两相互差90度
***************************************/
void Clarke_Transf(CURRENT_ABC_DEF Current_abc_temp,CURRENT_ALPHA_BETA_DEF* Current_alpha_beta_temp)
{
  Current_alpha_beta_temp->Ialpha = (Current_abc_temp.Ia - (Current_abc_temp.Ib + Current_abc_temp.Ic) * 0.5F) * 2.0F / 3.0F;
  Current_alpha_beta_temp->Ibeta = (Current_abc_temp.Ib - Current_abc_temp.Ic) * 0.866025388F * 2.0F / 3.0F;
}

/***************************************
功能：PARK变换
形参：alpha_beta电流、COS_SIN值、DQ轴电流
说明：交流变直流
***************************************/
void Park_Transf(CURRENT_ALPHA_BETA_DEF current_alpha_beta_temp,TRANSF_COS_SIN_DEF cos_sin_temp,CURRENT_DQ_DEF* current_dq_temp)
{
  current_dq_temp->Id = current_alpha_beta_temp.Ialpha * cos_sin_temp.Cos + current_alpha_beta_temp.Ibeta * cos_sin_temp.Sin;
  current_dq_temp->Iq = -current_alpha_beta_temp.Ialpha * cos_sin_temp.Sin + current_alpha_beta_temp.Ibeta * cos_sin_temp.Cos;
}
/***************************************
功能：反PARK变换
形参：DQ轴电压、COS_SIN值、alpha_beta电压
说明：直流变交流
***************************************/
void Rev_Park_Transf(VOLTAGE_DQ_DEF v_dq_temp,TRANSF_COS_SIN_DEF cos_sin_temp,VOLTAGE_ALPHA_BETA_DEF* v_alpha_beta_temp)
{
  v_alpha_beta_temp->Valpha = cos_sin_temp.Cos * v_dq_temp.Vd - cos_sin_temp.Sin * v_dq_temp.Vq;
  v_alpha_beta_temp->Vbeta  = cos_sin_temp.Sin * v_dq_temp.Vd + cos_sin_temp.Cos * v_dq_temp.Vq;
}

/***************************************
功能：COS_SIN值计算
形参：角度以及COS_SIN结构体
说明：COS_SIN值计算
***************************************/
void Angle_To_Cos_Sin(real32_T angle_temp,TRANSF_COS_SIN_DEF* cos_sin_temp)
{
  cos_sin_temp->Cos = arm_cos_f32(angle_temp);
  cos_sin_temp->Sin = arm_sin_f32(angle_temp);
}

/***************************************
功能：SVPWM计算
形参：alpha_beta电压以及母线电压、定时器周期
说明：根据alpha_beta电压计算三相占空比
***************************************/
void SVPWM_Calc(VOLTAGE_ALPHA_BETA_DEF v_alpha_beta_temp,real32_T Udc_temp,real32_T Tpwm_temp)
{
  int32_T N =0;
  real32_T Tcmp1,Tcmp2,Tcmp3,Tx,Ty,f_temp,Ta,Tb,Tc;

  Tcmp1 = 0.0F;
  Tcmp2 = 0.0F;
  Tcmp3 = 0.0F;
  if (v_alpha_beta_temp.Vbeta > 0.0F) {
    N = 1;
  }
  
  if ((1.73205078F * v_alpha_beta_temp.Valpha - v_alpha_beta_temp.Vbeta) / 2.0F > 0.0F) {
    N += 2;
  }
  
  if ((-1.73205078F * v_alpha_beta_temp.Valpha - v_alpha_beta_temp.Vbeta) / 2.0F > 0.0F) {
    N += 4;
  }
  switch(N){

    case 1:
    sector = SECTOR_2;      
        break;
    
    case 2:
    sector = SECTOR_6;
        break;
    
    case 3:
    sector = SECTOR_1;
            
        break;
    
    case 4:
    sector = SECTOR_4;
        break;
    
    case 5:
    sector = SECTOR_3;
        break;
    
    case 6:
    sector = SECTOR_5;
        break;

    default:
        break;
}

  switch (N) {
  case 1:
    Tx = (-1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);
    Ty = (1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);
    break;
    
  case 2:
    Tx = (1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);
    Ty = -(1.73205078F * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp);
    break;
    
  case 3:
    Tx = -((-1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));
    Ty = 1.73205078F * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp;
    break;
    
  case 4:
    Tx = -(1.73205078F * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp);
    Ty = (-1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);
    break;
    
  case 5:
    Tx = 1.73205078F * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp;
    Ty = -((1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));
    break;
    
  default:
    Tx = -((1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));
    Ty = -((-1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));
    break;
  }
  
  f_temp = Tx + Ty;
  if (f_temp > Tpwm_temp) {
    Tx /= f_temp;
    Ty /= (Tx + Ty);
  }
  
  Ta = (Tpwm_temp - (Tx + Ty)) / 4.0F;
  Tb = Tx / 2.0F + Ta;
  Tc = Ty / 2.0F + Tb;
  switch (N) {
  case 1:
    Tcmp1 = Tb;
    Tcmp2 = Ta;
    Tcmp3 = Tc;
    break;
    
  case 2:
    Tcmp1 = Ta;
    Tcmp2 = Tc;
    Tcmp3 = Tb;
    break;
    
  case 3:
    Tcmp1 = Ta;
    Tcmp2 = Tb;
    Tcmp3 = Tc;
    break;
    
  case 4:
    Tcmp1 = Tc;
    Tcmp2 = Tb;
    Tcmp3 = Ta;
    break;
    
  case 5:
    Tcmp1 = Tc;
    Tcmp2 = Ta;
    Tcmp3 = Tb;
    break;
    
  case 6:
    Tcmp1 = Tb;
    Tcmp2 = Tc;
    Tcmp3 = Ta;
    break;
  }
  
  FOC_Output.Tcmp1 = Tcmp1;
  FOC_Output.Tcmp2 = Tcmp2;
  FOC_Output.Tcmp3 = Tcmp3;
	
	TIM1->CCR1= FOC_Output.Tcmp1;
	TIM1->CCR2= FOC_Output.Tcmp2;
	TIM1->CCR3= FOC_Output.Tcmp3;
}

/***************************************
功能：相电流采样
形参：void
说明：根据ADC采样值换算出真实电流值，单位：安培（A）
***************************************/
void Get_PhaseCurrentValues(CURRENT_ABC_DEF *Current_Iabc)
{

  int16_t ia_temp=0,ib_temp=0,ic_temp=0;

  switch (sector)
   {
		 //
   case 4:  //读A B 相电流
   case 5: //Current on Phase C not accessible     
						
    ia_temp = (phase_A_offset - (ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_1)));
    ib_temp	= (phase_B_offset - (ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_2)));
    ic_temp = (phase_C_offset - (ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_3)));//Ic
            
    break;
           
   case 6:
   case 1:  //  读BC
						
    ib_temp = (phase_B_offset-(ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_2)));//Ib
    ic_temp = (phase_C_offset-(ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_3)));//Ic
    ia_temp = (-ib_temp-ic_temp);//Ia = -Ib-Ic				

    break;
           
   case 2:
   case 3:  //读AC
		 		
   ia_temp = (phase_A_offset-(ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_1)));//Ia
   ic_temp = (phase_C_offset-(ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_3)));//Ic
   ib_temp = (-ia_temp - ic_temp) ;//Ib=-Ia-Ic

    break;		

   default:
           break;
   } 
	
   Current_Iabc->Ia = (float)ia_temp*ADC_TO_CurrentAmp;  //通过电流转换因子（通过采样电阻和运算放大倍数得到）把adc采样值转化为真实电流值
   Current_Iabc->Ib = (float)ib_temp*ADC_TO_CurrentAmp;  //通过电流转换因子（通过采样电阻和运算放大倍数得到）把adc采样值转化为真实电流值
   Current_Iabc->Ic = (float)ic_temp*ADC_TO_CurrentAmp;  //通过电流转换因子（通过采样电阻和运算放大倍数得到）把adc采样值转化为真实电流值

}

/***************************************
功能：母线电压采样
形参：void
说明：根据ADC采样值换算出真实母线电压，单位：伏特（V）
***************************************/
void Get_Vbus_Voltage(real32_T *V_bus)
{
    *V_bus = (real32_T)ADCConvertedValue[1]*VBUS_CONVERSION_FACTOR;
}
/***************************************
功能：FOC核心算法
形参：void
说明：void
***************************************/
void FOC_Algorithm(void)
{ 
		//FreqGenerator(10.0F,&(FOC_Input.theta));//角度生成器
	  FOC_Input.theta = ABS_Encoder_CalcAngle();//获取电角度
    
    Get_PhaseCurrentValues(&Current_Iabc);//相电流采样
    Clarke_Transf(Current_Iabc,&Current_Ialpha_beta);        //CLARK 变换
    Angle_To_Cos_Sin(FOC_Input.theta,&Transf_Cos_Sin);     //由角度计算 park变换和 反park变换的 COS SIN值
    Park_Transf(Current_Ialpha_beta,Transf_Cos_Sin,&Current_Idq);  //Park变换，由Ialpha Ibeta 与角度信息，去计算Id Iq  // 由交流信息转化为直流信息，方便PID控制 
    Current_PID_Calc(FOC_Input.Id_ref,Current_Idq.Id,&ID_InitStructure,&Voltage_DQ.Vd);     //D轴电流环PID  根据电流参考与电流反馈去计算 输出电压
    Current_PID_Calc(FOC_Input.Iq_ref,Current_Idq.Iq,&IQ_InitStructure,&Voltage_DQ.Vq);     //Q轴电流环PID  根据电流参考与电流反馈去计算 输出电压
    Rev_Park_Transf(Voltage_DQ,Transf_Cos_Sin,&Voltage_Alpha_Beta);                //反park变换  通过电流环得到的dq轴电压信息结合角度信息，去把直流信息转化为交流信息用于SVPWM的输入
    SVPWM_Calc(Voltage_Alpha_Beta,FOC_Input.Udc,TS);       //SVPWM 计算模块
		ABS_Encoder_DMA_SendData(CF_DATA_ID_0,1);//读取绝对值编码器角度
		Uart_SendWave(Current_Iabc.Ia,FOC_Input.speed_ref,ABS_Encoder.AvrMecSpeed);
	// DAC_SetChannel1Data(DAC_Align_12b_R,(u16)(FOC_Input.Id_ref*1000));
	// DAC_SetChannel2Data(DAC_Align_12b_R,(u16)(Current_Idq.Id*1000));
}
/***************************************
功能：FOC初始化
形参：void
说明：void
***************************************/
void FOC_Reset(void)
{
   FOC_Input.speed_ref =0;
}





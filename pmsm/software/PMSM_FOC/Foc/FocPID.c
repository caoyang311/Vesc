#include "FocPID.h"
#include <stddef.h> 


//力矩PID默认值
#define PID_IQ_KP_DEFAULT  ((float)(0.25f))     
#define PID_IQ_KI_DEFAULT  ((float)(600.0f))
#define PID_IQ_KD_DEFAULT  ((float)(0.0f))
//磁链PID默认值
#define PID_ID_KP_DEFAULT  ((float)(0.25f)) 
#define PID_ID_KI_DEFAULT  ((float)(600.0f))
#define PID_ID_KD_DEFAULT  ((float)(0.0f))
//电流PID输出限幅
#define ID_LOWER_LIMIT   ((float)(-32.0f))
#define ID_UPPER_LIMIT   ((float)(32.0f))
#define IQ_LOWER_LIMIT   ((float)(-32.0f))
#define IQ_UPPER_LIMIT   ((float)(32.0f))
//速度环PID
#define PID_SPEED_KP_DEFAULT       ((float)(0.5f))
#define PID_SPEED_KI_DEFAULT       ((float)(500.0f))
#define PID_SPEED_KD_DEFAULT       ((float)(0.0f))
//速度PID输出限幅
#define PID_SPEED_LOWER_LIMIT   ((float)(-10.0f))
#define PID_SPEED_UPPER_LIMIT   ((float)(10.0f))


PID_Struct IQ_InitStructure;
PID_Struct ID_InitStructure; 
PID_Struct Speed_InitStructure;  


/**
  * @brief  力矩PID参数初始化
  *         
  * @param  PID_Torque：力矩PID参数结构体
  * @retval void
  */
void PID_Init_Iq(PID_Struct *PID_Iq)
{
    PID_Iq->Integral = 0; //清除积分累积

	PID_Iq->Kp_Gain = PID_IQ_KP_DEFAULT; //kp默认值
	PID_Iq->Ki_Gain = PID_IQ_KI_DEFAULT; //ki默认值
	PID_Iq->Kd_Gain = PID_IQ_KD_DEFAULT; //kd默认值
	PID_Iq->PreviousError = 0;
	
	PID_Iq->Lower_Limit_Output =   IQ_LOWER_LIMIT;   //PID输出下限
	PID_Iq->Upper_Limit_Output =   IQ_UPPER_LIMIT;   //PID输出上限
	PID_Iq->Lower_Limit_Integral = IQ_LOWER_LIMIT; //积分下限
	PID_Iq->Upper_Limit_Integral = IQ_UPPER_LIMIT; //积分上限
 	
}
/**
  * @brief  磁链PID参数初始化
  *         
  * @param  PID_Torque：磁链PID参数结构体
  * @retval void
  */
void PID_Init_Id(PID_Struct *PID_Id)
{

	PID_Id->Integral = 0;  // 清除积分累积
	PID_Id->Kp_Gain = PID_ID_KP_DEFAULT;//kp默认值
	PID_Id->Ki_Gain = PID_ID_KI_DEFAULT;//ki默认值
	PID_Id->Kd_Gain = PID_ID_KD_DEFAULT;//kd默认值
	PID_Id->PreviousError = 0;//微分误差清零
	PID_Id->Lower_Limit_Output=ID_LOWER_LIMIT; //PID输出下限  
	PID_Id->Upper_Limit_Output=ID_UPPER_LIMIT; //PID输出上限
	PID_Id->Lower_Limit_Integral = ID_LOWER_LIMIT;//积分下限
	PID_Id->Upper_Limit_Integral = ID_UPPER_LIMIT;//积分上限
}
/**
  * @brief  速度PID参数初始化
  *         
  * @param  PID_Torque：速度PID参数结构体
  * @retval void
  */
void PID_Init_Speed(PID_Struct *PID_Speed)
{
  PID_Speed->Integral = 0;  // reset integral value 
  PID_Speed->Kp_Gain    = PID_SPEED_KP_DEFAULT; 
  PID_Speed->Ki_Gain = PID_SPEED_KI_DEFAULT;
  PID_Speed->Kd_Gain = PID_SPEED_KD_DEFAULT;
  PID_Speed->PreviousError = 0;
  PID_Speed->Lower_Limit_Output= PID_SPEED_LOWER_LIMIT;   //Lower Limit for Output limitation
  PID_Speed->Upper_Limit_Output= PID_SPEED_UPPER_LIMIT;   //Upper Limit for Output limitation
  PID_Speed->Lower_Limit_Integral = PID_SPEED_LOWER_LIMIT;
  PID_Speed->Upper_Limit_Integral = PID_SPEED_UPPER_LIMIT;
}
/**
  * @brief  电流PID计算
  *         
  * @param  Reference：参考电流值
  * @param  Feedback：反馈电流值
  * @param  PID_Struct：电流PID参数结构体
  * @param  sample_period_s：采样周期（秒）
  * @param  output：原始PI输出电压指针，限幅在解耦后统一执行
  * @retval void
  */
static float FocPID_LimitIntegral(float integral,
                                  float limit)
{
    float limited_integral = integral;

    if (limited_integral > limit)
    {
        limited_integral = limit;
    }
    else if (limited_integral < -limit)
    {
        limited_integral = -limit;
    }
    else
    {
    }

    return limited_integral;
}
/**
  * @brief  电流PID积分限幅
  *         
  * @param  d_axis_pid：d轴电流PID参数结构体
  * @param  q_axis_pid：q轴电流PID参数结构体
  * @param  voltage_limit：电压限制结构体
  * @retval void
  */
void Current_PID_LimitIntegralDq(
    PID_Struct *d_axis_pid,
    PID_Struct *q_axis_pid,
    const Foc_VoltageLimitType *voltage_limit)
{
    if ((d_axis_pid != NULL) &&
        (q_axis_pid != NULL) &&
        (voltage_limit != NULL) &&
        (voltage_limit->MaxDVoltage >= 0.0F) &&
        (voltage_limit->MaxQVoltage >= 0.0F))
    {
        d_axis_pid->Integral = FocPID_LimitIntegral(
            d_axis_pid->Integral,
            voltage_limit->MaxDVoltage);
        q_axis_pid->Integral = FocPID_LimitIntegral(
            q_axis_pid->Integral,
            voltage_limit->MaxQVoltage);
    }
    else
    {
    }
}
/**
  * @brief  电流PID计算
  *         
  * @param  Reference：参考电流值
  * @param  Feedback：反馈电流值
  * @param  PID_Struct：电流PID参数结构体
  * @param  sample_period_s：采样周期（秒）
  * @param  output：原始PI输出电压指针，限幅在解耦后统一执行
  * @retval void
  */
void Current_PID_Calc(float Reference,
                      float Feedback,
                      PID_Struct *PID_Struct,
                      float sample_period_s,
                      float *output)
{
    float error = 0.0F;
    float integral_candidate = 0.0F;

    if ((PID_Struct != NULL) &&
        (output != NULL) &&
        (sample_period_s > 0.0F))
    {
        error = Reference - Feedback;
        PID_Struct->Error = error;
        integral_candidate = PID_Struct->Integral +
                             (PID_Struct->Ki_Gain * error *
                              sample_period_s);
        PID_Struct->Integral = integral_candidate;
        *output = PID_Struct->Integral +
                  (PID_Struct->Kp_Gain * error);
    }
    else
    {
        if (output != NULL)
        {
            *output = 0.0F;
        }
        else
        {
        }
    }
}


/**
  * @brief  速度PID计算
  *         
  * @param  Reference：参考速度值
  * @param  Feedback：反馈速度值
  * @param  PID_Struct：速度PID参数结构体
  * @param  output：输出速度值指针
  * @retval void
  */
void Speed_PID_Calc(float Reference,
                      float Feedback,PID_Struct *PID_Struct,float *output)
{
 
    float Error, Proportional_Term,Integral_Term;	
    float M1_dwAux; 
#ifdef DIFFERENTIAL_TERM_ENABLED    
    float Differential_Term;
#endif  
      // 误差计算
      Error= (float)(Reference - Feedback);

      // 计算比例项
      Proportional_Term = (PID_Struct->Kp_Gain * Error);

      // 计算积分项
      if (PID_Struct->Ki_Gain == 0.0f)
      {
        PID_Struct->Integral = 0.0f;
      }
      else
      { 
        Integral_Term = (PID_Struct->Ki_Gain * Error);
        M1_dwAux = PID_Struct->Integral + Integral_Term;
        //积分限幅
        if (M1_dwAux > PID_Struct->Upper_Limit_Integral)
        {
          PID_Struct->Integral = PID_Struct->Upper_Limit_Integral;
        }
        else if (M1_dwAux < PID_Struct->Lower_Limit_Integral)
        { 
          PID_Struct->Integral = PID_Struct->Lower_Limit_Integral;
        }
        else
        {
          PID_Struct->Integral =M1_dwAux;
        }
      }
      // 计算微分项
    #ifdef DIFFERENTIAL_TERM_ENABLED
      {
      float temp;
      
      temp = Error - PID_Struct->PreviousError;
      Differential_Term = PID_Struct->Kd_Gain * temp;
      PID_Struct->PreviousError = Error;    // store value 
      }
      *output = (Proportional_Term + PID_Struct->Integral + Differential_Term); 

    #else  
        *output = (Proportional_Term + PID_Struct->Integral); //PID输出
    #endif
      //PID输出限幅
      if (*output >= PID_Struct->Upper_Limit_Output)
      {
        *output = PID_Struct->Upper_Limit_Output;		  			 	
      }
      else if (*output < PID_Struct->Lower_Limit_Output)
      {
       *output = PID_Struct->Lower_Limit_Output;
      }
 
}
/**
  * @brief  PID参数初始化
  *         
  * @param  void
  * @retval void
  */
void PID_init(void)
{
  PID_Init_Iq(&IQ_InitStructure);         //力矩PID参数初始化
  PID_Init_Id(&ID_InitStructure);         //磁链PID参数初始化
  PID_Init_Speed(&Speed_InitStructure);   //速度PID参数初始化
}

/**
  * @brief  PID参数复位
  *         
  * @param  void
  * @retval void
  */
 void PID_Reset(void)
 {
    IQ_InitStructure.Integral =0;
    IQ_InitStructure.PreviousError =0;
    ID_InitStructure.Integral =0;
    ID_InitStructure.PreviousError =0;
    Speed_InitStructure.Integral =0;
    Speed_InitStructure.PreviousError =0;
 }



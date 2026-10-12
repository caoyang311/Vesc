
#ifndef FOC_PID_H
#define FOC_PID_H
/*----------------------------------include-----------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "FocAlgorithm.h"

/*-----------------------------------macro------------------------------------*/
/*----------------------------------typedef-----------------------------------*/
//PID参数结构体
typedef struct PID_Struct
{  
  float Kp_Gain;                //kp
  float Ki_Gain;                //ki
  float Kd_Gain;                //kd
  float Lower_Limit_Output;     //Lower Limit for Output limitation
  float Upper_Limit_Output;     //Lower Limit for Output limitation
  float Lower_Limit_Integral;   //Lower Limit for Integral term limitation
  float Upper_Limit_Integral;   //Lower Limit for Integral term limitation
  float Integral;               //积分累积
  float PreviousError;          //微分误差
  float Error;                  //误差
} PID_Struct;
/*----------------------------------variable----------------------------------*/
extern PID_Struct IQ_InitStructure;
extern PID_Struct ID_InitStructure; 
extern PID_Struct Speed_InitStructure; 
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void PID_init(void);
void PID_Reset(void);
void PID_Init_Iq(PID_Struct *PID_Iq);
void PID_Init_Id(PID_Struct *PID_Id);
void PID_Init_Speed(PID_Struct *PID_Speed);
void Current_PID_Calc(float Reference,
                      float Feedback,
                      PID_Struct *PID_Struct,
                      float sample_period_s,
                      float *output);
void Current_PID_LimitIntegralDq(PID_Struct *d_axis_pid,
                                 PID_Struct *q_axis_pid,
                                 const Foc_VoltageLimitType *voltage_limit);
void Speed_PID_Calc(float Reference, float Feedback,PID_Struct *PID_Struct,float *output);
/*------------------------------------test------------------------------------*/
#endif





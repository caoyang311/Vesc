/**
 * *****************************************************************************
 * @file        PID.h
 * @brief       PID模块
 * @author      caoyang
 * @date        2023-11-13
 * @copyright   欢阳技有限公司
 * *****************************************************************************
 */
#ifndef PID_H
#define PID_H
/*----------------------------------include-----------------------------------*/
#include "stm32f4xx.h"
#include "motor_pmsm.h"
/*-----------------------------------macro------------------------------------*/
//力矩PID默认值
#define PID_IQ_KP_DEFAULT  ((float)CurrentKp)     
#define PID_IQ_KI_DEFAULT  ((float)CurrentKi)
#define PID_IQ_KD_DEFAULT  ((float)(0.0f))
//磁链PID默认值
#define PID_ID_KP_DEFAULT  ((float)CurrentKp) 
#define PID_ID_KI_DEFAULT  ((float)CurrentKi)
#define PID_ID_KD_DEFAULT  ((float)(0.0f))
//电流PID输出限幅
#define ID_LOWER_LIMIT   ((float)CURREN_MIN_TLIMIT_OUT)
#define ID_UPPER_LIMIT   ((float)CURREN_MAX_TLIMIT_OUT)
#define IQ_LOWER_LIMIT   ((float)CURREN_MIN_TLIMIT_OUT)
#define IQ_UPPER_LIMIT   ((float)CURREN_MAX_TLIMIT_OUT)
//速度环PID
#define PID_SPEED_KP_DEFAULT       ((float)SpeedtKp)
#define PID_SPEED_KI_DEFAULT       ((float)SpeedtKi)
#define PID_SPEED_KD_DEFAULT       ((float)(0.0f))
//速度PID输出限幅
#define PID_SPEED_LOWER_LIMIT   ((float)SPEED_MIN_TLIMIT_OUT)
#define PID_SPEED_UPPER_LIMIT   ((float)SPEED_MAX_TLIMIT_OUT)

/*----------------------------------typedef-----------------------------------*/
//PID参数结构体
typedef struct 
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
void Current_PID_Calc(float Reference, float Feedback,PID_Struct *PID_Struct,float *output);
void Speed_PID_Calc(float Reference, float Feedback,PID_Struct *PID_Struct,float *output);
/*------------------------------------test------------------------------------*/
#endif





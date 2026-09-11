/**
 * *****************************************************************************
 * @file        foc.h
 * @brief       foc核心文件
 * @author      caoyang
 * @date        2023-11-13
 * @copyright   欢阳技有限公司
 * *****************************************************************************
 */
#ifndef FOC_H
#define FOC_H

/*----------------------------------include-----------------------------------*/
#include "rtwtypes.h"
/*-----------------------------------macro------------------------------------*/
#define SECTOR_1	(u32)1
#define SECTOR_2	(u32)2
#define SECTOR_3	(u32)3
#define SECTOR_4	(u32)4
#define SECTOR_5	(u32)5
#define SECTOR_6	(u32)6
/*----------------------------------typedef-----------------------------------*/
typedef struct
{
  real32_T Ia;
  real32_T Ib;
  real32_T Ic;
}CURRENT_ABC_DEF;

typedef struct
{
  real32_T Ialpha;
  real32_T Ibeta;
}CURRENT_ALPHA_BETA_DEF;

typedef struct
{
  real32_T Valpha;
  real32_T Vbeta;
}VOLTAGE_ALPHA_BETA_DEF;

typedef struct
{
  real32_T Cos;
  real32_T Sin;
}TRANSF_COS_SIN_DEF;

typedef struct
{
  real32_T Id;
  real32_T Iq;
}CURRENT_DQ_DEF;

typedef struct
{
  real32_T Vd;
  real32_T Vq;
}VOLTAGE_DQ_DEF;

typedef struct 
{
    real32_T Tcmp1;                      
    real32_T Tcmp2;                      
    real32_T Tcmp3;                                       
} FOC_OUTPUT_DEF;

typedef struct {
    real32_T Id_ref;                     
    real32_T Iq_ref;                     
    real32_T speed_ref;                  
    real32_T theta;                                              
    real32_T Udc;                                               
    real32_T Rs;                         
    real32_T Ls;                         
    real32_T flux;                       
  } FOC_INPUT_DEF;
/*----------------------------------variable----------------------------------*/
extern FOC_OUTPUT_DEF FOC_Output;
extern FOC_INPUT_DEF FOC_Input;
extern CURRENT_ABC_DEF Current_Iabc;
extern CURRENT_ALPHA_BETA_DEF Current_Ialpha_beta;
extern VOLTAGE_ALPHA_BETA_DEF Voltage_Alpha_Beta;
extern TRANSF_COS_SIN_DEF Transf_Cos_Sin;
extern CURRENT_DQ_DEF Current_Idq; 
extern VOLTAGE_DQ_DEF Voltage_DQ;
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void FreqGenerator(real32_T Freq,real32_T *Theta);
void FOC_Algorithm(void);
void FOC_Reset(void);
void Clarke_Transf(CURRENT_ABC_DEF Current_abc_temp,CURRENT_ALPHA_BETA_DEF* Current_alpha_beta_temp);
void Park_Transf(CURRENT_ALPHA_BETA_DEF current_alpha_beta_temp,TRANSF_COS_SIN_DEF cos_sin_temp,CURRENT_DQ_DEF* current_dq_temp);
void Rev_Park_Transf(VOLTAGE_DQ_DEF v_dq_temp,TRANSF_COS_SIN_DEF cos_sin_temp,VOLTAGE_ALPHA_BETA_DEF* v_alpha_beta_temp);
void Angle_To_Cos_Sin(real32_T angle_temp,TRANSF_COS_SIN_DEF* cos_sin_temp);
void SVPWM_Calc(VOLTAGE_ALPHA_BETA_DEF v_alpha_beta_temp,real32_T Udc_temp,real32_T Tpwm_temp);
void Get_PhaseCurrentValues(CURRENT_ABC_DEF *Current_Iabc);
void Get_Vbus_Voltage(real32_T *V_bus);
/*------------------------------------test------------------------------------*/


#endif



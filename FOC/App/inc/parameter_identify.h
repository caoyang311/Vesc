/**
 * *****************************************************************************
 * @file        parameter_identify.h
 * @brief       电机参数辨识模块
 * @author      caoyang
 * @date        2023-11-13
 * @copyright   欢阳技有限公司
 * *****************************************************************************
 */
#ifndef PARAMETER_IDENTIFY_H
#define PARAMETER_IDENTIFY_H
/*----------------------------------include-----------------------------------*/
#include "stm32f4xx.h"
#include "motor_pmsm.h"

/*-----------------------------------macro------------------------------------*/
//参数辨识状态机
#define RESISTOR                0
#define D_INDUCTOR              1
#define Q_INDUCTOR              2
#define MAGNETIC_CHAIN          3

#define ST_IDENTIFY

#define FFT_LENGTH              128
/*----------------------------------typedef-----------------------------------*/
//电阻辨识参数结构体
typedef struct 
{  
  float Resistor_Value;               //辨识出来的电阻值
  uint8_t  Resistor_Identify_Status;  //状态机
} Resistor_Struct;

//D轴电感辨识参数结构体
typedef struct 
{  
  float Ld_Value;               //辨识出来的电阻值
  uint8_t  Ld_Identify_Status;  //状态机
} D_Inductor_Struct;
/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void Parameter_Identify_Handle(void);
/*------------------------------------test------------------------------------*/
#endif


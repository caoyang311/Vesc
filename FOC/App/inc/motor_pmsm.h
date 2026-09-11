/**
 * *****************************************************************************
 * @file        motor_pmsm.h
 * @brief       电机参数定义
 * @author      caoyang
 * @date        2023-11-13
 * @copyright   欢阳技有限公司
 * *****************************************************************************
 */
#ifndef MOTOR_PMSM_H
#define MOTOR_PMSM_H
/*----------------------------------include-----------------------------------*/
#include "stm32f4xx.h"
/*-----------------------------------macro------------------------------------*/
//数学表达式定义
#define TWOPI             ((float)(6.283185f))
#define SQRT3             ((float)(1.732050f))     //√3
#define ONE_DIV_SQRT3	  ((float)(0.577350f))     //  1/√3  
#define SQRT3_OV2		  ((float)(0.866025f))     //  √3/2  
//电机PWM定时器相关参数定义
#define SYSCLK_FREQ						(168000000u) //定时器主时钟频率：168M
#define PWM_FREQUENCY					(16000)     //PWM 频率：20Khz	周期：50us
#define PWM_PRSC                        (0u)         //预分频 ：0                         
#define PWM_PERIOD ((u16) (SYSCLK_FREQ / (u32)(2 * PWM_FREQUENCY *(PWM_PRSC+1)))) // 预装载值：4200
#define TS                              ((u16)2*(PWM_PERIOD))                     //周期计数值：8400
//电机PWM死区时间定义
#define PWM_DEAD_TIME_NS			(500u) //死区时间：1000 单位 ns    range is [0...1500] 
#define DEADTIME  (u16)((unsigned long long)SYSCLK_FREQ/2 *(unsigned long long)PWM_DEAD_TIME_NS/1000000000uL) 
//编码器定时器相关参数定义
#define ENCODER_MAX_VALUE                      (131071)                     /*17位绝对值编码器最大计数值 */

//电机基本参数

#define MOTOR_PP   ((float)5.0f)                             //极对数
#define MOTOR_RS   ((float)135.0f)                           //单位：mΩ 相电阻
#define MOTOR_LD   ((float)280.0f)                           //单位：uH D轴电感
#define MOTOR_LQ   ((float)280.0f)                           //单位：uH Q轴电感
#define MOTOR_KE   ((float)8.0f)                             //单位： V/KRPM 反电动势常数
#define MOTOR_FLUX ((float)((MOTOR_KE*0.007796968f)/MOTOR_PP))  //电机磁链 单位：Wb
#define	MOTOR_MAX_SPEED	((float)3000.0f)                       //最大转速：rpm  
#define MOTOR_I_MAX            ((float)(10.0f))              //电机额定电流：10A
#define MOTOR_J            ((float)(0.000058f))              //电机转动惯量：kg·m2
#define MOTOR_MAX_FREQ    ((float)((MOTOR_MAX_SPEED * MOTOR_PP) / 60.0f)) //最大频率 ：f = (N*P / 60)(转速*极对数/60) = 200HZ


//电机零位校准
#define T_ALIGNMENT           ((float) (1000.0f))    // 对齐时间：1000ms
#define ALIGNMENT_ANGLE       ((float) (0.0f))      //角度 0° [0..359] 
#define T_ALIGNMENT_PWM_STEPS ((float) ((T_ALIGNMENT * PWM_FREQUENCY)/1000)) //步进
#define I_ALIGNMENT           ((float)(5.0f))   //对齐时，Id的设定值 :5A

//板子硬件参数            
#define VBUS       ((float)(48.0f))                 //母线电压 单位：V
#define Ubase      ((float)((VBUS)/(SQRT3)))       //最大不失真圆相电压：Udc/√3
#define ADC_REF           ((float)(3.3f))           //参考电压3.3V
#define ADC_FullValue     ((float)(4095.0f))        //ADC满量程
#define GAIN              ((float)(6.0f))           //运放增益
#define RSHUNT            ((float)(20.0f))            //采样点阻 单位：mΩ
#define ADC_TO_CurrentAmp ((float)((1000.0f)*(ADC_REF)/(ADC_FullValue)/(GAIN)/(RSHUNT)))//每一个ADC值对应多少A电流
#define VBUS_UP_RES                 (float)(24.0f) //V_BUS上端分压电阻
#define VBUS_DOWN_RES               (float)(1.0f) //V_BUS下端分压电阻
#define VBUS_CONVERSION_FACTOR      (float)(ADC_REF*(VBUS_UP_RES+VBUS_DOWN_RES)/VBUS_DOWN_RES/4095.0f)//母线电压比例因子

//电流环PID KP/KI参数计算
#define CurrentBandWidth  ((float)(PWM_FREQUENCY/10.0f))//设置电流环带宽：20000/10 = 2000Hz
#define WC                ((float)((TWOPI)*(CurrentBandWidth)))//带宽的弧度表达:rad/s
#define AB                ((float)(VBUS*RSHUNT*GAIN)/(3.3f)/(1000.0f))
#define CurrentKp         ((float)((WC*MOTOR_LD/1000000.0f)/AB))     //KP = (Ld * Bandwidth)/(AB)
#define CurrentKi         ((float)(((WC*MOTOR_RS)/1000.0f/PWM_FREQUENCY)/AB))         //KI = R *Bandwidth *T/(AB)
//#define CurrentKp         (float)(WC*MOTOR_LD/1000000.0f)    //KP = (Ld * Bandwidth)
//#define CurrentKi         (float)(WC*MOTOR_RS/1000.0f)   //KI = R *Bandwidth
#define MAX_VOLTAGE_OUT   ((float)(VBUS*2.0f/3.0f)) //最大输出电压: 2Udc/3
#define CURREN_MAX_TLIMIT_OUT (MAX_VOLTAGE_OUT)
#define CURREN_MIN_TLIMIT_OUT (-MAX_VOLTAGE_OUT)
//速度环PID KP/KI参数计算
#define SpeedBandWidth  ((float)(CurrentBandWidth/250.0f))//设置速度环带宽：2000/250 = 200Hz
#define SpeedtKp        ((float)((SpeedBandWidth*MOTOR_J)/(1.5f*MOTOR_PP*MOTOR_FLUX)))     //KP = (J * Bandwidth)/(1.5*P*FLUX)
#define SpeedtKi        ((float)((float)SpeedBandWidth * SpeedtKp/750.0f)) //KI = Bandwidth*KP
//#define SpeedtKi        ((float)(0.0f)) //KI = Bandwidth*KP
#define SPEED_MAX_TLIMIT_OUT (MOTOR_I_MAX)
#define SPEED_MIN_TLIMIT_OUT (-MOTOR_I_MAX)



/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/

/*------------------------------------test------------------------------------*/
#endif





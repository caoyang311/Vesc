/**
 * *****************************************************************************
 * @file        motor.h
 * @brief       电机状态机
 * @author      caoyang
 * @date        2023-11-13
 * @copyright   欢阳技有限公司
 * *****************************************************************************
 */
#ifndef MORTOR_H
#define MORTOR_H
/*----------------------------------include-----------------------------------*/
#include "stm32f4xx.h"
/*-----------------------------------macro------------------------------------*/
#define NB_CONVERSIONS 16  //静态偏置ADC采样滤波

#define TRUE 1U
#define FALSE 0U

//电机启动阶段
#define	MOTOR_INIT_ENCODER_ALIGNMENT_NO_START		  0
#define	MOTOR_INIT_ENCODER_ALIGNMENT_START			  1
#define	MOTOR_INIT_ENCODER_ALIGNMENT_END			  2
//电机控制模式
#define MOTOR_TORQUE_MODE	      0     //力矩模式
#define MOTOR_SPEED_MODE		  1       //速度模式
#define	MOTOR_POSITION_MODE		  2     //位置模式
//传感器类型
#define HALL_SENSOR_MODE        0
#define	ENCODER_SENSOR_MODE		1
#define	NO_SENSOR_MODE			2


/*----------------------------------typedef-----------------------------------*/

#define MOTOR_IDLE      0 //空闲
#define MOTOR_INIT      1//初始化
#define MOTOR_START     2//开始启动
#define MOTOR_RUN       3//运行
#define MOTOR_STOP      4//停止
#define MOTOR_BRAKE     5//刹车
#define MOTOR_WAIT      6//等待
#define MOTOR_FAULT     7//错误

//FOC参数结构体
typedef struct Motor_Struct_Type
{
 uint8_t  Motor_State;//电机状态机
 uint8_t  Motor_State_Prv;//电机上一次状态
 uint8_t  Motor_Init_State;//电机初始化子状态
 uint8_t  Motor_Control_Mode;//电机控制模式
 uint8_t  Sensor_Type;       //电机传感器类型
}Motor_Struct;

/*----------------------------------variable----------------------------------*/
extern Motor_Struct BLDC_Motor;
extern uint16_t phase_A_offset;//无电流通过时采集到的AD值
extern uint16_t phase_B_offset;//无电流通过时采集到的AD值
extern uint16_t phase_C_offset;//无电流通过时采集到的AD值
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void MCL_MediumFrequency_Task(void);
void MCL_HighFrequency_Task(void);
void Motor_Init(void);
void SVPWM_CurrentReadingCalibration(void);
/*------------------------------------test------------------------------------*/
#endif 


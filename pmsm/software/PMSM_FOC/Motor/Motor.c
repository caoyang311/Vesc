#include "Motor.h"
#include "Motor_Identification.h"
#include "Motor_Position.h"
#include "Motor_Speed.h"
#include "Motor_VF.h"
#include "Motor_IF.h"
#include "Pwm.h"
#include "Dio.h"
#include "Board_Adc.h"
static Motor_ModeType Motor_CurrentMode = MOTOR_MODE_VF;
static bool Motor_IsRunning = false;

/**
 * @brief 初始化电机。
 */
void Motor_Init(void)
{
    
    
    Motor_VF_Init();
    Motor_IF_Init();
    Motor_Speed_Init();
    Motor_Position_Init();
    Motor_Identification_Init();
    Motor_CurrentMode = MOTOR_MODE_VF;
    Motor_IsRunning = false; 
    Adc_Start();/*启动ADC转换*/
    Pwm_StartAdcTrigger();/*开启定时器触发ADC转换*/
}
/**
 * @brief 设置电机模式。
 *
 * @param[in] Mode 电机模式。
 */
void Motor_SetMode(Motor_ModeType Mode)
{

    if ((Mode < MOTOR_MODE_COUNT) &&
        (Motor_IsRunning == false)) /* 检查模式是否有效且电机未运行 */
    {
        Motor_CurrentMode = Mode;
    }
}
/**
 * @brief 获取当前电机模式。
 *
 * @return Motor_ModeType 当前电机模式。
 */
Motor_ModeType Motor_GetMode(void)
{
    return Motor_CurrentMode;
}
/**
 * @brief 检查电机是否正在运行。
 *
 * @return bool 如果电机正在运行则返回 true，否则返回 false。
 */
bool Motor_GetIsRunning(void)
{
    return Motor_IsRunning;
}
/**
 * @brief 启动电机。
 */
void Motor_Start(void)
{
    if (Motor_IsRunning == false)
    {
        Motor_IsRunning = true;
        Dio_WriteChannel(DIO_CHANNEL_PM1_CTRL_SD,STD_HIGH);/*使能Mosfet输出*/
        Pwm_Start();/*开启PWM输出*/
    }
}
/**
 * @brief 停止电机。
 */
void Motor_Stop(void)
{
    if (Motor_IsRunning == true)
    {
        Motor_IsRunning = false;
        Pwm_Stop();/*关闭PWM输出*/    
        Dio_WriteChannel(DIO_CHANNEL_PM1_CTRL_SD,STD_LOW);/*关闭Mosfet输出*/
    }
}
/**
 * @brief 主函数，根据当前电机模式执行相应操作。
 */
void Motor_MainFunction(void)
{
    switch (Motor_CurrentMode)
    {
        case MOTOR_MODE_VF:
            Motor_VF_MainFunction();
            break;
        case MOTOR_MODE_IF:
            Motor_IF_MainFunction();
            break;
        case MOTOR_MODE_SPEED:
            Motor_Speed_MainFunction();
            break;
        case MOTOR_MODE_POSITION:
            Motor_Position_MainFunction();
            break;
        case MOTOR_MODE_IDENTIFICATION:
            Motor_Identification_MainFunction();
            break;
        default:
            break;
    }
}

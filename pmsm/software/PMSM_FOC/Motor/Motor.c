#include "Motor.h"
#include "Motor_Identification.h"
#include "Motor_Position.h"
#include "Motor_Speed.h"
#include "Motor_VF.h"
#include "Motor_IF.h"
#include "Pwm.h"
#include "Dio.h"
#include "Board_Adc.h"
#include "Sensor_Tamagawa.h"
#include "FocFilter.h"

#define MOTOR_ADC_REFERENCE_VOLTAGE_V     (3.3F)      /**< ADC参考电压 */
#define MOTOR_ADC_RESOLUTION_COUNT        (4095.0F)   /**< ADC分辨率计数 */
#define MOTOR_DC_BUS_R_TOP_OHM            (24000.0F)    /**< DC上分压电阻 */
#define MOTOR_DC_BUS_R_BOTTOM_OHM         (1000.0F)     /**< DC底部分压电阻 */



static Motor_ModeType Motor_CurrentMode = MOTOR_MODE_VF;
static bool Motor_IsRunning = false;
static float Motor_DcBusVoltageV = 0.0F;
static LPF_Filter Motor_DcBusVoltageFilter = {0.0f, 0.0f};

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
    FocLpf_Init(&Motor_DcBusVoltageFilter, 0.00626F);
    Motor_CurrentMode = MOTOR_MODE_IF;
    Motor_IsRunning = false;
    Motor_DcBusVoltageV = 0.0F;
    Adc_Start();/*启动ADC转换*/
    Pwm_StartAdcTrigger();/*开启定时器触发ADC转换*/
    //Sensor_Tamagawa_RequestAngle();/*读取电机初始角度*/
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
 * @brief 获取DC总电压。
 *
 * @return float DC总电压。
 */
float Motor_GetDcBusVoltage(void)
{
    Adc_ValueType adc_value = 0U;

    if (Adc_ReadChannel(ADC_CH_ID_DC_BUS_VOLTAGE, &adc_value) == ADC_RESULT_OK)
    {
        Motor_DcBusVoltageV =
            ((float)adc_value * MOTOR_ADC_REFERENCE_VOLTAGE_V /
             MOTOR_ADC_RESOLUTION_COUNT) *
            ((MOTOR_DC_BUS_R_TOP_OHM + MOTOR_DC_BUS_R_BOTTOM_OHM) /
             MOTOR_DC_BUS_R_BOTTOM_OHM);
    }
    return FocLpf_Update(&Motor_DcBusVoltageFilter, Motor_DcBusVoltageV);
}
/**
 * @brief 启动电机。
 */
void Motor_Start(void)
{
    if (Motor_IsRunning == false)
    {
        Pwm_Set_UVW_CCR(0U, 0U, 0U);
        Pwm_Start();/*开启PWM输出*/
        Dio_WriteChannel(DIO_CHANNEL_PM1_CTRL_SD,STD_HIGH);/*使能Mosfet输出*/
        Motor_IsRunning = true;
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
        Pwm_Set_UVW_CCR(0U, 0U, 0U);
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

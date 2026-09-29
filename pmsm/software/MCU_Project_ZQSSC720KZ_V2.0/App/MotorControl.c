/**
 * @file MotorControl.c
 * @brief 电机控制状态机模块实现。
 *
 * 当前版本仅建立状态机框架，各状态内部控制功能待后续实现。
 */

#include "MotorControl.h"
#include "FocAlgorithm.h"
#include "FocIfStart.h"
#include "FocVfStart.h"
#include "FocTrigTable.h"
#include <stddef.h>
#include <stdint.h>
#include "Test.h"
#include "Pwm.h"
#include "AdcService.h"
#include "FocMovingAverage.h"



float MotorControl_Vbus= 0.0F;    /**< 母线电压，单位为V。 */

static bool MotorControl_PWM_Test_Enable = true;
static MotorControl_StateType MotorControl_CurrentState = MOTOR_STATE_INITIALIZATION;
static MotorControl_ModeType MotorControl_CurrentMode = MOTOR_MODE_VF;
static FocMovingAverage_ContextType MotorControl_Vbus_Context =
{
    {0.0F},
    0.0F,
    0U,
    0U
};


static void MotorControl_Initialization(void);
static void MotorControl_Preposition(void);
static void MotorControl_ForceDragStart(void);
static void MotorControl_AngleClosedLoop(void);
static void MotorControl_SpeedClosedLoop(void);
static void MotorControl_Stop(void);
static void MotorControl_GteVbus(void);
/**
 * @brief 初始化电机控制状态机。
 *
 * 将当前状态设置为初始化状态。初始化状态内部功能待后续实现。
 */
void MotorControl_Init(void)
{
    MotorControl_CurrentState = MOTOR_STATE_INITIALIZATION;
    MotorControl_CurrentMode = MOTOR_MODE_VF;

    FocPiController_Init();
    FocVfStart_Reset();
    FocIfStart_Reset();
    FocIfStart_InitCurrentOffsetCalibration();
    FocMovingAverage_Init(&MotorControl_Vbus_Context);
}

/**
 * @brief 执行一次电机控制状态机任务。
 *
 * 根据当前状态调用对应的状态处理函数。各状态处理函数当前为空框架，
 * 不执行状态切换或电机控制动作。
 */
void MotorControl_Task10ms(void)
{
    switch (MotorControl_CurrentState)
    {
        case MOTOR_STATE_INITIALIZATION:
            MotorControl_Initialization();
            break;

        case MOTOR_STATE_PREPOSITION:
            MotorControl_Preposition();
            break;

        case MOTOR_STATE_FORCE_DRAG_START:
            MotorControl_ForceDragStart();
            break;

        case MOTOR_STATE_ANGLE_CLOSED_LOOP:
            MotorControl_AngleClosedLoop();
            break;

        case MOTOR_STATE_SPEED_CLOSED_LOOP:
            MotorControl_SpeedClosedLoop();
            break;

        case MOTOR_STATE_STOP:
            MotorControl_Stop();
            break;

        default:
            MotorControl_CurrentState = MOTOR_STATE_STOP;
            break;
    }
}
/**
 * @brief 执行一次5ms电机控制状态机任务。
 *
 * 根据当前状态调用对应的状态处理函数。各状态处理函数当前为空框架，
 * 不执行状态切换或电机控制动作。
 */
void MotorControl_Task5ms(void)
{
    MotorControl_GteVbus();
}

/**
 * @brief 获取当前电机控制状态。
 *
 * @return 当前电机控制状态。
 */
MotorControl_StateType MotorControl_GetState(void)
{
    return MotorControl_CurrentState;
}

/**
 * @brief 获取当前电机启动控制模式。
 *
 * @return 当前模式，初始化后的默认值为MOTOR_MODE_VF。
 */
MotorControl_ModeType MotorControl_GetMode(void)
{
    return MotorControl_CurrentMode;
}

/**
 * @brief 设置电机启动控制模式。
 *
 * @param[in] mode 待设置的启动模式，仅接受MOTOR_MODE_VF或MOTOR_MODE_IF。
 *
 * @return 模式有效并完成设置时返回true，否则返回false。
 */
bool MotorControl_SetMode(MotorControl_ModeType mode)
{
    bool is_set = false;

    if (((mode == MOTOR_MODE_VF) || (mode == MOTOR_MODE_IF)) &&
        ((MotorControl_CurrentState == MOTOR_STATE_INITIALIZATION) ||
         (MotorControl_CurrentState == MOTOR_STATE_STOP)))
    {
        MotorControl_CurrentMode = mode;
        FocVfStart_Reset();
        FocIfStart_Reset();
        is_set = true;
    }
    else
    {
        /* 非法模式或启动期间不修改已锁存模式。 */
    }

    return is_set;
}

/**
 * @brief 处理初始化状态并完成三相电流静态偏置标定。
 *
 * 标定期间保持初始化状态，累计完成后进入预定位状态。
 */
static void MotorControl_Initialization(void)
{
    if (MotorControl_PWM_Test_Enable == true)
    {
            Pwm_SetCompare(PWM_CHANNEL_U, 900);
            Pwm_SetCompare(PWM_CHANNEL_V, 1800);
            Pwm_SetCompare(PWM_CHANNEL_W, 2700);
            Pwm_StartOutputs();
    }
    else if (FocIfStart_CalibratePhaseCurrentOffset() == true)
    {
        if (message.id == 0x123)
        {
            if (message.data[0] == 0x01)
            {
                Pwm_StartOutputs();
                MotorControl_CurrentState = MOTOR_STATE_PREPOSITION; /* 进入预定位状态。 */
            }
        }     
    } 
   
}

/** @brief 处理预定位状态。 */
static void MotorControl_Preposition(void)
{

    if ((MotorControl_CurrentMode == MOTOR_MODE_VF) &&
        (FocVfStart_IsPrepositionComplete() == true))
    {
        FocVfStart_Reset();
        MotorControl_CurrentState = MOTOR_STATE_FORCE_DRAG_START;
    }
    else if ((MotorControl_CurrentMode == MOTOR_MODE_IF) &&
             (FocIfStart_IsPrepositionComplete() == true))
    {
        MotorControl_CurrentState = MOTOR_STATE_FORCE_DRAG_START;
    }
    else
    {
        if (message.id == 0x123)
        {
            if (message.data[0] == 0x00)
            {
                MotorControl_CurrentState = MOTOR_STATE_STOP; /* 停止功能待后续实现。 */
            }
        }        
    }
}

/** @brief 处理强拖启动状态。 */
static void MotorControl_ForceDragStart(void)
{
    if (message.id == 0x123)
    {
        if (message.data[0] == 0x00)
        {
            MotorControl_CurrentState = MOTOR_STATE_STOP; /* 停止功能待后续实现。 */
        }
    }
}

/** @brief 处理角度闭环状态。 */
static void MotorControl_AngleClosedLoop(void)
{
    /* 角度闭环功能待后续实现。 */
}

/** @brief 处理速度闭环状态。 */
static void MotorControl_SpeedClosedLoop(void)
{
    /* 速度闭环功能待后续实现。 */
}

/** @brief 处理停止状态。 */
static void MotorControl_Stop(void)
{   
    if (MotorControl_CurrentMode == MOTOR_MODE_VF)
    {
        FocVfStart_Reset();
    }
    else if (MotorControl_CurrentMode == MOTOR_MODE_IF)
    {
        FocIfStart_Reset();
    }
    else
    {
        /* 未知模式，不执行任何操作。 */    
    } 
    Pwm_StopOutputs();
    MotorControl_CurrentState = MOTOR_STATE_INITIALIZATION;   
}

/** @brief 获取电机电压。 */
static void MotorControl_GteVbus(void)
{
    uint16_t adc_value = 0U;
    float vbus = 0.0F;
    (void)AdcService_ReadChannel(ADC_SERVICE_UNIT_2, ADC_SERVICE_CHANNEL_ADC2_IN5, &adc_value);
    vbus = (float)adc_value * ADC_REF_VOLTAGE_V * MOTOR_VOLTAGE_DIVIDER_RATIO / 4095.0F;
    FocMovingAverage_Update(&MotorControl_Vbus_Context, vbus, &MotorControl_Vbus);
}
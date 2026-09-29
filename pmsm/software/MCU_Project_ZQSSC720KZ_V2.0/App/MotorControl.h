/**
 * @file MotorControl.h
 * @brief 电机控制状态机模块接口声明。
 *
 * 当前版本仅建立状态机框架，各状态内部控制功能待后续实现。
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "FocAlgorithm.h"
#include "FocPiController.h"
/**
 * @brief 电机参数。
 */
#define MOTOR_POLE_PAIRS             (4U)   /**< 磁极对数。 */
#define MOTOR_PHASE_COUNT            (3U)   /**< 相数。 */

#define MOTOR_RATED_VOLTAGE_V        (24.0F) /**< 额定电压，单位V。 */
#define MOTOR_RATED_SPEED_RPM        (3000.0F) /**< 额定转速，单位RPM。 */
#define MOTOR_RATED_CURRENT_A        (4.0F) /**< 额定电流，单位A。 */
#define MOTOR_BACK_EMF_V_PER_KRPM    (4.3F) /**< 反电势，单位V/kRPM。 */
#define MOTOR_ENCODER_LINES          (1000U) /**< 编码器线数。 */
#define MOTOR_RATED_POWER_W          (64.0F) /**< 输出功率，单位W。 */
#define MOTOR_RATED_TORQUE_NM        (0.2F) /**< 额定扭矩，单位N·m。 */

/**
 * @brief 板子硬件参数。
 */
/*PWM频率*/
#define PWM_FREQUENCY_HZ (10000.0F) /**< PWM频率，单位Hz。 */
/*ADC参考电压参数*/ 
#define ADC_REF_VOLTAGE_V (3.3F) /**< ADC参考电压，单位V。 */
/*母线电压分压比例参数*/
#define MOTOR_VOLTAGE_DIVIDER_RATIO (31.0F) /**< 母线电压分压比例。 */
/*电机相电流采样灵敏度参数*/
#define MOTOR_CURRENT_DIVIDER_RATIO (0.00222F) /**< 分压后的霍尔电流采样灵敏度，单位V/A。 */
/**
 * @brief 电机控制状态。
 */
typedef enum
{
    MOTOR_STATE_INITIALIZATION = 0U, /**< 初始化。 */
    MOTOR_STATE_PREPOSITION,         /**< 预定位。 */
    MOTOR_STATE_FORCE_DRAG_START,    /**< 强拖启动。 */
    MOTOR_STATE_ANGLE_CLOSED_LOOP,   /**< 角度闭环。 */
    MOTOR_STATE_SPEED_CLOSED_LOOP,   /**< 速度闭环。 */
    MOTOR_STATE_STOP                 /**< 停止。 */
} MotorControl_StateType;
/**
 * @brief 电机控制模式。
 */
typedef enum
{
    MOTOR_MODE_VF = 0U, /**< V/F模式。 */
    MOTOR_MODE_IF       /**< I/F模式。 */
} MotorControl_ModeType;    


extern float MotorControl_Vbus;
/**
 * @brief 初始化电机控制状态机。
 *
 * @post 状态机进入初始化状态。
 */
void MotorControl_Init(void);

/**
 * @brief 执行一次电机控制状态机任务。
 *
 * 当前版本仅保留各状态的处理边界，不实现状态内部功能和状态切换策略。
 *
 * @pre MotorControl_Init() 已完成。
 */
void MotorControl_Task10ms(void);

/**
 * @brief 获取当前电机控制状态。
 *
 * @return 当前电机控制状态。
 */
MotorControl_StateType MotorControl_GetState(void);

/**
 * @brief 获取当前电机启动控制模式。
 *
 * @return 当前模式，初始化后的默认值为MOTOR_MODE_VF。
 */
MotorControl_ModeType MotorControl_GetMode(void);

/**
 * @brief 设置电机启动控制模式。
 *
 * @param[in] mode 待设置的启动模式，仅接受MOTOR_MODE_VF或MOTOR_MODE_IF。
 *
 * @return 模式有效并完成设置时返回true，否则返回false。
 * @note 为保持启动过程一致性，仅初始化或停止状态允许修改模式。
 */
bool MotorControl_SetMode(MotorControl_ModeType mode);

/**
 * @brief 执行一次电机控制状态机任务。
 *
 * 当前版本仅保留各状态的处理边界，不实现状态内部功能和状态切换策略。
 *
 * @pre MotorControl_Init() 已完成。
 */
void MotorControl_Task5ms(void);

#endif /* MOTOR_CONTROL_H */

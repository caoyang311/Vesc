#include "Motor_Speed.h"
#include "FocAlgorithm.h"
#include "FocPID.h"
#include "FocTrigTable.h"
#include "FocCurrent.h"
#include "Sensor_Tamagawa.h"
#include "Com.h"
#include "Pwm.h"
#include <math.h>
#define MOTOR_SPEED_POLE_PAIRS                 (5U)      /**< 电机极对数 */
#define MOTOR_SPEED_CONTROL_FREQUENCY_HZ      (10000.0F) /**< 控制频率 */
#define MOTOR_SPEED_CONTROL_PERIOD_S          (1.0F / MOTOR_SPEED_CONTROL_FREQUENCY_HZ) /**< 电流环周期 */
#define MOTOR_SPEED_LOOP_PERIOD_S             (0.001F) /**< 速度环周期 */
#define MOTOR_SPEED_LOOP_DIVIDER              ((uint32_t)(MOTOR_SPEED_LOOP_PERIOD_S / MOTOR_SPEED_CONTROL_PERIOD_S)) /**< 速度环分频 */
#define MOTOR_SPEED_REFERENCE_RPM_DEFAULT     (0.0F) /**< 默认速度参考 */
#define MOTOR_SPEED_SENSOR_ANGLE_COMPENSATION_S (0.0001F) /**< 角度补偿时间 */
#define MOTOR_SPEED_ANGLE_COMPENSATION_SPEED_RPM (300.0F) /**< 角度补偿启用转速 */
#define MOTOR_SPEED_TWO_PI                    (6.2831853072F) /**< 2π */
#define MOTOR_SPEED_LD                         (0.00028F)    /**< D轴电感 单位：H */
#define MOTOR_SPEED_LQ                         (0.00028F)    /**< Q轴电感 单位：H */
#define MOTOR_SPEED_FLUX                       (0.011F)    /**< 磁链 单位：Wb */
#define MOTOR_SPEED_POWER_FILTER_ALPHA         (0.00626F)    /**< 功率滤波系数 */
#define MOTOR_SPEED_MIN_INPUT_POWER_W          (1.0F)        /**< 最小有效输入功率 */


#define MOTOR_SPEED_PREPOSITION_TIME_S         (0.5F)    /**< 编码器零度校准时间 */
#define MOTOR_SPEED_PREPOSITION_ID_REFERENCE_A (3.0F)    /**< 编码器零度校准D轴参考电流 */
#define MOTOR_SPEED_PREPOSITION_CURRENT_RAMP_TIME_S (0.2F) /**< 编码器零度校准电流斜坡时间 */


/**
 * @brief 编码器零度校准计数。
 */
#define MOTOR_SPEED_PREPOSITION_TICK_COUNT     \
    ((uint32_t)(MOTOR_SPEED_PREPOSITION_TIME_S * \
                MOTOR_SPEED_CONTROL_FREQUENCY_HZ))

/**
 * @brief零度校准电流斜坡步长。
 */
#define MOTOR_SPEED_PREPOSITION_CURRENT_STEP_A \
    (MOTOR_SPEED_PREPOSITION_ID_REFERENCE_A / \
     (MOTOR_SPEED_PREPOSITION_CURRENT_RAMP_TIME_S * MOTOR_SPEED_CONTROL_FREQUENCY_HZ))


/**
 * @brief Speed启动状态。
 */
typedef enum
{
    MOTOR_SPEED_STATE_IDLE = 0U, /**< 空闲状态 */
    MOTOR_SPEED_STATE_PREPOSITION, /**< 预定位状态 */
    MOTOR_SPEED_STATE_CLOSED_LOOP /**< 速度闭环状态 */
} Motor_SpeedStateIdType;


typedef struct
{
    Motor_SpeedStateIdType State; /**< IF状态 */
    Foc_TrigType Trig; /**< 三角函数类型 */
    Foc_PwmCompareType PwmCompare; /**< PWM比较类型 */
    Foc_Volt_AlphaBetaType VoltageAlphaBeta; /**< 电压AlphaBeta */
    Foc_Volt_DqType VoltageDq; /**< 电压Dq */
    Foc_PhaseCurrentType Current; /**< 三相电流 */
    Foc_Curr_AlphaBetaType CurrentAlphaBeta; /**< 电流AlphaBeta */
    Foc_Curr_DqType CurrentDq; /**< 电流Dq */
    Foc_VoltageLimitType VoltageLimit; /**< 电压限制 */
    uint16_t ElectricalAngle; /**< 原始电气相位 */
    uint16_t PllElectricalAngle; /**< PLL电气相位 */
    uint32_t PrepositionTick; /**< 预定位计数 */
    float ElectricalFrequencyHz; /**< 电气频率 */
    float ElectricalSpeed; /**< 电气角速度 */
    float MotorSpeedRpm; /**< 电机转速 */
    float IdReferenceA; /**< 当前D轴参考电流 */
    float IqReferenceA; /**< 当前Q轴参考电流 */
    float SpeedReferenceRpm; /**< 速度环参考转速 */
    uint32_t SpeedLoopTick; /**< 速度环分频计数 */
    float InputPowerW; /**< 电机输入功率，单位为瓦特 */
    float OutputPowerW; /**< 电机输出功率，单位为瓦特 */
    float EfficiencyPercent; /**< 电机效率，单位为百分比 */
    bool SensorZeroCalibrated; /**< 传感器零偏校准状态 */
} Motor_SpeedStateType;

static Motor_SpeedStateType Motor_Speed_State;

/**
 * @brief 斜坡函数。
 * @param current_reference_a 当前参考。
 * @param target_reference_a 目标参考。
 * @param step_a 斜坡步长。
 * @return float 下一个参考。
 */
static float Motor_Speed_RampReference(float current_reference_a,
                                    float target_reference_a,
                                    float step_a)
{
    float next_reference_a = current_reference_a;

    if (next_reference_a < target_reference_a)
    {
        next_reference_a += step_a;
        if (next_reference_a > target_reference_a)
        {
            next_reference_a = target_reference_a;
        }
        else
        {
        }
    }
    else if (next_reference_a > target_reference_a)
    {
        next_reference_a -= step_a;
        if (next_reference_a < target_reference_a)
        {
            next_reference_a = target_reference_a;
        }
        else
        {
        }
    }
    else
    {
    }

    return next_reference_a;
}


/**
 * @brief 重置Speed状态。
 */
static void Motor_Speed_ResetState(void)
{
    Motor_Speed_State = (Motor_SpeedStateType){0};
    Motor_Speed_State.State = MOTOR_SPEED_STATE_IDLE;
    Motor_Speed_State.SensorZeroCalibrated = Sensor_Tamagawa_IsElectricalOffsetValid();
    Motor_Speed_State.SpeedReferenceRpm = MOTOR_SPEED_REFERENCE_RPM_DEFAULT;
    Motor_Speed_State.SpeedLoopTick = 0U;
    PID_Reset();
}

/**
 * @brief 运行FOC。
 */
static void Motor_Speed_FocRun(void)
{
    float dc_bus_voltage_v = 0.0F;
    float voltage_limit_v = 0.0F;

    /*采样电流*/
    FocCurrent_SamplePhaseCurrent(&Motor_Speed_State.Current);
    /*转换为AlphaBeta坐标系*/
    FocAlgorithm_Clarke(&Motor_Speed_State.Current,
                        &Motor_Speed_State.CurrentAlphaBeta);
    /*Park变换*/                    
    FocTrig_GetSinCos(Motor_Speed_State.ElectricalAngle,
                        &Motor_Speed_State.Trig);
    FocAlgorithm_Park(&Motor_Speed_State.CurrentAlphaBeta,
                      &Motor_Speed_State.Trig,
                      &Motor_Speed_State.CurrentDq);
    dc_bus_voltage_v = Motor_GetDcBusVoltage();
    voltage_limit_v = dc_bus_voltage_v * 0.5773502692F;

    Current_PID_Calc(Motor_Speed_State.IdReferenceA,
                     Motor_Speed_State.CurrentDq.Id,
                     &ID_InitStructure,
                     MOTOR_SPEED_CONTROL_PERIOD_S,
                     &Motor_Speed_State.VoltageDq.Vd);

    Current_PID_Calc(Motor_Speed_State.IqReferenceA,
                     Motor_Speed_State.CurrentDq.Iq,
                     &IQ_InitStructure,
                     MOTOR_SPEED_CONTROL_PERIOD_S,
                     &Motor_Speed_State.VoltageDq.Vq);
    /*解耦反馈*/
    FocAlgorithm_DecouplingFeedforward(
        &Motor_Speed_State.CurrentDq,
        Motor_Speed_State.ElectricalSpeed,
        MOTOR_SPEED_LD,
        MOTOR_SPEED_LQ,
        MOTOR_SPEED_FLUX,
        FOC_DECOUPLING_DISABLED,
        &Motor_Speed_State.VoltageDq);

    (void)FocAlgorithm_LimitVoltage(
        &Motor_Speed_State.VoltageDq,
        voltage_limit_v,
        1.0F,
        1.0F,
        1.0F,
        &Motor_Speed_State.VoltageLimit);

    Current_PID_LimitIntegralDq(
        &ID_InitStructure,
        &IQ_InitStructure,
        &Motor_Speed_State.VoltageLimit);

    /*逆Park变换*/
    FocAlgorithm_InversePark(&Motor_Speed_State.VoltageDq,
                             &Motor_Speed_State.Trig,
                             &Motor_Speed_State.VoltageAlphaBeta);
    /*SVPWM*/
    if (dc_bus_voltage_v > 0.0F)
    {
        FocAlgorithm_Svpwm(&Motor_Speed_State.VoltageAlphaBeta,
                           &dc_bus_voltage_v,
                           &Motor_Speed_State.PwmCompare);
    }
    else
    {
        Motor_Speed_State.PwmCompare = (Foc_PwmCompareType){0};
    }
    /*设置PWM占空比*/
    Pwm_Set_UVW_CCR(Motor_Speed_State.PwmCompare.compare_u,
                    Motor_Speed_State.PwmCompare.compare_v,
                    Motor_Speed_State.PwmCompare.compare_w);
    // debug[0] = Motor_IF_State.Current.phase_u;
    // debug[1] = Motor_IF_State.Current.phase_v;
    // debug[2] = Motor_IF_State.Current.phase_w;
    // debug[0] = Motor_IF_State.DifferentialElectricalSpeed;
    // debug[2] = Motor_IF_State.ElectricalSpeed;
    // debug[0] = Motor_IF_State.PwmCompare.compare_u;
    // debug[1] = Motor_IF_State.PwmCompare.compare_v;
    // debug[2] = Motor_IF_State.PwmCompare.compare_w;  
    debug[0] = Motor_Speed_State.ElectricalSpeed;
    debug[1] = Motor_Speed_State.ElectricalFrequencyHz;
    debug[2] = Motor_Speed_State.MotorSpeedRpm;       
    Com_Uart_SendJustFloat(debug, 3);
}

/**
 * @brief 编码器零度校准。
 */
static void Motor_Speed_ZeroCalibrate(void)
{
    uint16_t sensor_phase = 0U;

    Motor_Speed_State.ElectricalAngle = 0U;
    Motor_Speed_State.IdReferenceA = Motor_Speed_RampReference(
        Motor_Speed_State.IdReferenceA,
        MOTOR_SPEED_PREPOSITION_ID_REFERENCE_A,
        MOTOR_SPEED_PREPOSITION_CURRENT_STEP_A);
    Motor_Speed_State.IqReferenceA = Motor_Speed_RampReference(
        Motor_Speed_State.IqReferenceA,
        0.0F,
        MOTOR_SPEED_PREPOSITION_CURRENT_STEP_A);
    // Motor_IF_State.IdReferenceA = 1.0F;
    // Motor_IF_State.IqReferenceA = 0.0F;
    Motor_Speed_FocRun();

    if (Motor_Speed_State.PrepositionTick >= MOTOR_SPEED_PREPOSITION_TICK_COUNT)
    {
        sensor_phase = Sensor_Tamagawa_GetElectricalPhase(MOTOR_SPEED_POLE_PAIRS);/*获取当前电气相位*/
        Sensor_Tamagawa_SetElectricalPhaseOffset(sensor_phase);/*设置电气相位偏移量*/
        Motor_Speed_State.SensorZeroCalibrated = true;
    }
    else
    {
    }
}

/**
 * @brief 运行闭环。
 */
static void Motor_Speed_RunCurrentLoop(void)
{
    /*速度计算*/
    Sensor_Tamagawa_SampleSpeed( MOTOR_SPEED_CONTROL_PERIOD_S, MOTOR_SPEED_POLE_PAIRS);/*PLL采样速度*/
    Motor_Speed_State.ElectricalSpeed = Sensor_Tamagawa_GetElectricalAngularSpeedRadS();/*获取当前角速度*/
    Motor_Speed_State.ElectricalFrequencyHz =Motor_Speed_State.ElectricalSpeed / MOTOR_SPEED_TWO_PI;/*获取电气频率*/
    Motor_Speed_State.MotorSpeedRpm = Sensor_Tamagawa_GetMotorSpeedRpm(MOTOR_SPEED_POLE_PAIRS,Motor_Speed_State.ElectricalSpeed);/*获取当前电机转速*/

    Motor_Speed_State.SpeedLoopTick++;
    /*1ms更新一次*/
    if (Motor_Speed_State.SpeedLoopTick >= MOTOR_SPEED_LOOP_DIVIDER)
    {
        Motor_Speed_State.SpeedLoopTick = 0U;
        Motor_Speed_State.IdReferenceA = 0.0F;
        Speed_PID_Calc(Motor_Speed_State.SpeedReferenceRpm,
                       Motor_Speed_State.MotorSpeedRpm,
                       &Speed_InitStructure,
                       MOTOR_SPEED_LOOP_PERIOD_S,
                       &Motor_Speed_State.IqReferenceA);
    }

    /*如果电机转速大于补偿阈值*/
    if (fabsf(Motor_Speed_State.MotorSpeedRpm) >= MOTOR_SPEED_ANGLE_COMPENSATION_SPEED_RPM)
    {
        Motor_Speed_State.ElectricalAngle = Sensor_Tamagawa_GetElectricalPhaseWithCompensation(MOTOR_SPEED_SENSOR_ANGLE_COMPENSATION_S);/*获取补偿后的电气相位*/
    }
    else
    {
        Motor_Speed_State.ElectricalAngle = Sensor_Tamagawa_GetElectricalPhaseWithOffset(MOTOR_SPEED_POLE_PAIRS);/*获取无补偿的电气相位*/
    } 

    Motor_Speed_FocRun();
}
/**
 * @brief 初始化速度控制。
 */
void Motor_Speed_Init(void)
{
    PID_init();
    FocCurrent_Init();
    Motor_Speed_ResetState();
}
/**
 * @brief 设置参考速度。
 * @param reference_rpm：参考速度值（RPM）
 */
void Motor_Speed_SetReferenceRpm(float reference_rpm)
{
    Motor_Speed_State.SpeedReferenceRpm = reference_rpm;
}
/**
 * @brief 速度控制函数。
 */
void Motor_Speed_ControlFunction(void)
{
    if (Motor_GetIsRunning() == true)
    {
        switch (Motor_Speed_State.State)
        {
            case MOTOR_SPEED_STATE_IDLE:
                Motor_Speed_State.PrepositionTick = 0U;
                Motor_Speed_State.SpeedLoopTick = 0U;
                Motor_Speed_State.IqReferenceA = 0.0F;
                Motor_Speed_State.ElectricalFrequencyHz = 0.0F;
                Motor_Speed_State.State = MOTOR_SPEED_STATE_PREPOSITION;
                break;

            case MOTOR_SPEED_STATE_PREPOSITION:
                if (Motor_Speed_State.SensorZeroCalibrated == true)
                {
                    Motor_Speed_State.State = MOTOR_SPEED_STATE_CLOSED_LOOP;
                    Motor_Speed_State.SpeedLoopTick = 0U;
                    Motor_Speed_State.IqReferenceA = 0.0F;
                }
                else
                {
                    Motor_Speed_ZeroCalibrate();
                    Motor_Speed_State.PrepositionTick++;
                    if (Motor_Speed_State.PrepositionTick >= MOTOR_SPEED_PREPOSITION_TICK_COUNT)
                    {
                        if (Motor_Speed_State.SensorZeroCalibrated == true)
                        {
                            Motor_Speed_State.State = MOTOR_SPEED_STATE_CLOSED_LOOP;
                            Motor_Speed_State.SpeedLoopTick = 0U;
                            Motor_Speed_State.IqReferenceA = 0.0F;
                        }
                        else
                        {
                        }
                    }
                    else
                    {
                    }
                }
                break;

            case MOTOR_SPEED_STATE_CLOSED_LOOP:
                Motor_Speed_RunCurrentLoop();
                break;

            default:
                Motor_Speed_ResetState(); 
                break;
        }        
    }
}
/**
 * @brief 主速度控制函数。
 */
void Motor_Speed_MainFunction(void)
{
    if (Motor_GetIsRunning() == false)
    {
        Motor_Speed_ResetState();
        FocCurrent_CalibrateBiasVoltage();
    }
    else
    {
        
    }    
}

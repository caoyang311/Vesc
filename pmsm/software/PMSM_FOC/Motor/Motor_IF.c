#include "Motor_IF.h"

#include "FocAlgorithm.h"
#include "FocCurrent.h"
#include "FocFilter.h"
#include "FocPID.h"
#include "FocTrigTable.h"
#include "Motor.h"
#include "Pwm.h"
#include "Sensor_Tamagawa.h"
#include "Com.h"
#include <math.h>

#define MOTOR_IF_POLE_PAIRS                 (5U)      /**< 电机极对数 */
#define MOTOR_IF_CONTROL_FREQUENCY_HZ      (10000.0F) /**< 控制频率 */
#define MOTOR_IF_CONTROL_PERIOD_S          (1.0F / MOTOR_IF_CONTROL_FREQUENCY_HZ) /**< 控制周期 */
#define MOTOR_IF_SENSOR_ANGLE_COMPENSATION_S (0.0001F) /**< 角度补偿时间 */
#define MOTOR_IF_ANGLE_COMPENSATION_SPEED_RPM (300.0F) /**< 角度补偿启用转速 */
#define MOTOR_IF_TWO_PI                    (6.2831853072F) /**< 2π */
#define MOTOR_IF_LD                         (0.00028F)    /**< D轴电感 单位：H */
#define MOTOR_IF_LQ                         (0.00028F)    /**< Q轴电感 单位：H */
#define MOTOR_IF_FLUX                       (0.011F)    /**< 磁链 单位：Wb */
#define MOTOR_IF_POWER_FILTER_ALPHA         (0.00626F)    /**< 功率滤波系数 */
#define MOTOR_IF_MIN_INPUT_POWER_W          (1.0F)        /**< 最小有效输入功率 */


#define MOTOR_IF_PREPOSITION_TIME_S         (0.5F)    /**< 预定位时间 */
#define MOTOR_IF_PREPOSITION_ID_REFERENCE_A (3.0F)    /**< 预定位D轴参考电流 */
#define MOTOR_IF_PREPOSITION_CURRENT_RAMP_TIME_S (0.2F) /**< 预定位电流斜坡时间 */

#define MOTOR_IF_TARGET_FREQUENCY_HZ        (10.0F)   /**< 强拖目标电频率 */
#define MOTOR_IF_FREQUENCY_RAMP_TIME_S      (0.5F)    /**< 强拖频率斜坡时间 */
#define MOTOR_IF_FORCE_DRAG_IQ_REFERENCE_A  (1.0F)    /**< 强拖Q轴参考电流 */
#define MOTOR_IF_FORCE_DRAG_CURRENT_RAMP_TIME_S (0.2F)  /**< 强拖电流斜坡时间 */

#define MOTOR_IF_CLOSED_LOOP_ID_REFERENCE_A (0.0F)    /**< 闭环D轴参考电流 */
#define MOTOR_IF_CLOSED_LOOP_IQ_REFERENCE_A (1.0F)    /**< 闭环Q轴参考电流 */
#define MOTOR_IF_CLOSED_LOOP_CURRENT_RAMP_TIME_S (0.2F) /**< 闭环电流斜坡时间 */
/**
 * @brief 预定位计数。
 */
#define MOTOR_IF_PREPOSITION_TICK_COUNT     \
    ((uint32_t)(MOTOR_IF_PREPOSITION_TIME_S * \
                MOTOR_IF_CONTROL_FREQUENCY_HZ))
/**
 * @brief 频率斜坡步长。
 */
#define MOTOR_IF_FREQUENCY_STEP_HZ          \
    (MOTOR_IF_TARGET_FREQUENCY_HZ / \
     (MOTOR_IF_FREQUENCY_RAMP_TIME_S * MOTOR_IF_CONTROL_FREQUENCY_HZ))

/**
 * @brief 预定位电流斜坡步长。
 */
#define MOTOR_IF_PREPOSITION_CURRENT_STEP_A \
    (MOTOR_IF_PREPOSITION_ID_REFERENCE_A / \
     (MOTOR_IF_PREPOSITION_CURRENT_RAMP_TIME_S * MOTOR_IF_CONTROL_FREQUENCY_HZ))
/**
 * @brief 强拖电流斜坡步长。
 */
#define MOTOR_IF_FORCE_DRAG_CURRENT_STEP_A \
    (MOTOR_IF_FORCE_DRAG_IQ_REFERENCE_A / \
     (MOTOR_IF_FORCE_DRAG_CURRENT_RAMP_TIME_S * MOTOR_IF_CONTROL_FREQUENCY_HZ))
/**
 * @brief 闭环电流斜坡步长。
 */
#define MOTOR_IF_CLOSED_LOOP_CURRENT_STEP_A \
    (MOTOR_IF_CLOSED_LOOP_IQ_REFERENCE_A / \
     (MOTOR_IF_CLOSED_LOOP_CURRENT_RAMP_TIME_S * MOTOR_IF_CONTROL_FREQUENCY_HZ))

/**
 * @brief IF启动状态。
 */
typedef enum
{
    MOTOR_IF_STATE_IDLE = 0U, /**< 空闲状态 */
    MOTOR_IF_STATE_PREPOSITION, /**< 预定位状态 */
    MOTOR_IF_STATE_FORCE_DRAG, /**< 强拖状态 */
    MOTOR_IF_STATE_CLOSED_LOOP /**< 角度闭环状态 */
} Motor_IFStateIdType;

typedef struct
{
    Motor_IFStateIdType State; /**< IF状态 */
    Motor_IFAngleSourceType AngleSource; /**< 角度来源 */
    Foc_TrigType Trig; /**< 三角函数类型 */
    Foc_PwmCompareType PwmCompare; /**< PWM比较类型 */
    Foc_Volt_AlphaBetaType VoltageAlphaBeta; /**< 电压AlphaBeta */
    Foc_Volt_DqType VoltageDq; /**< 电压Dq */
    Foc_PhaseCurrentType Current; /**< 三相电流 */
    Foc_Curr_AlphaBetaType CurrentAlphaBeta; /**< 电流AlphaBeta */
    Foc_Curr_DqType CurrentDq; /**< 电流Dq */
    Foc_VoltageLimitType VoltageLimit; /**< 电压限制 */
    uint16_t ElectricalAngle; /**< 电气相位 */
    uint32_t PrepositionTick; /**< 预定位计数 */
    float ElectricalFrequencyHz; /**< 电气频率 */
    float ElectricalSpeed; /**< FOC使用的电气速度 */
    float DifferentialElectricalSpeed; /**< 差分滤波电气速度 */
    float PllElectricalSpeed; /**< PLL电气速度 */
    float IdReferenceA; /**< 当前D轴参考电流 */
    float IqReferenceA; /**< 当前Q轴参考电流 */
    float InputPowerW; /**< 电机输入功率，单位为瓦特 */
    float OutputPowerW; /**< 电机输出功率，单位为瓦特 */
    float EfficiencyPercent; /**< 电机效率，单位为百分比 */
    bool SensorZeroCalibrated; /**< 传感器零偏校准状态 */
} Motor_IFStateType;

static Motor_IFStateType Motor_IF_State = {0};
static LPF_Filter Motor_IF_InputPowerFilter = {0};
static LPF_Filter Motor_IF_OutputPowerFilter = {0};
/**
 * @brief 电流斜坡。
 * @param current_reference_a 当前参考。
 * @param target_reference_a 目标参考。
 * @param step_a 斜坡步长。
 * @return float 下一个参考。
 */
static float Motor_IF_RampReference(float current_reference_a,
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
 * @brief 重置IF状态。
 */
static void Motor_IF_ResetState(void)
{
    Motor_IF_State = (Motor_IFStateType){0};
    Motor_IF_State.State = MOTOR_IF_STATE_IDLE;
    Motor_IF_State.AngleSource = MOTOR_IF_ANGLE_SOURCE_SENSOR;
    Motor_IF_State.SensorZeroCalibrated = Sensor_Tamagawa_IsElectricalOffsetValid();
    PID_Reset();
}
/**
 * @brief 运行FOC。
 */
static void Motor_IF_FocRun(void)
{
    float dc_bus_voltage_v = 0.0F;
    float voltage_limit_v = 0.0F;
    /*采样电流*/
    FocCurrent_SamplePhaseCurrent(&Motor_IF_State.Current);
    /*转换为AlphaBeta坐标系*/
    FocAlgorithm_Clarke(&Motor_IF_State.Current,
                        &Motor_IF_State.CurrentAlphaBeta);
    /*Park变换*/                    
    FocTrig_GetSinCos(Motor_IF_State.ElectricalAngle,
                        &Motor_IF_State.Trig);
    FocAlgorithm_Park(&Motor_IF_State.CurrentAlphaBeta,
                      &Motor_IF_State.Trig,
                      &Motor_IF_State.CurrentDq);
    /*电角速度计算*/
    if (Motor_IF_State.AngleSource == MOTOR_IF_ANGLE_SOURCE_SENSOR)
    {
        Sensor_Tamagawa_SampleSpeed(
            SENSOR_TAMAGAWA_UPDATE_PERIOD_S,
            MOTOR_IF_POLE_PAIRS);
            Sensor_Tamagawa_UpdatePll(MOTOR_IF_CONTROL_PERIOD_S,
                                   MOTOR_IF_POLE_PAIRS);
        Motor_IF_State.DifferentialElectricalSpeed =
            Sensor_Tamagawa_GetElectricalAngularSpeedRadS();
        Motor_IF_State.PllElectricalSpeed =
            Sensor_Tamagawa_GetPllElectricalAngularSpeedRadS();
        Motor_IF_State.ElectricalSpeed =
            Motor_IF_State.PllElectricalSpeed;
    }
    else
    {
        Motor_IF_State.ElectricalSpeed =
            MOTOR_IF_TWO_PI * Motor_IF_State.ElectricalFrequencyHz;
    }
    dc_bus_voltage_v = Motor_GetDcBusVoltage();
    voltage_limit_v = dc_bus_voltage_v * 0.5773502692F;

    Current_PID_Calc(Motor_IF_State.IdReferenceA,
                     Motor_IF_State.CurrentDq.Id,
                     &ID_InitStructure,
                     MOTOR_IF_CONTROL_PERIOD_S,
                     &Motor_IF_State.VoltageDq.Vd);

    Current_PID_Calc(Motor_IF_State.IqReferenceA,
                     Motor_IF_State.CurrentDq.Iq,
                     &IQ_InitStructure,
                     MOTOR_IF_CONTROL_PERIOD_S,
                     &Motor_IF_State.VoltageDq.Vq);
    /*解耦反馈*/
    FocAlgorithm_DecouplingFeedforward(
        &Motor_IF_State.CurrentDq,
        Motor_IF_State.ElectricalSpeed,
        MOTOR_IF_LD,
        MOTOR_IF_LQ,
        MOTOR_IF_FLUX,
        FOC_DECOUPLING_DISABLED,
        &Motor_IF_State.VoltageDq);

    (void)FocAlgorithm_LimitVoltage(
        &Motor_IF_State.VoltageDq,
        voltage_limit_v,
        1.0F,
        1.0F,
        1.0F,
        &Motor_IF_State.VoltageLimit);

    Current_PID_LimitIntegralDq(
        &ID_InitStructure,
        &IQ_InitStructure,
        &Motor_IF_State.VoltageLimit);

    /*逆Park变换*/
    FocAlgorithm_InversePark(&Motor_IF_State.VoltageDq,
                             &Motor_IF_State.Trig,
                             &Motor_IF_State.VoltageAlphaBeta);
    /*SVPWM*/
    if (dc_bus_voltage_v > 0.0F)
    {
        FocAlgorithm_Svpwm(&Motor_IF_State.VoltageAlphaBeta,
                           &dc_bus_voltage_v,
                           &Motor_IF_State.PwmCompare);
    }
    else
    {
        Motor_IF_State.PwmCompare = (Foc_PwmCompareType){0};
    }
    /*设置PWM占空比*/
    Pwm_Set_UVW_CCR(Motor_IF_State.PwmCompare.compare_u,
                    Motor_IF_State.PwmCompare.compare_v,
                    Motor_IF_State.PwmCompare.compare_w);
    // debug[0] = Motor_IF_State.Current.phase_u;
    // debug[1] = Motor_IF_State.Current.phase_v;
    // debug[2] = Motor_IF_State.Current.phase_w;
    debug[0] = Motor_IF_State.DifferentialElectricalSpeed;
    debug[1] = Motor_IF_State.PllElectricalSpeed;
    debug[2] = Motor_IF_State.ElectricalSpeed;
    // debug[0] = Motor_IF_State.PwmCompare.compare_u;
    // debug[1] = Motor_IF_State.PwmCompare.compare_v;
    // debug[2] = Motor_IF_State.PwmCompare.compare_w;    
    Com_Uart_SendJustFloat(debug, 3);
}
/**
 * @brief 运行预定位。
 */
static void Motor_IF_RunPreposition(void)
{
    uint16_t sensor_phase = 0U;

    Motor_IF_State.ElectricalAngle = 0U;
    Motor_IF_State.IdReferenceA = Motor_IF_RampReference(
        Motor_IF_State.IdReferenceA,
        MOTOR_IF_PREPOSITION_ID_REFERENCE_A,
        MOTOR_IF_PREPOSITION_CURRENT_STEP_A);
    Motor_IF_State.IqReferenceA = Motor_IF_RampReference(
        Motor_IF_State.IqReferenceA,
        0.0F,
        MOTOR_IF_PREPOSITION_CURRENT_STEP_A);
    // Motor_IF_State.IdReferenceA = 1.0F;
    // Motor_IF_State.IqReferenceA = 0.0F;
    Motor_IF_FocRun();

    if ((Motor_IF_State.AngleSource == MOTOR_IF_ANGLE_SOURCE_SENSOR) &&
        (Motor_IF_State.PrepositionTick >= MOTOR_IF_PREPOSITION_TICK_COUNT))
    {
        sensor_phase = Sensor_Tamagawa_GetElectricalPhase(MOTOR_IF_POLE_PAIRS);/*获取当前电气相位*/
        Sensor_Tamagawa_SetElectricalPhaseOffset(sensor_phase);/*设置电气相位偏移量*/
        Motor_IF_State.SensorZeroCalibrated = true;
    }
    else
    {
    }
}
/**
 * @brief 运行强拖。
 */
static void Motor_IF_RunForceDrag(void)
{
    if ((Motor_IF_State.AngleSource == MOTOR_IF_ANGLE_SOURCE_MANUAL) &&
        (Motor_IF_State.ElectricalFrequencyHz < MOTOR_IF_TARGET_FREQUENCY_HZ))
    {
        Motor_IF_State.ElectricalFrequencyHz += MOTOR_IF_FREQUENCY_STEP_HZ;
        if (Motor_IF_State.ElectricalFrequencyHz > MOTOR_IF_TARGET_FREQUENCY_HZ)
        {
            Motor_IF_State.ElectricalFrequencyHz = MOTOR_IF_TARGET_FREQUENCY_HZ;
        }
        else
        {
        }
    }
    else
    {
        Motor_IF_State.ElectricalFrequencyHz = MOTOR_IF_TARGET_FREQUENCY_HZ;
    }

    FocAlgorithm_GenerateAngle(Motor_IF_State.ElectricalFrequencyHz,
                               &Motor_IF_State.ElectricalAngle);
    Motor_IF_State.IdReferenceA = Motor_IF_RampReference(
        Motor_IF_State.IdReferenceA,
        0.0F,
        MOTOR_IF_FORCE_DRAG_CURRENT_STEP_A);
    Motor_IF_State.IqReferenceA = Motor_IF_RampReference(
        Motor_IF_State.IqReferenceA,
        MOTOR_IF_FORCE_DRAG_IQ_REFERENCE_A,
        MOTOR_IF_FORCE_DRAG_CURRENT_STEP_A);
    Motor_IF_FocRun();
}
/**
 * @brief 运行闭环。
 */
static void Motor_IF_RunCurrentLoop(void)
{
    float motor_speed_rpm = 0.0F;

    motor_speed_rpm = Sensor_Tamagawa_GetPllMotorSpeedRpm(
        MOTOR_IF_POLE_PAIRS);

    if (fabsf(motor_speed_rpm) >= MOTOR_IF_ANGLE_COMPENSATION_SPEED_RPM)
    {
        Motor_IF_State.ElectricalAngle =
            Sensor_Tamagawa_GetPllElectricalPhaseWithOffsetAndLead(
                MOTOR_IF_SENSOR_ANGLE_COMPENSATION_S);
    }
    else
    {
        Motor_IF_State.ElectricalAngle =
            Sensor_Tamagawa_GetElectricalPhaseWithOffset(
                MOTOR_IF_POLE_PAIRS);
    }
    // Motor_IF_State.IdReferenceA = Motor_IF_RampReference(
    //     Motor_IF_State.IdReferenceA,
    //     MOTOR_IF_CLOSED_LOOP_ID_REFERENCE_A,
    //     MOTOR_IF_CLOSED_LOOP_CURRENT_STEP_A);
    // Motor_IF_State.IqReferenceA = Motor_IF_RampReference(
    //     Motor_IF_State.IqReferenceA,
    //     MOTOR_IF_CLOSED_LOOP_IQ_REFERENCE_A,
    //     MOTOR_IF_CLOSED_LOOP_CURRENT_STEP_A);
    Motor_IF_State.IdReferenceA = 0;
    Motor_IF_State.IqReferenceA = 1.0F;
    Motor_IF_FocRun();
}
/**
 * @brief 计算电机输入功率。
 * @return float 电机功率，单位为瓦特
 */
static float Motor_IF_CalculateInputPowerW(void)
{
    float motor_electrical_power_w = 0.0F;

    motor_electrical_power_w = 1.5F *
        ((Motor_IF_State.VoltageDq.Vd * Motor_IF_State.CurrentDq.Id) +
         (Motor_IF_State.VoltageDq.Vq * Motor_IF_State.CurrentDq.Iq));

    return FocLpf_Update(&Motor_IF_InputPowerFilter,
                         motor_electrical_power_w);
}

static float Motor_IF_CalculateOutputPowerW(void)
{
    float electromagnetic_torque_nm = 0.0F;
    float mechanical_speed_rad_s = 0.0F;
    float motor_output_power_w = 0.0F;

    electromagnetic_torque_nm = 1.5F * (float)MOTOR_IF_POLE_PAIRS *
        ((MOTOR_IF_FLUX * Motor_IF_State.CurrentDq.Iq) +
         ((MOTOR_IF_LD - MOTOR_IF_LQ) *
          Motor_IF_State.CurrentDq.Id * Motor_IF_State.CurrentDq.Iq));
    mechanical_speed_rad_s = Motor_IF_State.ElectricalSpeed /
                             (float)MOTOR_IF_POLE_PAIRS;
    motor_output_power_w = electromagnetic_torque_nm *
                           mechanical_speed_rad_s;

    return FocLpf_Update(&Motor_IF_OutputPowerFilter,
                         motor_output_power_w);
}
/**
 * @brief 计算电机效率百分比。
 * @return float 电机效率百分比，单位为百分比
 */
static float Motor_IF_CalculateEfficiencyPercent(void)
{
    float efficiency_percent = 0.0F;

    if ((Motor_IF_State.InputPowerW > MOTOR_IF_MIN_INPUT_POWER_W) &&
        (Motor_IF_State.OutputPowerW > 0.0F))
    {
        efficiency_percent =
            (Motor_IF_State.OutputPowerW / Motor_IF_State.InputPowerW) *
            100.0F;
        if (efficiency_percent > 100.0F)
        {
            efficiency_percent = 100.0F;
        }
        else
        {
        }
    }
    else
    {
    }

    return efficiency_percent;
}
/**
 * @brief 初始化IF。
 */
void Motor_IF_Init(void)
{
    PID_init();
    Motor_IF_ResetState();
    FocCurrent_Init();
    FocLpf_Init(&Motor_IF_InputPowerFilter,
                MOTOR_IF_POWER_FILTER_ALPHA);
    FocLpf_Init(&Motor_IF_OutputPowerFilter,
                MOTOR_IF_POWER_FILTER_ALPHA);
}
/**
 * @brief 设置角度源。
 */
void Motor_IF_SetAngleSource(Motor_IFAngleSourceType angle_source)
{
    if (angle_source < 2U)
    {
        Motor_IF_State.AngleSource = angle_source;
        if (angle_source == MOTOR_IF_ANGLE_SOURCE_SENSOR)
        {
            Motor_IF_State.SensorZeroCalibrated =
                Sensor_Tamagawa_IsElectricalOffsetValid();
        }
        else
        {
            Motor_IF_State.SensorZeroCalibrated = false;
        }
    }
    else
    {
    }
}
/**
 * @brief 控制函数。
 */
void Motor_IF_ControlFunction(void)
{
    if (Motor_GetIsRunning() == true)
    {
        switch (Motor_IF_State.State)
        {
            case MOTOR_IF_STATE_IDLE:
                Motor_IF_State.PrepositionTick = 0U;
                Motor_IF_State.ElectricalFrequencyHz = 0.0F;
                Motor_IF_State.State = MOTOR_IF_STATE_PREPOSITION;
                break;

            case MOTOR_IF_STATE_PREPOSITION:
                if ((Motor_IF_State.AngleSource == MOTOR_IF_ANGLE_SOURCE_SENSOR) &&
                    (Motor_IF_State.SensorZeroCalibrated == true))
                {
                    Motor_IF_State.State = MOTOR_IF_STATE_CLOSED_LOOP;
                }
                else
                {
                    Motor_IF_RunPreposition();
                    Motor_IF_State.PrepositionTick++;
                    if (Motor_IF_State.PrepositionTick >= MOTOR_IF_PREPOSITION_TICK_COUNT)
                    {
                        if (Motor_IF_State.AngleSource == MOTOR_IF_ANGLE_SOURCE_SENSOR)
                        {
                            if (Motor_IF_State.SensorZeroCalibrated == true)
                            {
                                Motor_IF_State.State = MOTOR_IF_STATE_CLOSED_LOOP;
                            }
                            else
                            {
                            }
                        }
                        else
                        {
                            Motor_IF_State.State = MOTOR_IF_STATE_FORCE_DRAG;
                        }
                    }
                    else
                    {
                    }
                }
                break;

            case MOTOR_IF_STATE_FORCE_DRAG:
                if (Motor_IF_State.AngleSource == MOTOR_IF_ANGLE_SOURCE_MANUAL)
                {
                    Motor_IF_RunForceDrag();
                }
                else
                {
                    Motor_IF_State.State = MOTOR_IF_STATE_CLOSED_LOOP;
                }
                break;

            case MOTOR_IF_STATE_CLOSED_LOOP:
                Motor_IF_RunCurrentLoop();
                Motor_IF_State.InputPowerW = Motor_IF_CalculateInputPowerW();
                Motor_IF_State.OutputPowerW = Motor_IF_CalculateOutputPowerW();
                Motor_IF_State.EfficiencyPercent =
                    Motor_IF_CalculateEfficiencyPercent();
                break;

            default:
                Motor_IF_ResetState(); 
                break;
        }
    }
}
/**
 * @brief 主函数。
 */

void Motor_IF_MainFunction(void)
{
    if (Motor_GetIsRunning() == false)
    {
        Motor_IF_ResetState();
        FocCurrent_CalibrateBiasVoltage();
    }
    else
    {
        
    }
}

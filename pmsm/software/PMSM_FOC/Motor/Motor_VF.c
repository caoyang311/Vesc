#include "Motor_VF.h"

#include "FocAlgorithm.h"
#include "FocTrigTable.h"
#include "Pwm.h"

#define MOTOR_VF_POLE_PAIRS                         (5U)     /**< 电机极对数 */
#define MOTOR_VF_BACK_EMF_V_PER_KRPM               (7.0F)   /**< 电机反电动势V/kRPM */
#define MOTOR_VF_RATED_SPEED_RPM                   (3000.0F) /**< 电机额定转速RPM */
#define MOTOR_VF_CONTROL_FREQUENCY_HZ              (10000.0F) /**< PWM控制频率Hz */
#define MOTOR_VF_TARGET_FREQUENCY_HZ               (30.0F)    /**< 强拖目标频率Hz */
#define MOTOR_VF_FREQUENCY_RAMP_TIME_S             (0.5F)   /**< 强拖斜坡时间s */
#define MOTOR_VF_PREPOSITION_TIME_S                (0.5F)   /**< 预定位时间s */
#define MOTOR_VF_PREPOSITION_TICK_COUNT            \
    ((uint32_t)(MOTOR_VF_PREPOSITION_TIME_S * MOTOR_VF_CONTROL_FREQUENCY_HZ))
#define MOTOR_VF_FREQUENCY_STEP_HZ                 \
    (MOTOR_VF_TARGET_FREQUENCY_HZ /                              \
     (MOTOR_VF_FREQUENCY_RAMP_TIME_S * MOTOR_VF_CONTROL_FREQUENCY_HZ))
#define MOTOR_VF_BOOST_VOLTAGE_V                   (0.135F)    /**< 电机启动电压V */
#define MOTOR_VF_ALIGN_VOLTAGE_V                   (1.0F)     /**< 电机对齐电压V */
#define MOTOR_VF_BUS_VOLTAGE_V                     (48.0F)    /**< 母线电压V */
#define MOTOR_VF_INV_SQRT_THREE                    (0.5773502692F) /**< 1/sqrt(3) */
#define MOTOR_VF_D_AXIS_RATIO                      (0.0F)     /**< D轴比 */
#define MOTOR_VF_Q_AXIS_RATIO                      (1.0F)     /**< Q轴比 */
#define MOTOR_VF_RATED_ELECTRICAL_FREQUENCY_HZ     \
    ((MOTOR_VF_RATED_SPEED_RPM * (float)MOTOR_VF_POLE_PAIRS) / 60.0F)
#define MOTOR_VF_VF_SLOPE_V_PER_HZ                 \
    ((MOTOR_VF_BACK_EMF_V_PER_KRPM *                            \
      (MOTOR_VF_RATED_SPEED_RPM / 1000.0F)) /                   \
     MOTOR_VF_RATED_ELECTRICAL_FREQUENCY_HZ)

typedef enum
{
    MOTOR_VF_STATE_IDLE = 0U,
    MOTOR_VF_STATE_PREPOSITION,
    MOTOR_VF_STATE_FORCE_DRAG
} Motor_VfStateIdType;

typedef struct
{
    Foc_TrigType Trig;
    Foc_PwmCompareType PwmCompare;
    Foc_Volt_AlphaBetaType VoltageAlphaBeta;
    Foc_Volt_DqType VoltageDq;
    uint16_t ElectricalAngle;
    float ElectricalFrequencyHz;
    uint32_t PrepositionTick;
    Motor_VfStateIdType State;
} Motor_VF_Type;

static Motor_VF_Type Motor_VfState;
static const float Motor_VF_BusVoltageV = MOTOR_VF_BUS_VOLTAGE_V;
/**
 * @brief 重置电机状态。
 */
static void Motor_VF_ResetState(void)
{
    Motor_VfState.Trig.sine = 0.0F;
    Motor_VfState.Trig.cosine = 0.0F;
    Motor_VfState.PwmCompare.compare_u = 0U;
    Motor_VfState.PwmCompare.compare_v = 0U;
    Motor_VfState.PwmCompare.compare_w = 0U;
    Motor_VfState.VoltageAlphaBeta.V_alpha = 0.0F;
    Motor_VfState.VoltageAlphaBeta.V_beta = 0.0F;
    Motor_VfState.VoltageDq.Vd = 0.0F;
    Motor_VfState.VoltageDq.Vq = 0.0F;
    Motor_VfState.ElectricalAngle = 0U;
    Motor_VfState.ElectricalFrequencyHz = 0.0F;
    Motor_VfState.PrepositionTick = 0U;
    Motor_VfState.State = MOTOR_VF_STATE_IDLE;
}
/**
 * @brief 根据电机频率Hz，计算V/F输出电压。
 * 
 * @param frequency_hz 电机频率Hz。
 * @return float V/F输出电压。
 */
static float Motor_VF_CalculateVoltage(float frequency_hz)
{
    float voltage_v = MOTOR_VF_BOOST_VOLTAGE_V +
                      (MOTOR_VF_VF_SLOPE_V_PER_HZ * frequency_hz);
    float voltage_limit_v = MOTOR_VF_BUS_VOLTAGE_V *
                            MOTOR_VF_INV_SQRT_THREE;

    if (voltage_v > voltage_limit_v)
    {
        voltage_v = voltage_limit_v;
    }
    else
    {
        /* V/F输出未达到母线电压限制。 */
    }

    return (voltage_v < 0.0F) ? 0.0F : voltage_v;
}
/**
 * @brief 输出V/F电压到三相占空比。
 * 
 * @param voltage_d D轴电压。
 * @param voltage_q Q轴电压。
 */
static void Motor_VF_OutputVoltage(float voltage_d, float voltage_q)
{
    Motor_VfState.VoltageDq.Vd = voltage_d;
    Motor_VfState.VoltageDq.Vq = voltage_q;

    FocTrig_GetSinCos(Motor_VfState.ElectricalAngle,
                      &Motor_VfState.Trig);
    FocAlgorithm_InversePark(&Motor_VfState.VoltageDq,
                             &Motor_VfState.Trig,
                             &Motor_VfState.VoltageAlphaBeta);
    FocAlgorithm_Svpwm(&Motor_VfState.VoltageAlphaBeta,
                       &Motor_VF_BusVoltageV,
                       &Motor_VfState.PwmCompare);

    Pwm_Set_UVW_CCR(Motor_VfState.PwmCompare.compare_u,
                    Motor_VfState.PwmCompare.compare_v,
                    Motor_VfState.PwmCompare.compare_w);
}
/**
 * @brief 预定位。
 */
static void Motor_VF_RunPreposition(void)
{
    Motor_VfState.ElectricalAngle = 0U;
    Motor_VF_OutputVoltage(MOTOR_VF_ALIGN_VOLTAGE_V, 0.0F);
}
/**
 * @brief V/F强拖。
 */
static void Motor_VF_RunForceDrag(void)
{
    float voltage_magnitude_v = 0.0F;

    if (Motor_VfState.ElectricalFrequencyHz < MOTOR_VF_TARGET_FREQUENCY_HZ)
    {
        Motor_VfState.ElectricalFrequencyHz += MOTOR_VF_FREQUENCY_STEP_HZ;
        if (Motor_VfState.ElectricalFrequencyHz >
            MOTOR_VF_TARGET_FREQUENCY_HZ)
        {
            Motor_VfState.ElectricalFrequencyHz =
                MOTOR_VF_TARGET_FREQUENCY_HZ;
        }
        else
        {
            /* 电频率仍处于斜坡上升阶段。 */
        }
    }
    else
    {
        Motor_VfState.ElectricalFrequencyHz = MOTOR_VF_TARGET_FREQUENCY_HZ;
    }

    FocAlgorithm_GenerateAngle(Motor_VfState.ElectricalFrequencyHz,&Motor_VfState.ElectricalAngle);/** 更新电机角度 */
    voltage_magnitude_v = Motor_VF_CalculateVoltage(Motor_VfState.ElectricalFrequencyHz);/** 根据电机频率Hz，计算V/F输出电压 */
    Motor_VF_OutputVoltage(MOTOR_VF_D_AXIS_RATIO * voltage_magnitude_v,
                           MOTOR_VF_Q_AXIS_RATIO * voltage_magnitude_v);/** 输出V/F电压 */
}
/**
 * @brief 初始化V/F控制。
 */
void Motor_VF_Init(void)
{
    Motor_VF_ResetState();
}
/**
 * @brief V/F控制函数。
 * 
 * 该函数根据电机运行状态和V/F控制参数，输出V/F控制电压。
 */
void Motor_VF_ControlFunction(void)
{
    if (Motor_GetIsRunning() == true)
    {
        switch (Motor_VfState.State)
        {
            case MOTOR_VF_STATE_IDLE:
                Motor_VfState.PrepositionTick = 0U;
                Motor_VfState.ElectricalFrequencyHz = 0.0F;
                Motor_VfState.State = MOTOR_VF_STATE_PREPOSITION;
                break;

            case MOTOR_VF_STATE_PREPOSITION:
                Motor_VF_RunPreposition();
                Motor_VfState.PrepositionTick++;
                if (Motor_VfState.PrepositionTick >=
                    MOTOR_VF_PREPOSITION_TICK_COUNT)
                {
                    Motor_VfState.State = MOTOR_VF_STATE_FORCE_DRAG;
                }
                break;

            case MOTOR_VF_STATE_FORCE_DRAG:
                Motor_VF_RunForceDrag();
                break;

            default:
                Motor_VF_ResetState();
                break;
        }
    }
}

void Motor_VF_MainFunction(void)
{
    if (Motor_GetIsRunning() == false)
    {
       Motor_VF_ResetState();
    }
    else
    {
        /* V/F快速控制由ADC注入组转换完成中断回调执行。 */
    }
}

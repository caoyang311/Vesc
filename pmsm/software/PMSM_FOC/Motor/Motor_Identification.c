#include "Motor_Identification.h"
#include "FocAlgorithm.h"
#include "FocCurrent.h"
#include "FocTrigTable.h"
#include "FocPID.h"
#include "Pwm.h"
#include "Com.h"
#include "Motor.h"
#include <math.h>

#define MOTOR_IF_CONTROL_FREQUENCY_HZ      (10000.0F) /**< 控制频率 */
#define MOTOR_IF_CONTROL_PERIOD_S          (1.0F / MOTOR_IF_CONTROL_FREQUENCY_HZ) /**< 控制周期 */

#define MOTOR_IDENTIFICATION_RESISTANCE_TICK (10U) /**< 电阻辨识周期计数 */
#define MOTOR_IDENTIFICATION_RESISTANCE_SETTLE_TICK (50U) /**< 电阻辨识稳定周期计数 */
#define MOTOR_IDENTIFICATION_RESISTANCE_MAX_CURRENT (10.0F) /**< 电阻辨识最大电流 */
/**
 * @brief 电机辨识启动状态。
 */
typedef enum
{
    MOTOR_IDENTIFICATION_STATE_IDLE = 0U, /**< 空闲状态 */
    MOTOR_IDENTIFICATION_STATE_RESISTANCE = 1U, /**< 电阻辨识 */
    MOTOR_IDENTIFICATION_STATE_INDUCTANCE = 2U, /**< 电感辨识 */
    MOTOR_IDENTIFICATION_STATE_FLUX = 3U /**< 磁链辨识 */
} Motor_IdentificationStateIdType;

typedef struct
{
    Motor_IdentificationStateIdType State; /**< 电机辨识状态机 */
    Foc_TrigType Trig; /**< 三角函数类型 */
    Foc_PwmCompareType PwmCompare; /**< PWM比较类型 */
    Foc_Volt_AlphaBetaType VoltageAlphaBeta; /**< 电压AlphaBeta */
    Foc_Volt_DqType VoltageDq; /**< 电压Dq */
    Foc_PhaseCurrentType Current; /**< 三相电流 */
    Foc_Curr_AlphaBetaType CurrentAlphaBeta; /**< 电流AlphaBeta */
    Foc_Curr_DqType CurrentDq; /**< 电流Dq */
    Foc_VoltageLimitType VoltageLimit; /**< 电压限制 */
    uint16_t ElectricalAngle; /**< 原始电气相位 */
    float IdReferenceA; /**< 电流参考A */
    float IqReferenceA; /**< 电流参考B */
    float Resistance; /**< 电阻 */
    float Inductance; /**< 电感 */
    float Flux; /**< 磁链 */
} Motor_IdentificationType;

Motor_IdentificationType Motor_Identification;

/**
 * @brief 重置电机辨识状态机。
 */
static void Motor_Identification_ResetState(void)
{
    Motor_Identification = (Motor_IdentificationType){0};
    Motor_Identification.State = MOTOR_IDENTIFICATION_STATE_IDLE;
    PID_Reset();
}

/**
 * @brief 运行FOC。
 */
static void Motor_Identification_FocRun(void)
{
    float dc_bus_voltage_v = 0.0F;
    float voltage_limit_v = 0.0F;

    /*采样电流*/
    FocCurrent_SamplePhaseCurrent(&Motor_Identification.Current);
    /*转换为AlphaBeta坐标系*/
    FocAlgorithm_Clarke(&Motor_Identification.Current,
                        &Motor_Identification.CurrentAlphaBeta);
    FocTrig_GetSinCos(Motor_Identification.ElectricalAngle,
                        &Motor_Identification.Trig);
    FocAlgorithm_Park(&Motor_Identification.CurrentAlphaBeta,
                      &Motor_Identification.Trig,
                      &Motor_Identification.CurrentDq);
    dc_bus_voltage_v = Motor_GetDcBusVoltage();
    voltage_limit_v = dc_bus_voltage_v * 0.5773502692F;

    Current_PID_Calc(Motor_Identification.IdReferenceA,
                     Motor_Identification.CurrentDq.Id,
                     &ID_InitStructure,
                     MOTOR_IF_CONTROL_PERIOD_S,
                     &Motor_Identification.VoltageDq.Vd);
    
    Current_PID_Calc(Motor_Identification.IqReferenceA,
                     Motor_Identification.CurrentDq.Iq,
                     &IQ_InitStructure,
                     MOTOR_IF_CONTROL_PERIOD_S,
                     &Motor_Identification.VoltageDq.Vq);

    (void)FocAlgorithm_LimitVoltage(
        &Motor_Identification.VoltageDq,
        voltage_limit_v,
        1.0F,
        1.0F,
        1.0F,
        &Motor_Identification.VoltageLimit);

    Current_PID_LimitIntegralDq(
        &ID_InitStructure,
        &IQ_InitStructure,
        &Motor_Identification.VoltageLimit);

    /*逆Park变换*/
    FocAlgorithm_InversePark(&Motor_Identification.VoltageDq,
                             &Motor_Identification.Trig,
                             &Motor_Identification.VoltageAlphaBeta);
    /*SVPWM*/
    if (dc_bus_voltage_v > 0.0F)
    {
        FocAlgorithm_Svpwm(&Motor_Identification.VoltageAlphaBeta,
                           &dc_bus_voltage_v,
                           &Motor_Identification.PwmCompare);
    }
    else
    {
        Motor_Identification.PwmCompare = (Foc_PwmCompareType){0};
    }
    /*设置PWM占空比*/
    Pwm_Set_UVW_CCR(Motor_Identification.PwmCompare.compare_u,
                    Motor_Identification.PwmCompare.compare_v,
                    Motor_Identification.PwmCompare.compare_w);     
    // debug[0] = Motor_Identification.ElectricalAngle;
    // debug[1] = Motor_Identification.IdReferenceA;
    // debug[2] = Motor_Identification.IqReferenceA;       
    // Com_Uart_SendJustFloat(debug, 3);                               

}
/**
 * @brief 向目标值移动。
 */
static void utils_step_towards(float *value, float goal, float step)
{
    if (step < 0.0f) {
        step = -step;
    }

    if (*value < goal) {
        *value = ((goal - *value) <= step) ? goal : (*value + step);
    } else if (*value > goal) {
        *value = ((*value - goal) <= step) ? goal : (*value - step);
    }
}

/**
 * @brief 电阻辨识。
 */
static void Motor_Identification_IdentifyResistance(float current, int samples, bool stop_after, float *resistance)
{
    static uint32_t ResistanceSettleTick = 0U;
    static float avg_current_tot = 0.0F;
    static float avg_voltage_tot = 0.0F;
    static uint32_t sample_count = 0U;
       
    Motor_Identification.ElectricalAngle = 0.0F;
    Motor_Identification.IdReferenceA = 0.0F;
    if(fabsf(Motor_Identification.IqReferenceA- current) > 0.001F)
    {
        utils_step_towards(&Motor_Identification.IqReferenceA, current, fabsf(current) / (float)200.0F); 
    }
    else 
    {
        if(ResistanceSettleTick<50U)
        {
            ResistanceSettleTick++;
        }
        else
        {
            if(sample_count < samples)
            {
                avg_current_tot += sqrtf((Motor_Identification.CurrentDq.Id *
                            Motor_Identification.CurrentDq.Id) +
                            (Motor_Identification.CurrentDq.Iq *
                            Motor_Identification.CurrentDq.Iq));
                avg_voltage_tot += sqrtf((Motor_Identification.VoltageDq.Vd *
                            Motor_Identification.VoltageDq.Vd) +
                            (Motor_Identification.VoltageDq.Vq *
                            Motor_Identification.VoltageDq.Vq));
                sample_count++;               
            }
            else
            {
                avg_current_tot /= sample_count;
                avg_voltage_tot /= sample_count;
                *resistance = avg_current_tot / avg_voltage_tot;
                if(stop_after)
                {
                    Motor_Identification_ResetState();
                    Motor_Identification.State = MOTOR_IDENTIFICATION_STATE_IDLE;
                }
                else
                {
                    Motor_Identification.State = MOTOR_IDENTIFICATION_STATE_INDUCTANCE;
                }
            }
        }

    }


}

void Motor_Identification_Init(void)
{
    Motor_Identification = (Motor_IdentificationType){0};
    Motor_Identification.State = MOTOR_IDENTIFICATION_STATE_IDLE;
}

void Motor_Identification_ControlFunction(void)
{
    float i_last = 0.0;

    if (Motor_GetIsRunning() == true)
    {
        switch(Motor_Identification.State)
        {
            case MOTOR_IDENTIFICATION_STATE_IDLE:
                Motor_Identification_Init();
                Motor_Identification.State = MOTOR_IDENTIFICATION_STATE_RESISTANCE;
                break;
            case MOTOR_IDENTIFICATION_STATE_RESISTANCE:  
                for (float i = 2.0;i < (MOTOR_IDENTIFICATION_RESISTANCE_MAX_CURRENT / 2.0);i *= 1.5) {
                    float r_tmp = 0.0;
                    Motor_Identification_IdentifyResistance(i, 20, false, &r_tmp);
                    if (i > (1.0 / r_tmp)) {
                        i_last = i;
                        break;
                    }
                }
                if (i_last < 0.01) {
                    i_last = (MOTOR_IDENTIFICATION_RESISTANCE_MAX_CURRENT / 2.0);
                }
                Motor_Identification_IdentifyResistance(i_last, 200, true, &Motor_Identification.Resistance);
                break;
            case MOTOR_IDENTIFICATION_STATE_INDUCTANCE:
                break;
            case MOTOR_IDENTIFICATION_STATE_FLUX:
                break;
        }
       
    }
    else
    {
    }
}

void Motor_Identification_MainFunction(void)
{
    if (Motor_GetIsRunning() == false)
    {
        Motor_Identification_ResetState();
        FocCurrent_CalibrateBiasVoltage();
    }
    else
    {
        
    }

}



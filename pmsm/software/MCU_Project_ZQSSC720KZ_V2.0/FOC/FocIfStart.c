/**
 * @file FocIfStart.c
 * @brief I/F开环启动算法模块实现。
 */

#include "FocIfStart.h"
#include "FocAlgorithm.h"
#include "FocPiController.h"
#include "FocTrigTable.h"
#include "Pwm.h"
#include "AdcService.h"
#include "Test.h"
#define PWM_FREQUENCY_HZ                 (10000.0F) /**< PWM频率，单位Hz。 */
#define FOC_IF_START_ALIGN_CURRENT_A      (7.5F)  /**< 对齐电流，单位A。 */
#define FOC_IF_START_TORQUE_CURRENT_A     (7.5F)  /**< 转矩电流，单位A。 */
#define FOC_IF_START_TARGET_FREQ_HZ       (10.0F) /**< 目标电频率，单位Hz。 */
#define FOC_IF_START_ALIGN_RAMP_S         (0.3F)  /**< 对齐电流斜坡时间，单位s。 */
#define FOC_IF_START_PREPOSITION_HOLD_S   (0.2F)  /**< 对齐电流维持时间，单位s。 */
#define FOC_IF_START_ALIGN_RAMP_TICK_COUNT ((uint32_t)(FOC_IF_START_ALIGN_RAMP_S * PWM_FREQUENCY_HZ)) /**< 对齐电流斜坡中断次数。 */
#define FOC_IF_START_PREPOSITION_HOLD_TICK_COUNT ((uint32_t)(FOC_IF_START_PREPOSITION_HOLD_S * PWM_FREQUENCY_HZ)) /**< 对齐电流维持中断次数。 */
#define FOC_IF_START_CURRENT_RAMP_S       (0.5F)  /**< 转矩电流斜坡时间，单位s。 */
#define FOC_IF_START_FREQ_RAMP_S          (0.5F)  /**< 电频率斜坡时间，单位s。 */
#define FOC_IF_START_ALIGN_STEP_A         (FOC_IF_START_ALIGN_CURRENT_A / (FOC_IF_START_ALIGN_RAMP_S * PWM_FREQUENCY_HZ)) /**< 对齐电流斜坡步长，单位A。 */
#define FOC_IF_START_ALIGN_DOWN_STEP_A    (FOC_IF_START_ALIGN_CURRENT_A / (FOC_IF_START_CURRENT_RAMP_S * PWM_FREQUENCY_HZ)) /**< 强拖D轴电流下降步长，单位A。 */
#define FOC_IF_START_TORQUE_STEP_A        (FOC_IF_START_TORQUE_CURRENT_A / (FOC_IF_START_CURRENT_RAMP_S * PWM_FREQUENCY_HZ)) /**< 转矩电流斜坡步长，单位A。 */
#define FOC_IF_START_FREQ_STEP_HZ         (FOC_IF_START_TARGET_FREQ_HZ / (FOC_IF_START_FREQ_RAMP_S * PWM_FREQUENCY_HZ)) /**< 电频率斜坡步长，单位Hz。 */

#define ADC_REF_VOLTAGE_V (3.3F) /**< ADC参考电压，单位V。 */
#define VOLTAGE_DIVIDER_RATIO (31.0F) /**< 母线电压分压比例。 */
#define CURRENT_DIVIDER_RATIO (0.00222F) /**< 分压后的霍尔电流采样灵敏度，单位V/A。 */
#define ADC_FULL_SCALE_COUNT         (4095.0F) /**< ADC满量程计数。 */
#define FOC_IF_START_AMPERE_PER_COUNT (ADC_REF_VOLTAGE_V / (ADC_FULL_SCALE_COUNT * CURRENT_DIVIDER_RATIO)) /**< ADC计数对应电流，单位A/count。 */

#define FOC_IF_START_OFFSET_SAMPLE_COUNT (64U) /**< 静态偏置采样数量。 */

static Foc_PhaseCurrentType FocIfStart_PhaseCurrent = {0.0F, 0.0F, 0.0F}; /**< 三相电流，单位A。 */
static Foc_Curr_AlphaBetaType FocIfStart_CurrentAlphaBeta = {0.0F, 0.0F}; /**< 电流Alpha-Beta坐标，单位A。 */
static Foc_Curr_DqType FocIfStart_CurrentFeedbackDq = {0.0F, 0.0F}; /**< 电流DQ坐标，单位A。 */
static Foc_Volt_DqType FocIfStart_VoltageDq = {0.0F, 0.0F}; /**< 电压DQ坐标，单位V。 */
static Foc_Volt_AlphaBetaType FocIfStart_VoltageAlphaBeta = {0.0F, 0.0F}; /**< 电压Alpha-Beta坐标，单位V。 */
static Foc_TrigType FocIfStart_Trig = {0.0F, 1.0F}; /**< 三角函数值。 */
static Foc_PwmCompareType FocIfStart_PwmCompare = {0U, 0U, 0U};/*PWM比较输出*/
static uint16_t FocIfStart_ElectricalAngle = 0U; /**< 电角度，单位度。 */
static float FocIfStart_ElectricalFrequencyHz = 0.0F; /**< 电频率，单位Hz。 */
static uint32_t FocIfStart_PrepositionTick = 0U; /**< 预定位计数。 */
static bool FocIfStart_PrepositionComplete = false; /**< 预定位完成标志。 */
static Foc_Curr_DqType FocIfStart_CurrentReferenceDq = {0.0F, 0.0F}; /**< 电流DQ坐标，单位A。 */
static float FocIfStart_Vbus = 24.0F;   /**< 当前母线电压 */

static uint32_t FocIfStart_CurrentOffsetSumU = 0U;
static uint32_t FocIfStart_CurrentOffsetSumV = 0U;
static uint32_t FocIfStart_CurrentOffsetSumW = 0U;
static uint16_t FocIfStart_CurrentOffsetCount = 0U;
static float FocIfStart_CurrentOffsetU = 0.0F;
static float FocIfStart_CurrentOffsetV = 0.0F;
static float FocIfStart_CurrentOffsetW = 0.0F;
static bool FocIfStart_CurrentOffsetCalibrationComplete = false;


/**
 * @brief 初始化三相电流静态偏置标定。
 */
void FocIfStart_InitCurrentOffsetCalibration(void)
{
    FocIfStart_CurrentOffsetSumU = 0U;
    FocIfStart_CurrentOffsetSumV = 0U;
    FocIfStart_CurrentOffsetSumW = 0U;
    FocIfStart_CurrentOffsetCount = 0U;
    FocIfStart_CurrentOffsetU = 0.0F;
    FocIfStart_CurrentOffsetV = 0.0F;
    FocIfStart_CurrentOffsetW = 0.0F;
    FocIfStart_CurrentOffsetCalibrationComplete = false;
}

/**
 * @brief 将输入值按给定步长斜坡上升至目标值。
 *
 * @param[in] value 当前值。
 * @param[in] target 目标值。
 * @param[in] step 每次调用的正增量。
 *
 * @return 更新并限制后的值。
 */
static float FocIfStart_RampUp(float value, float target, float step);

/**
 * @brief 将输入值按给定步长斜坡下降至目标值。
 *
 * @param[in] value 当前值。
 * @param[in] target 目标值。
 * @param[in] step 每次调用的正减量。
 *
 * @return 更新并限制后的值。
 */
static float FocIfStart_RampDown(float value, float target, float step);

/**
 * @brief 读取并换算三相电流。
 *
 * @param[out] phase_current 三相电流输出，单位A，不得为NULL。
 *
 * @return 静态偏置有效且三个ADC通道读取成功时返回true，否则返回false。
 */
void FocIfStart_GetPhaseCurrent(Foc_PhaseCurrentType * phase_current);
/**
 * @brief 执行一次I/F电流闭环及PWM更新。
 *
 * @note 本函数在10 kHz中断中执行，调用有界且不阻塞。
 */
static void FocIfStart_Run(void);

/**
 * @brief 将输入值按给定步长斜坡上升至目标值。
 *
 * @param[in] value 当前值。
 * @param[in] target 目标值。
 * @param[in] step 每次调用的正增量。
 *
 * @return 更新并限制后的值。
 */
static float FocIfStart_RampUp(float value, float target, float step)
{
    float result = value;

    if (result < target)
    {
        result += step;
        if (result > target)
        {
            result = target;
        }
        else
        {
            /* 斜坡尚未达到目标值。 */
        }
    }
    else
    {
        result = target;
    }

    return result;
}

/**
 * @brief 将输入值按给定步长斜坡下降至目标值。
 *
 * @param[in] value 当前值。
 * @param[in] target 目标值。
 * @param[in] step 每次调用的正减量。
 *
 * @return 更新并限制后的值。
 */
static float FocIfStart_RampDown(float value, float target, float step)
{
    float result = value;

    if (result > target)
    {
        result -= step;
        if (result < target)
        {
            result = target;
        }
        else
        {
            /* 斜坡尚未达到目标值。 */
        }
    }
    else
    {
        result = target;
    }

    return result;
}

/**
 * @brief 读取并换算三相电流。
 *
 * 使用已标定的ADC静态偏置，将三相ADC计数差按采样灵敏度换算为安培。
 * 本函数供10 kHz中断调用，执行时间有界且不阻塞。
 *
 * @param[out] phase_current 三相电流输出，单位A，不得为NULL。
 *
 * @return 静态偏置有效且三个ADC通道读取成功时返回true，否则返回false。
 */
void FocIfStart_GetPhaseCurrent(Foc_PhaseCurrentType * phase_current)
{
    AdcService_ValueType adc_u = 0U;
    AdcService_ValueType adc_v = 0U;
    AdcService_ValueType adc_w = 0U;
    

    AdcService_ReadChannel(ADC_SERVICE_UNIT_2,
                                        ADC_SERVICE_CHANNEL_ADC2_IN12,
                                        &adc_u);
    AdcService_ReadChannel(ADC_SERVICE_UNIT_3,
                                        ADC_SERVICE_CHANNEL_ADC3_IN1,
                                        &adc_v);
    AdcService_ReadChannel(ADC_SERVICE_UNIT_3,
                                        ADC_SERVICE_CHANNEL_ADC3_IN12,
                                        &adc_w);
                                        
    phase_current->phase_u =((float)adc_u - FocIfStart_CurrentOffsetU) *FOC_IF_START_AMPERE_PER_COUNT;
    phase_current->phase_v =((float)adc_v - FocIfStart_CurrentOffsetV) *FOC_IF_START_AMPERE_PER_COUNT;
    phase_current->phase_w =((float)adc_w - FocIfStart_CurrentOffsetW) *FOC_IF_START_AMPERE_PER_COUNT;

}

/**
 * @brief 累积三相零电流采样并计算静态偏置。
 *
 * 每次调用读取一组三相ADC值，累计64组有效采样后计算各相平均偏置。
 * 标定期间必须保证逆变器无相电流。
 *
 * @return 静态偏置标定完成时返回true，否则返回false。
 */
bool FocIfStart_CalibratePhaseCurrentOffset(void)
{
    AdcService_ValueType adc_u = 0U;
    AdcService_ValueType adc_v = 0U;
    AdcService_ValueType adc_w = 0U;
    bool result = false;

    if (FocIfStart_CurrentOffsetCalibrationComplete == true)
    {
        return true;
    }
    else
    {
        /* 继续执行偏置校准。 */
    }

    AdcService_ReadChannel(ADC_SERVICE_UNIT_2,
                                         ADC_SERVICE_CHANNEL_ADC2_IN12,
                                         &adc_u);
    AdcService_ReadChannel(ADC_SERVICE_UNIT_3,
                                         ADC_SERVICE_CHANNEL_ADC3_IN1,
                                         &adc_v);
    AdcService_ReadChannel(ADC_SERVICE_UNIT_3,
                                         ADC_SERVICE_CHANNEL_ADC3_IN12,
                                         &adc_w);  

    FocIfStart_CurrentOffsetSumU += (uint32_t)adc_u;
    FocIfStart_CurrentOffsetSumV += (uint32_t)adc_v;
    FocIfStart_CurrentOffsetSumW += (uint32_t)adc_w;
    FocIfStart_CurrentOffsetCount++;

    if (FocIfStart_CurrentOffsetCount >=
        FOC_IF_START_OFFSET_SAMPLE_COUNT)
    {
        FocIfStart_CurrentOffsetU =
            (float)FocIfStart_CurrentOffsetSumU /
            (float)FOC_IF_START_OFFSET_SAMPLE_COUNT;
        FocIfStart_CurrentOffsetV =
            (float)FocIfStart_CurrentOffsetSumV /
            (float)FOC_IF_START_OFFSET_SAMPLE_COUNT;
        FocIfStart_CurrentOffsetW =
            (float)FocIfStart_CurrentOffsetSumW /
            (float)FOC_IF_START_OFFSET_SAMPLE_COUNT;
        FocIfStart_CurrentOffsetCalibrationComplete = true;
        result = true;
    }
    else
    {
        /* 继续累积静态偏置采样。 */
    }

    return result;
}


/**
 * @brief 执行一次I/F电流闭环及PWM更新。
 *
 * 依次完成三相采样、Clarke、Park、电流PI、反Park、SVPWM和PWM提交。
 *
 * @note 本函数在10 kHz中断中执行，调用有界且不阻塞。
 */
static void FocIfStart_Run(void)
{


    FocIfStart_GetPhaseCurrent(&FocIfStart_PhaseCurrent);

    FocAlgorithm_Clarke(&FocIfStart_PhaseCurrent,
                        &FocIfStart_CurrentAlphaBeta);
    FocTrig_GetSinCos(FocIfStart_ElectricalAngle, &FocIfStart_Trig);
    FocAlgorithm_Park(&FocIfStart_CurrentAlphaBeta,
                        &FocIfStart_Trig,
                        &FocIfStart_CurrentFeedbackDq);
    FocPiController_UpdateD(
        FocIfStart_CurrentFeedbackDq.Id,
        FocIfStart_CurrentReferenceDq.Id,
        FocIfStart_Vbus,
        &FocIfStart_VoltageDq.Vd);
    FocPiController_UpdateQ(
        FocIfStart_CurrentFeedbackDq.Iq,
        FocIfStart_CurrentReferenceDq.Iq,
        FocIfStart_Vbus,
        &FocIfStart_VoltageDq.Vq);

    FocAlgorithm_LimitVoltageCircle(&FocIfStart_VoltageDq, FocIfStart_Vbus);
    
    FocAlgorithm_InversePark(&FocIfStart_VoltageDq,
                                    &FocIfStart_Trig,
                                    &FocIfStart_VoltageAlphaBeta);
    FocAlgorithm_Svpwm(&FocIfStart_VoltageAlphaBeta,
                            &FocIfStart_Vbus,
                            &FocIfStart_PwmCompare);/* SVPWM计算 */
    Pwm_SetCompare(PWM_CHANNEL_U, FocIfStart_PwmCompare.compare_u);/* 提交PWM比较值 */
    Pwm_SetCompare(PWM_CHANNEL_V, FocIfStart_PwmCompare.compare_v);/* 提交PWM比较值 */
    Pwm_SetCompare(PWM_CHANNEL_W, FocIfStart_PwmCompare.compare_w);/* 提交PWM比较值 */
    Test_SendJustFloat(FocIfStart_PhaseCurrent.phase_u,FocIfStart_PhaseCurrent.phase_v,FocIfStart_PhaseCurrent.phase_w);
}

/**
 * @brief 复位I/F开环启动状态和电流PI积分状态。
 */
void FocIfStart_Reset(void)
{
    FocIfStart_PrepositionTick = 0U;
    FocIfStart_PrepositionComplete = false;
    FocIfStart_CurrentReferenceDq.Id = 0.0F;
    FocIfStart_CurrentReferenceDq.Iq = 0.0F;
    FocIfStart_CurrentFeedbackDq.Id = 0.0F;
    FocIfStart_CurrentFeedbackDq.Iq = 0.0F;
    FocIfStart_VoltageDq.Vd = 0.0F;
    FocIfStart_VoltageDq.Vq = 0.0F;
    FocIfStart_ElectricalAngle = 0U;
    FocIfStart_ElectricalFrequencyHz = 0.0F;
    FocPiController_Reset();
}

/**
 * @brief 执行一次I/F启动预定位控制。
 *
 * 固定开环电角度，先斜坡建立D轴定位电流，再维持该电流一段时间，Q轴电流给定保持为零。
 *
 * @note 本函数在10 kHz中断中调用，执行时间有界且不阻塞。
 */
void FocIfStart_Preposition(void)
{
    FocIfStart_ElectricalAngle = 0U;
    FocIfStart_ElectricalFrequencyHz = 0.0F;

    if (FocIfStart_PrepositionTick < FOC_IF_START_ALIGN_RAMP_TICK_COUNT)
    {
        FocIfStart_CurrentReferenceDq.Id = FocIfStart_RampUp(
            FocIfStart_CurrentReferenceDq.Id,
            FOC_IF_START_ALIGN_CURRENT_A,
            FOC_IF_START_ALIGN_STEP_A);
        FocIfStart_PrepositionTick++;
    }
    else if (FocIfStart_PrepositionTick <
             (FOC_IF_START_ALIGN_RAMP_TICK_COUNT + FOC_IF_START_PREPOSITION_HOLD_TICK_COUNT))
    {
        FocIfStart_CurrentReferenceDq.Id = FOC_IF_START_ALIGN_CURRENT_A;
        FocIfStart_PrepositionTick++;
    }
    else
    {
        FocIfStart_CurrentReferenceDq.Id = FOC_IF_START_ALIGN_CURRENT_A;
        FocIfStart_PrepositionComplete = true;
    }

    FocIfStart_CurrentReferenceDq.Iq = 0.0F;
    FocIfStart_Run();
}

/**
 * @brief 获取I/F预定位完成状态。
 *
 * @return 预定位斜坡和维持时间均完成时返回true，否则返回false。
 */
bool FocIfStart_IsPrepositionComplete(void)
{
    return FocIfStart_PrepositionComplete;
}

/**
 * @brief 执行一次I/F开环强拖控制。
 *
 * 保持预定位完成时的D轴电流，从该电流开始逐步减小，同时建立Q轴电流和开环电频率斜坡。
 *
 * @note 本函数在10 kHz中断中调用，执行时间有界且不阻塞。
 */
void FocIfStart_ForceDrag(void)
{
    FocIfStart_CurrentReferenceDq.Id = FocIfStart_RampDown(
        FocIfStart_CurrentReferenceDq.Id,
        0.0F,
        FOC_IF_START_ALIGN_DOWN_STEP_A);
    FocIfStart_CurrentReferenceDq.Iq = FocIfStart_RampUp(
        FocIfStart_CurrentReferenceDq.Iq,
        FOC_IF_START_TORQUE_CURRENT_A,
        FOC_IF_START_TORQUE_STEP_A);
    FocIfStart_ElectricalFrequencyHz = FocIfStart_RampUp(
        FocIfStart_ElectricalFrequencyHz,
        FOC_IF_START_TARGET_FREQ_HZ,
        FOC_IF_START_FREQ_STEP_HZ);
    FocAlgorithm_GenerateAngle(FocIfStart_ElectricalFrequencyHz,
                               &FocIfStart_ElectricalAngle);
    FocIfStart_Run();
}

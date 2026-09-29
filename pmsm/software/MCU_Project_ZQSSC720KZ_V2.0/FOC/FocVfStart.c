/**
 * @file FocVfStart.c
 * @brief V/F开环启动算法模块实现。
 */

#include "FocVfStart.h"
#include "FocAlgorithm.h"
#include "FocTrigTable.h"
#include "FocSmObserver.h"
#include "FocIfStart.h"
#include "Pwm.h"
#include "AdcService.h"
#include "Test.h"
#include "EncoderService.h"



#define MOTOR_POLE_PAIRS             (4U)   /**< 磁极对数。 */
#define MOTOR_BACK_EMF_V_PER_KRPM (4.3F) /**< 额定反电动势，单位V/kRPM。 */
#define MOTOR_RATED_SPEED_RPM (3000.0F) /**< 额定速度，单位RPM。 */
#define PWM_FREQUENCY_HZ (10000.0F) /**< PWM频率，单位Hz。 */
#define FOC_VF_START_INV_SQRT_THREE (0.5773502692F) /**< 1/√3 */
#define FOC_VF_START_TARGET_FREQ_HZ (50.0F) /**< 目标频率 */
#define FOC_VF_START_FREQUENCY_RAMP_TIME_S (0.5F) /**< 强拖频率斜坡时间 */
#define FOC_VF_START_PREPOSITION_TIME_S (0.5F) /**< 预定位时间，单位s。 */
#define FOC_VF_START_PREPOSITION_TICK_COUNT ((uint32_t)(FOC_VF_START_PREPOSITION_TIME_S * PWM_FREQUENCY_HZ)) /**< 预定位中断次数。 */
#define FOC_VF_START_FREQ_STEP_HZ     (FOC_VF_START_TARGET_FREQ_HZ / (FOC_VF_START_FREQUENCY_RAMP_TIME_S * PWM_FREQUENCY_HZ)) /**< 强拖频率斜坡步长 */
#define FOC_VF_START_BOOST_VOLTAGE_V     (0.05F)  /**< 强拖启动电压 */
#define FOC_VF_START_ALIGN_VOLTAGE_V     (1.0F)  /**< D轴对齐电压 */
#define MOTOR_RATED_ELECTRICAL_FREQUENCY_HZ  ((MOTOR_RATED_SPEED_RPM * (float)MOTOR_POLE_PAIRS) / 60.0F) /**< 额定电频率，单位Hz。 */
#define FOC_VF_START_VF_SLOPE_V_PER_HZ ((MOTOR_BACK_EMF_V_PER_KRPM * (MOTOR_RATED_SPEED_RPM / 1000.0F)) / MOTOR_RATED_ELECTRICAL_FREQUENCY_HZ) /**< V/F斜率，单位V/Hz。 */
#define FOC_VF_START_D_AXIS_RATIO       (0.0F) /**< D轴比例 */
#define FOC_VF_START_Q_AXIS_RATIO       (1.F) /**< Q轴比例 */

static Foc_TrigType FocVfStart_Trig = {0.0F, 0.0F};   /**< 三角函数表索引 */
static Foc_PwmCompareType FocVfStart_PwmCompare = {0U, 0U, 0U};   /**< PWM比较值 */
static Foc_Volt_AlphaBetaType FocVfStart_VoltageAlphaBeta = {0.0F, 0.0F};   /**< αβ轴电压矢量 */
static Foc_Volt_AlphaBetaType FocVfStart_PreviousVoltageAlphaBeta = {0.0F, 0.0F};   /**< 上一周期αβ轴电压矢量 */
static Foc_Volt_DqType FocVfStart_VoltageDq = {0.0F, 0.0F};   /**< D/Q电压矢量 */
static Foc_PhaseCurrentType FocVfStart_PhaseCurrent = {0.0F, 0.0F, 0.0F};   /**< 三相电流 */
static Foc_Curr_AlphaBetaType FocVfStart_CurrentAlphaBeta = {0.0F, 0.0F};   /**< αβ轴电流 */
static Foc_SmoObserverOutputType FocVfStart_ObserverOutput = {0.0F, 0U, 0.0F, 0.0F, 0.0F, 0.0F, false, false};   /**< 观测器输出 */
static EncoderService_OutputType EncoderService_Output = {0U};   /**< 编码器输出 */
static bool FocVfStart_ObserverInitialized = false;   /**< 观测器初始化状态 */
static uint16_t FocVfStart_ElectricalAngle = 0U;   /**< 电角 */
static float FocVfStart_ElectricalFrequencyHz = 0.0F;   /**< 电频率 */
static uint32_t FocVfStart_PrepositionTick = 0U;   /**< 预定位计数 */
static bool FocVfStart_PrepositionComplete = false;   /**< 预定位完成标志 */
static float FocVfStart_Vbus = 24.0F;   /**< 当前母线电压 */
/**
 * @brief 更新并限制开环电频率斜坡。
 */
static void FocVfStart_UpdateFrequency(void);

/**
 * @brief 根据当前电频率计算受限的V/F电压幅值。
 *
 * @param[in] frequency_hz 当前电频率，单位Hz，取值应不小于0。
 *
 * @return 受母线电压限制的电压矢量幅值，单位V。
 */
static float FocVfStart_CalculateVoltage(float frequency_hz);

/**
 * @brief 将D/Q电压转换为三相PWM比较值并输出。
 *
 * @param[in] voltage_d D轴电压，单位V。
 * @param[in] voltage_q Q轴电压，单位V。
 */
static void FocVfStart_Run(float voltage_d, float voltage_q);

/**
 * @brief 更新并限制开环电频率斜坡。
 */
static void FocVfStart_UpdateFrequency(void)
{
    if (FocVfStart_ElectricalFrequencyHz < FOC_VF_START_TARGET_FREQ_HZ)
    {
        FocVfStart_ElectricalFrequencyHz += FOC_VF_START_FREQ_STEP_HZ;
        if (FocVfStart_ElectricalFrequencyHz > FOC_VF_START_TARGET_FREQ_HZ)
        {
            FocVfStart_ElectricalFrequencyHz = FOC_VF_START_TARGET_FREQ_HZ;
        }
        else
        {
            /* 频率仍处于上升斜坡内。 */
        }
    }
    else
    {
        FocVfStart_ElectricalFrequencyHz = FOC_VF_START_TARGET_FREQ_HZ;
    }
}

/**
 * @brief 根据当前电频率计算受限的V/F电压幅值。
 *
 * @param[in] frequency_hz 当前电频率，单位Hz，取值应不小于0。
 *
 * @return 受母线电压限制的电压矢量幅值，单位V。
 */
static float FocVfStart_CalculateVoltage(float frequency_hz)
{
    float voltage_v = 0.0F;
    float voltage_limit_v = 0.0F;

    voltage_v = FOC_VF_START_BOOST_VOLTAGE_V +
                (FOC_VF_START_VF_SLOPE_V_PER_HZ * frequency_hz);
    voltage_limit_v = FocVfStart_Vbus * FOC_VF_START_INV_SQRT_THREE;

    if (voltage_v > voltage_limit_v)
    {
        voltage_v = voltage_limit_v;
    }
    else
    {
        /* 电压未达到SVPWM线性调制上限。 */
    }

    if (voltage_v < 0.0F)
    {
        voltage_v = 0.0F;
    }
    else
    {
        /* 电压保持非负。 */
    }

    return voltage_v;
}

/**
 * @brief 将D/Q电压转换为三相PWM比较值并输出。
 *
 * @param[in] voltage_d D轴电压，单位V。
 * @param[in] voltage_q Q轴电压，单位V。
 */
static void FocVfStart_Run(float voltage_d, float voltage_q)
{
    FocVfStart_VoltageDq.Vd = voltage_d;
    FocVfStart_VoltageDq.Vq = voltage_q;

    FocTrig_GetSinCos(FocVfStart_ElectricalAngle, &FocVfStart_Trig);
    FocAlgorithm_InversePark(&FocVfStart_VoltageDq,
                             &FocVfStart_Trig,
                             &FocVfStart_VoltageAlphaBeta);
    FocAlgorithm_Svpwm(&FocVfStart_VoltageAlphaBeta,
                       &FocVfStart_Vbus,
                       &FocVfStart_PwmCompare);

    FocVfStart_PreviousVoltageAlphaBeta = FocVfStart_VoltageAlphaBeta;

    Pwm_SetCompare(PWM_CHANNEL_U, FocVfStart_PwmCompare.compare_u);
    Pwm_SetCompare(PWM_CHANNEL_V, FocVfStart_PwmCompare.compare_v);
    Pwm_SetCompare(PWM_CHANNEL_W, FocVfStart_PwmCompare.compare_w);

}

/**
 * @brief 执行一次V/F启动预定位控制。
 *
 * @note 本函数在10 kHz中断中调用，执行时间有界且不阻塞。
 */
void FocVfStart_Preposition(void)
{
    FocVfStart_ElectricalAngle = 0U;
    FocVfStart_Run(FOC_VF_START_ALIGN_VOLTAGE_V, 0.0F);

    if (FocVfStart_PrepositionTick < FOC_VF_START_PREPOSITION_TICK_COUNT)
    {
        FocVfStart_PrepositionTick++;
    }
    else
    {
        FocVfStart_PrepositionComplete = true;
    }
}

/**
 * @brief 获取V/F预定位完成状态。
 *
 * @return 预定位时间达到宏定义值时返回true，否则返回false。
 */
bool FocVfStart_IsPrepositionComplete(void)
{
    return FocVfStart_PrepositionComplete;
}

/**
 * @brief 复位V/F开环启动状态。
 */
void FocVfStart_Reset(void)
{
    FocSmObserver_Reset();
    FocVfStart_ObserverInitialized = false;
    FocVfStart_ElectricalAngle = 0U;
    FocVfStart_ElectricalFrequencyHz = 0.0F;
    FocVfStart_PrepositionTick = 0U;
    FocVfStart_PrepositionComplete = false;
    FocVfStart_VoltageDq.Vd = 0.0F;
    FocVfStart_VoltageDq.Vq = 0.0F;
}

/**
 * @brief 执行一次V/F开环强拖控制。
 *
 * @note 本函数在10 kHz中断中调用，执行时间有界且不阻塞。
 */
void FocVfStart_ForceDrag(void)
{
    float voltage_magnitude_v = 0.0F;

    if (FocVfStart_ObserverInitialized == false)
    {
        FocSmObserver_Init();
        FocVfStart_ObserverInitialized = true;
    }
    else
    {
        /* 观测器已经初始化。 */
    }

    //FocIfStart_GetPhaseCurrent(&FocVfStart_PhaseCurrent);
    //FocAlgorithm_Clarke(&FocVfStart_PhaseCurrent,
                        // &FocVfStart_CurrentAlphaBeta);
    //(void)FocSmObserver_Update(FocVfStart_CurrentAlphaBeta.I_alpha,
                            //    FocVfStart_CurrentAlphaBeta.I_beta,
                            //    FocVfStart_PreviousVoltageAlphaBeta.V_alpha,
                            //    FocVfStart_PreviousVoltageAlphaBeta.V_beta,
                            //    FocVfStart_Vbus);
    //(void)FocSmObserver_GetOutput(&FocVfStart_ObserverOutput);
    //(void)EncoderService_GetOutput(&EncoderService_Output);

    FocVfStart_UpdateFrequency();   /**< 更新电频率 */
    FocAlgorithm_GenerateAngle(FocVfStart_ElectricalFrequencyHz,&FocVfStart_ElectricalAngle);   /**< 生成电角 */
    voltage_magnitude_v = FocVfStart_CalculateVoltage(FocVfStart_ElectricalFrequencyHz);   /**< 计算电压幅值 */

    FocVfStart_Run(
        FOC_VF_START_D_AXIS_RATIO * voltage_magnitude_v,
        FOC_VF_START_Q_AXIS_RATIO * voltage_magnitude_v);   /**< 应用电压 */
    //Test_SendJustFloat(FocVfStart_ElectricalAngle,FocVfStart_ObserverOutput.electrical_angle_phase,EncoderService_Output.electrical_angle_phase);     
}



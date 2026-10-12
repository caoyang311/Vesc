#ifndef FOC_ALGORITHM_H
#define FOC_ALGORITHM_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @file FocAlgorithm.h
 * @brief FOC通用算法接口声明。
 */

typedef struct
{
    float phase_u;
    float phase_v;
    float phase_w;
} Foc_PhaseCurrentType;

typedef struct
{
    float I_alpha;
    float I_beta;
} Foc_Curr_AlphaBetaType;

typedef struct
{
    float V_alpha;
    float V_beta;
} Foc_Volt_AlphaBetaType;

typedef struct
{
    float Id;
    float Iq;
} Foc_Curr_DqType;

typedef struct
{
    float Vd;
    float Vq;
} Foc_Volt_DqType;

typedef struct
{
    float MaxDVoltage;
    float MaxQVoltage;
} Foc_VoltageLimitType;

typedef enum
{
    FOC_DECOUPLING_DISABLED = 0U,/** @brief 禁用去耦前馈 */
    FOC_DECOUPLING_CROSS = 1U,/** @brief 交叉去耦前馈 */
    FOC_DECOUPLING_BEMF = 2U,/** @brief BEMF去耦前馈 */
    FOC_DECOUPLING_CROSS_BEMF = 3U/** @brief 交叉BEMF去耦前馈 */
} Foc_DecouplingModeType;

typedef struct
{
    float sine;
    float cosine;
} Foc_TrigType;

typedef struct
{
    uint16_t compare_u;
    uint16_t compare_v;
    uint16_t compare_w;
} Foc_PwmCompareType;

/**
 * @brief 对完整三相电流执行Clarke变换。
 *
 * @param[in] phase_current 三相电流输入，指针不得为NULL。
 * @param[out] alpha_beta Alpha-Beta电流输出，指针不得为NULL。
 */
void FocAlgorithm_Clarke(const Foc_PhaseCurrentType * phase_current,
                         Foc_Curr_AlphaBetaType * alpha_beta);

/**
 * @brief 执行电流Park变换。
 *
 * @param[in] alpha_beta Alpha-Beta电流输入，指针不得为NULL。
 * @param[in] trig 电角度正弦与余弦值，指针不得为NULL。
 * @param[out] dq D-Q电流输出，指针不得为NULL。
 */
void FocAlgorithm_Park(const Foc_Curr_AlphaBetaType * alpha_beta,
                       const Foc_TrigType * trig,
                       Foc_Curr_DqType * dq);

/**
 * @brief 执行电压反Park变换。
 *
 * @param[in] dq D-Q电压输入，指针不得为NULL。
 * @param[in] trig 电角度正弦与余弦值，指针不得为NULL。
 * @param[out] alpha_beta Alpha-Beta电压输出，指针不得为NULL。
 */
void FocAlgorithm_InversePark(const Foc_Volt_DqType * dq,
                              const Foc_TrigType * trig,
                              Foc_Volt_AlphaBetaType * alpha_beta);

/**
 * @brief 应用去耦前馈。
 *
 * 根据当前dq电流、电角速度、电感值、磁通量、去耦模式和dq电压，
 * 计算并应用去耦前馈电压。
 *
 * @param[in] current_dq 当前dq电流，指针不得为NULL。
 * @param[in] electrical_speed_rad_s 电角速度，单位为rad/s。
 * @param[in] ld_h 电感值，单位为H。
 * @param[in] lq_h 电感值，单位为H。
 * @param[in] flux_linkage_wb 磁通量，单位为Wb。
 * @param[in] mode 去耦模式。
 * @param[inout] voltage_dq 输出dq电压，指针不得为NULL。
 */
void FocAlgorithm_DecouplingFeedforward(
    const Foc_Curr_DqType * current_dq,
    float electrical_speed_rad_s,
    float ld_h,
    float lq_h,
    float flux_linkage_wb,
    Foc_DecouplingModeType mode,
    Foc_Volt_DqType * voltage_dq);

/**
 * @brief 按母线电压对D/Q轴电压执行圆限幅。
 *
 * 当电压矢量幅值超过母线电压除以根号3时，保持矢量方向并按比例缩小。
 *
 * @param[in,out] voltage_dq D/Q轴电压，单位V，不得为NULL。
 * @param[in] dc_bus_voltage_v 母线电压，单位V，必须大于0。
 *
 * @return 输入有效并完成限幅处理时返回true，否则返回false。
 */
void FocAlgorithm_LimitVoltageCircle(Foc_Volt_DqType * voltage_dq,
                                     float dc_bus_voltage_v);

/**
 * @brief 按D轴优先策略限制D/Q轴电压。
 *
 * @param[in,out] voltage_dq D/Q轴最终电压，单位V，不得为NULL。
 * @param[in] dc_bus_voltage_v 母线电压，单位V，必须大于0。
 * @param[in] max_duty 最大调制占空比。
 * @param[in] overmodulation_factor 过调制系数。
 * @param[in] d_axis_voltage_ratio D轴电压分配比例。
 * @return 参数有效时返回true，否则返回false。
 */
void FocAlgorithm_LimitVoltage(
    Foc_Volt_DqType * voltage_dq,
    float dc_bus_voltage_v,
    float max_duty,
    float overmodulation_factor,
    float d_axis_voltage_ratio,
    Foc_VoltageLimitType * voltage_limit);



/**
 * @brief 根据电角频率生成电角度相位。
 *
 * @param[in] frequency_hz 电角频率，单位Hz，有效范围为0至5000Hz。
 * @param[out] phase 完整电周期相位，指针不得为NULL。
 *
 * @note 使用内部静态累加器，不可重入，应以10kHz周期调用。
 */
void FocAlgorithm_GenerateAngle(float frequency_hz,
                                uint16_t * phase);

/**
 * @brief 根据Alpha-Beta电压执行七段式SVPWM。
 *
 * @param[in] alpha_beta Alpha-Beta电压输入，指针不得为NULL。
 * @param[in] dc_bus_voltage 实时母线电压，单位V，指针不得为NULL。
 * @param[out] compare 三相PWM比较值，指针不得为NULL。
 */
 void FocAlgorithm_Svpwm(const Foc_Volt_AlphaBetaType * alpha_beta,
                        const float * dc_bus_voltage,
                        Foc_PwmCompareType * compare);
/**
 * @brief 获取最近一次SVPWM计算得到的扇区。
 *
 * @return 当前扇区，范围为1至6。
 */
uint8_t FocAlgorithm_GetSector(void);

#endif /* FOC_ALGORITHM_H */

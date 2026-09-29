/**
 * @file FocSmObserver.c
 * @brief 反电动势滑膜观测器与PLL实现。
 */

#include "FocSmObserver.h"

#include <math.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#define FOC_SMO_PLL_TWO_PI                         (6.2831853072F)
#define FOC_SMO_PLL_PHASE_SCALE                   (65536.0F / FOC_SMO_PLL_TWO_PI)/** @brief 2π */
#define FOC_SMO_PLL_SAMPLE_PERIOD_S                (0.0001F)/** @brief 采样周期 */
#define FOC_SMO_PLL_POLE_PAIRS                     (4.0F)/** @brief 电机极对数 */
#define FOC_SMO_PLL_MOTOR_RESISTANCE_OHM           (0.51F)/** @brief 电机相电阻 */
#define FOC_SMO_PLL_MOTOR_INDUCTANCE_H             (0.000295F)/** @brief 电机相电感 */
#define FOC_SMO_PLL_VBUS_MIN_V                    (1.0F)/** @brief 母线电压最小值 */
#define FOC_SMO_PLL_DC_BUS_DEFAULT_V              (24.0F)/** @brief DC总电压默认值 */
#define FOC_SMO_PLL_SLIDING_GAIN_V                 (24.0F)/** @brief 反电动势滑膜观测器增益 */
#define FOC_SMO_PLL_BOUNDARY_LAYER_A              (0.05F)/** @brief 反电动势滑膜观测器边界层A */
#define FOC_SMO_PLL_BEMF_LPF_CUTOFF_HZ            (500.0F)/** @brief 反电动势滑膜观测器低通滤波截止频率 */
#define FOC_SMO_PLL_BANDWIDTH_HZ                  (100.0F)/** @brief 反电动势滑膜观测器带宽 */
#define FOC_SMO_PLL_DAMPING_RATIO                 (0.707F)/** @brief 反电动势滑膜观测器阻尼比 */
#define FOC_SMO_PLL_MIN_BEMF_V                    (0.5F)/** @brief 反电动势滑膜观测器最小反电动势 */
#define FOC_SMO_PLL_MAX_BEMF_V                    (0.5F)/** @brief 反电动势滑膜观测器最大反电动势 */
#define FOC_SMO_PLL_LOCK_ERROR_LIMIT              (0.15F)/** @brief 反电动势滑膜观测器锁误差限制 */
#define FOC_SMO_PLL_LOCK_CONFIRM_COUNT            (100U)/** @brief 反电动势滑膜观测器锁确认计数 */
#define FOC_SMO_PLL_UNLOCK_CONFIRM_COUNT          (20U)/** @brief 反电动势滑膜观测器解锁确认计数 */
#define FOC_SMO_PLL_PI_LIMIT_RAD_S               (FOC_SMO_PLL_TWO_PI * 500.0F)/** @brief 反电动势滑膜观测器积分限制 */

#define FOC_SMO_PLL_LPF_COEFFICIENT \
    ((FOC_SMO_PLL_SAMPLE_PERIOD_S * FOC_SMO_PLL_TWO_PI * FOC_SMO_PLL_BEMF_LPF_CUTOFF_HZ) / \
     (1.0F + (FOC_SMO_PLL_SAMPLE_PERIOD_S * FOC_SMO_PLL_TWO_PI * FOC_SMO_PLL_BEMF_LPF_CUTOFF_HZ)))/** @brief 反电动势滑膜观测器低通滤波系数 */
#define FOC_SMO_PLL_NATURAL_FREQUENCY_RAD_S \
    (FOC_SMO_PLL_TWO_PI * FOC_SMO_PLL_BANDWIDTH_HZ)/** @brief 反电动势滑膜观测器自然频率 */
#define FOC_SMO_PLL_KP \
    (2.0F * FOC_SMO_PLL_DAMPING_RATIO * FOC_SMO_PLL_NATURAL_FREQUENCY_RAD_S)/** @brief 反电动势滑膜观测器增益 */
#define FOC_SMO_PLL_KI \
    (FOC_SMO_PLL_NATURAL_FREQUENCY_RAD_S * FOC_SMO_PLL_NATURAL_FREQUENCY_RAD_S)/** @brief 反电动势滑膜观测器积分增益 */

typedef struct
{
    float current_alpha_hat_a;
    float current_beta_hat_a;
    float bemf_alpha_v;
    float bemf_beta_v;
    float pll_integral_rad_s;
    float electrical_angle_rad;
    float electrical_speed_rad_s;
    uint32_t lock_count;
    uint32_t unlock_count;
    bool is_locked;
    bool is_valid;
} FocSmObserver_StateType;

static FocSmObserver_StateType FocSmObserver_State =
{
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0U, 0U, false, false
};

static float FocSmObserver_Clamp(float value, float lower_limit, float upper_limit);
static float FocSmObserver_Saturate(float value);
static float FocSmObserver_NormalizeAngle(float angle_rad);

/**
 * @brief 限制浮点值范围。
 *
 * @param[in] value 待限制值。
 * @param[in] lower_limit 下限。
 * @param[in] upper_limit 上限。
 *
 * @return 限制后的值。
 */
static float FocSmObserver_Clamp(float value, float lower_limit, float upper_limit)
{
    float result = value;

    if (value < lower_limit)
    {
        result = lower_limit;
    }
    else if (value > upper_limit)
    {
        result = upper_limit;
    }
    else
    {
        /* 数值位于限幅范围内。 */
    }

    return result;
}

/**
 * @brief 计算边界层滑膜饱和函数。
 *
 * @param[in] value 归一化滑膜误差。
 *
 * @return 范围为-1至1的饱和值。
 */
static float FocSmObserver_Saturate(float value)
{
    return FocSmObserver_Clamp(value, -1.0F, 1.0F);
}

/**
 * @brief 将电角度归一化到0至2π区间。
 *
 * @param[in] angle_rad 输入电角度，单位rad。
 *
 * @return 归一化后的电角度，单位rad。
 */
static float FocSmObserver_NormalizeAngle(float angle_rad)
{
    float result = angle_rad;

    while (result >= FOC_SMO_PLL_TWO_PI)
    {
        result -= FOC_SMO_PLL_TWO_PI;
    }

    while (result < 0.0F)
    {
        result += FOC_SMO_PLL_TWO_PI;
    }

    return result;
}

/**
 * @brief 初始化滑膜观测器与PLL。
 */
void FocSmObserver_Init(void)
{
    FocSmObserver_Reset();
}

/**
 * @brief 复位滑膜观测器与PLL状态。
 */
void FocSmObserver_Reset(void)
{
    FocSmObserver_State.current_alpha_hat_a = 0.0F;
    FocSmObserver_State.current_beta_hat_a = 0.0F;
    FocSmObserver_State.bemf_alpha_v = 0.0F;
    FocSmObserver_State.bemf_beta_v = 0.0F;
    FocSmObserver_State.pll_integral_rad_s = 0.0F;
    FocSmObserver_State.electrical_angle_rad = 0.0F;
    FocSmObserver_State.electrical_speed_rad_s = 0.0F;
    FocSmObserver_State.lock_count = 0U;
    FocSmObserver_State.unlock_count = 0U;
    FocSmObserver_State.is_locked = false;
    FocSmObserver_State.is_valid = false;
}

/**
 * @brief 执行一次滑膜观测器和PLL更新。
 *
 * @param[in] current_alpha_a Alpha轴电流，单位A。
 * @param[in] current_beta_a Beta轴电流，单位A。
 * @param[in] voltage_alpha_v 上一周期Alpha轴电压，单位V。
 * @param[in] voltage_beta_v 上一周期Beta轴电压，单位V。
 * @param[in] dc_bus_voltage_v 母线电压，单位V，必须大于0。
 *
 * @return 输入有效且更新完成时返回true，否则返回false。
 */
bool FocSmObserver_Update(float current_alpha_a,
                          float current_beta_a,
                          float voltage_alpha_v,
                          float voltage_beta_v,
                          float dc_bus_voltage_v)
{
    float error_alpha_a;
    float error_beta_a;
    float sliding_alpha_v;
    float sliding_beta_v;
    float bemf_magnitude_v;
    float sine_value;
    float cosine_value;
    float pll_error;
    float natural_frequency;
    bool result = false;

    if (isfinite(current_alpha_a) &&
        isfinite(current_beta_a) &&
        isfinite(voltage_alpha_v) &&
        isfinite(voltage_beta_v) &&
        isfinite(dc_bus_voltage_v) &&
        (dc_bus_voltage_v > 0.0F))
    {
        /*
         * 电流观测误差：
         *   S_alpha = I_alpha_hat - I_alpha
         *   S_beta  = I_beta_hat  - I_beta
         * 其中I_hat为观测电流，I为实际采样电流。
         */
        error_alpha_a = FocSmObserver_State.current_alpha_hat_a - current_alpha_a;
        error_beta_a = FocSmObserver_State.current_beta_hat_a - current_beta_a;

        /*
         * 边界层滑膜控制量：
         *   Z_alpha = K_slide * sat(S_alpha / Phi)
         *   Z_beta  = K_slide * sat(S_beta  / Phi)
         * 使用sat函数替代sign函数，减小滑膜抖振。
         */
        sliding_alpha_v = FOC_SMO_PLL_SLIDING_GAIN_V *
                          FocSmObserver_Saturate(error_alpha_a / FOC_SMO_PLL_BOUNDARY_LAYER_A);
        sliding_beta_v = FOC_SMO_PLL_SLIDING_GAIN_V *
                        FocSmObserver_Saturate(error_beta_a / FOC_SMO_PLL_BOUNDARY_LAYER_A);

        /*
         * 离散化定子电流模型：
         *   I_hat(k+1) = I_hat(k) + Ts/L *
         *                (V - R*I_hat(k) - Z)
         * 其中V为上一周期Alpha/Beta轴电压，R为相电阻，L为相电感。
         */
        FocSmObserver_State.current_alpha_hat_a +=
            (FOC_SMO_PLL_SAMPLE_PERIOD_S / FOC_SMO_PLL_MOTOR_INDUCTANCE_H) *
            (voltage_alpha_v -
             (FOC_SMO_PLL_MOTOR_RESISTANCE_OHM * FocSmObserver_State.current_alpha_hat_a) -
             sliding_alpha_v);
        FocSmObserver_State.current_beta_hat_a +=
            (FOC_SMO_PLL_SAMPLE_PERIOD_S / FOC_SMO_PLL_MOTOR_INDUCTANCE_H) *
            (voltage_beta_v -
             (FOC_SMO_PLL_MOTOR_RESISTANCE_OHM * FocSmObserver_State.current_beta_hat_a) -
             sliding_beta_v);

        /*
         * 反电动势低通滤波：
         *   E_hat(k) = E_hat(k-1) + K_lpf * (Z(k) - E_hat(k-1))
         * 滤除滑膜控制量中的高频抖振分量。
         */
        FocSmObserver_State.bemf_alpha_v +=
            FOC_SMO_PLL_LPF_COEFFICIENT *
            (sliding_alpha_v - FocSmObserver_State.bemf_alpha_v);
        FocSmObserver_State.bemf_beta_v +=
            FOC_SMO_PLL_LPF_COEFFICIENT *
            (sliding_beta_v - FocSmObserver_State.bemf_beta_v);

        /* 计算反电动势矢量幅值，用于判断当前是否具备可靠锁相条件。 */
        bemf_magnitude_v = sqrtf((FocSmObserver_State.bemf_alpha_v * FocSmObserver_State.bemf_alpha_v) +
                                 (FocSmObserver_State.bemf_beta_v * FocSmObserver_State.bemf_beta_v));
        if (bemf_magnitude_v >= FOC_SMO_PLL_MIN_BEMF_V)
        {
            /*
             * PLL相位误差：
             *   E_q = -E_alpha*sin(theta_hat) + E_beta*cos(theta_hat)
             *   error = E_q / |E|
             * 归一化误差不受反电动势幅值变化直接影响。
             */
            sine_value = sinf(FocSmObserver_State.electrical_angle_rad);
            cosine_value = cosf(FocSmObserver_State.electrical_angle_rad);
            pll_error = ((-FocSmObserver_State.bemf_alpha_v * sine_value) +
                         (FocSmObserver_State.bemf_beta_v * cosine_value)) / bemf_magnitude_v;

            /*
             * PLL积分环节：
             *   integral(k) = integral(k-1) + Ki_pll*error*Ts
             * 积分状态限幅，避免异常输入导致积分饱和。
             */
            natural_frequency = FOC_SMO_PLL_KI * pll_error * FOC_SMO_PLL_SAMPLE_PERIOD_S;
            FocSmObserver_State.pll_integral_rad_s =
                FocSmObserver_Clamp(FocSmObserver_State.pll_integral_rad_s + natural_frequency,
                                    -FOC_SMO_PLL_PI_LIMIT_RAD_S,
                                    FOC_SMO_PLL_PI_LIMIT_RAD_S);
            /*
             * PLL比例加积分输出：
             *   omega_hat = Kp_pll*error + integral
             * 输出为估计电角速度，单位rad/s。
             */
            FocSmObserver_State.electrical_speed_rad_s =
                FocSmObserver_Clamp((FOC_SMO_PLL_KP * pll_error) +
                                    FocSmObserver_State.pll_integral_rad_s,
                                    -FOC_SMO_PLL_PI_LIMIT_RAD_S,
                                    FOC_SMO_PLL_PI_LIMIT_RAD_S);

            /*
             * 电角度积分：
             *   theta_hat(k+1) = theta_hat(k) + omega_hat*Ts
             * 通过归一化将角度保持在[0, 2π)范围内。
             */
            FocSmObserver_State.electrical_angle_rad =
                FocSmObserver_NormalizeAngle(FocSmObserver_State.electrical_angle_rad +
                                             (FocSmObserver_State.electrical_speed_rad_s *
                                              FOC_SMO_PLL_SAMPLE_PERIOD_S));
            FocSmObserver_State.is_valid = true;
            if (fabsf(pll_error) <= FOC_SMO_PLL_LOCK_ERROR_LIMIT)
            {
                if (FocSmObserver_State.lock_count < FOC_SMO_PLL_LOCK_CONFIRM_COUNT)
                {
                    FocSmObserver_State.lock_count++;
                }
                else
                {
                    /* 锁定计数已达到上限。 */
                }
                FocSmObserver_State.unlock_count = 0U;
                if (FocSmObserver_State.lock_count >= FOC_SMO_PLL_LOCK_CONFIRM_COUNT)
                {
                    FocSmObserver_State.is_locked = true;
                }
                else
                {
                    /* 继续确认PLL锁定。 */
                }
            }
            else
            {
                FocSmObserver_State.lock_count = 0U;
                if (FocSmObserver_State.unlock_count < FOC_SMO_PLL_UNLOCK_CONFIRM_COUNT)
                {
                    FocSmObserver_State.unlock_count++;
                }
                else
                {
                    /* 失锁计数已达到上限。 */
                }
                if (FocSmObserver_State.unlock_count >= FOC_SMO_PLL_UNLOCK_CONFIRM_COUNT)
                {
                    FocSmObserver_State.is_locked = false;
                }
                else
                {
                    /* 继续保持当前锁定状态。 */
                }
            }
            result = true;
        }
        else
        {
            FocSmObserver_State.is_valid = false;
            FocSmObserver_State.is_locked = false;
            FocSmObserver_State.lock_count = 0U;
        }
    }
    else
    {
        FocSmObserver_State.is_valid = false;
    }

    return result;
}

/**
 * @brief 获取滑膜观测器与PLL输出。
 *
 * @param[out] output 输出快照，不得为NULL。
 *
 * @return 输出指针有效且观测器输出有效时返回true，否则返回false。
 */
bool FocSmObserver_GetOutput(Foc_SmoObserverOutputType * output)
{
    bool result = false;

    if (output != NULL)
    {
        output->electrical_angle_rad = FocSmObserver_State.electrical_angle_rad;
        output->electrical_angle_phase =
            (uint16_t)(FocSmObserver_State.electrical_angle_rad * FOC_SMO_PLL_PHASE_SCALE);
        output->electrical_speed_rad_s = FocSmObserver_State.electrical_speed_rad_s;
        output->mechanical_speed_rpm =
            (FocSmObserver_State.electrical_speed_rad_s * 60.0F) /
            (FOC_SMO_PLL_TWO_PI * FOC_SMO_PLL_POLE_PAIRS);
        output->bemf_alpha_v = FocSmObserver_State.bemf_alpha_v;
        output->bemf_beta_v = FocSmObserver_State.bemf_beta_v;
        output->is_locked = FocSmObserver_State.is_locked;
        output->is_valid = FocSmObserver_State.is_valid;
        result = FocSmObserver_State.is_valid;
    }
    else
    {
        /* 输出指针无效。 */
    }

    return result;
}

/**
 * @brief 获取PLL锁定状态。
 *
 * @return PLL锁定时返回true，否则返回false。
 */
bool FocSmObserver_IsLocked(void)
{
    return FocSmObserver_State.is_locked;
}

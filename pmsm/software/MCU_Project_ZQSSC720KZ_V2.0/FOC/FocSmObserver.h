/**
 * @file FocSmObserver.h
 * @brief 反电动势滑膜观测器与PLL接口。
 */

#ifndef FOC_SM_OBSERVER_H
#define FOC_SM_OBSERVER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 滑膜观测器与PLL输出数据。
 */
typedef struct
{
    float electrical_angle_rad;/** @brief 电角度，单位弧度 */
    uint16_t electrical_angle_phase;/** @brief 电角度相位，范围0~65535 */
    float electrical_speed_rad_s;/** @brief 电速度，单位弧度/秒 */
    float mechanical_speed_rpm;/** @brief 机械速度，单位转/秒 */
    float bemf_alpha_v;/** @brief Alpha轴BEMF电压，单位V */
    float bemf_beta_v;/** @brief Beta轴BEMF电压，单位V */
    bool is_locked;/** @brief PLL是否锁定 */
    bool is_valid;/** @brief 输出是否有效 */
} Foc_SmoObserverOutputType;

/**
 * @brief 初始化滑膜观测器与PLL。
 *
 * 初始化所有观测器、滤波器和PLL内部状态。该接口当前不由控制链调用。
 */
void FocSmObserver_Init(void);

/**
 * @brief 复位滑膜观测器与PLL状态。
 *
 * @note 调用后输出无效，下一次更新成功后重新建立估计状态。
 */
void FocSmObserver_Reset(void);

/**
 * @brief 执行一次滑膜观测器和PLL更新。
 *
 * @param[in] current_alpha_a Alpha轴电流，单位A。
 * @param[in] current_beta_a Beta轴电流，单位A。
 * @param[in] voltage_alpha_v 上一控制周期Alpha轴电压，单位V。
 * @param[in] voltage_beta_v 上一控制周期Beta轴电压，单位V。
 * @param[in] dc_bus_voltage_v 母线电压，单位V，必须大于0。
 *
 * @return 输入有效且更新完成时返回true，否则返回false。
 *
 * @note 本接口设计用于10 kHz快速控制上下文，当前暂不调用。
 */
bool FocSmObserver_Update(float current_alpha_a,
                          float current_beta_a,
                          float voltage_alpha_v,
                          float voltage_beta_v,
                          float dc_bus_voltage_v);

/**
 * @brief 获取滑膜观测器与PLL输出。
 *
 * @param[out] output 输出快照，不得为NULL。
 *
 * @return 输出指针有效且观测器输出有效时返回true，否则返回false。
 */
bool FocSmObserver_GetOutput(Foc_SmoObserverOutputType * output);

/**
 * @brief 获取PLL锁定状态。
 *
 * @return PLL连续满足锁定条件时返回true，否则返回false。
 */
bool FocSmObserver_IsLocked(void);

#endif /* FOC_SM_OBSERVER_H */

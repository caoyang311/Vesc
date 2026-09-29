#ifndef FOC_PI_CONTROLLER_H
#define FOC_PI_CONTROLLER_H

#include <stdbool.h>

/**
 * @brief 初始化D轴和Q轴并联型PI控制器。
 *
 * 根据模块内部的电机参数和电流环参数计算Kp、Ki、采样周期及积分限幅，
 * 并清零D轴和Q轴积分状态。
 */
void FocPiController_Init(void);

/**
 * @brief 清零D轴和Q轴PI控制器积分状态。
 */
void FocPiController_Reset(void);

/**
 * @brief 执行一次D轴并联型PI电流控制。
 *
 * @param[in] feedback_current_a D轴反馈电流，单位A。
 * @param[in] reference_current_a D轴参考电流，单位A。
 * @param[in] dc_bus_voltage_v 母线电压，单位V，必须大于0。
 * @param[out] output_voltage_v D轴输出电压，单位V，不得为NULL。
 *
 * @return 控制器和输入有效时返回true，否则返回false。
 * @note 本接口供10 kHz中断调用，仅实施单轴积分限幅和输出限幅。
 */
void FocPiController_UpdateD(float feedback_current_a,
                             float reference_current_a,
                             float dc_bus_voltage_v,
                             float * output_voltage_v);

/**
 * @brief 执行一次Q轴并联型PI电流控制。
 *
 * @param[in] feedback_current_a Q轴反馈电流，单位A。
 * @param[in] reference_current_a Q轴参考电流，单位A。
 * @param[in] dc_bus_voltage_v 母线电压，单位V，必须大于0。
 * @param[out] output_voltage_v Q轴输出电压，单位V，不得为NULL。
 *
 * @return 控制器和输入有效时返回true，否则返回false。
 * @note 本接口供10 kHz中断调用，仅实施单轴积分限幅和输出限幅。
 */
void FocPiController_UpdateQ(float feedback_current_a,
                             float reference_current_a,
                             float dc_bus_voltage_v,
                             float * output_voltage_v);

#endif /* FOC_PI_CONTROLLER_H */

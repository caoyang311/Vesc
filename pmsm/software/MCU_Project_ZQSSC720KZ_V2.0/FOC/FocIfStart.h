/**
 * @file FocIfStart.h
 * @brief I/F开环启动算法模块接口。
 */

#ifndef FOC_IF_START_H
#define FOC_IF_START_H

#include <stdbool.h>
#include "FocAlgorithm.h"

/**
 * @brief 复位I/F开环启动状态和电流PI积分状态。
 */
void FocIfStart_Reset(void);

/**
 * @brief 初始化三相电流静态偏置标定。
 *
 * 清零标定累加器，并准备在初始化状态中逐周期采集零电流ADC样本。
 */
void FocIfStart_InitCurrentOffsetCalibration(void);

/**
 * @brief 累积三相零电流采样并计算静态偏置。
 *
 * @return 静态偏置标定完成时返回true，否则返回false。
 * @note 仅应在PWM未输出且电机无相电流时调用。
 */
bool FocIfStart_CalibratePhaseCurrentOffset(void);

/**
 * @brief 读取已校准的三相电流。
 *
 * @param[out] phase_current 三相电流输出，单位A，不得为NULL。
 */
void FocIfStart_GetPhaseCurrent(Foc_PhaseCurrentType * phase_current);

/**
 * @brief 执行一次I/F启动预定位控制。
 *
 * 首次调用时开启PWM互补输出和ADC触发，随后固定开环电角度并执行预定位电流控制。
 *
 * @note 本接口由10 kHz中断调用，执行时间有界且不阻塞。
 */
void FocIfStart_Preposition(void);

/**
 * @brief 获取I/F预定位完成状态。
 *
 * @return 预定位斜坡和维持时间均完成时返回true，否则返回false。
 */
bool FocIfStart_IsPrepositionComplete(void);

/**
 * @brief 执行一次I/F开环强拖控制。
 *
 * D轴电流给定斜坡降至零，同时建立Q轴电流和开环电频率斜坡。
 *
 * @note 本接口由10 kHz中断调用，执行时间有界且不阻塞。
 */
void FocIfStart_ForceDrag(void);

#endif /* FOC_IF_START_H */

/**
 * @file FocVfStart.h
 * @brief V/F开环启动算法模块接口。
 */

#ifndef FOC_VF_START_H
#define FOC_VF_START_H

#include <stdbool.h>

/**
 * @brief 执行一次V/F启动预定位控制。
 *
 * @note 本接口由10 kHz中断调用，执行时间有界且不阻塞。
 */
void FocVfStart_Preposition(void);

/**
 * @brief 获取V/F预定位完成状态。
 *
 * @return 预定位时间达到宏定义值时返回true，否则返回false。
 */
bool FocVfStart_IsPrepositionComplete(void);

/**
 * @brief 复位V/F开环启动状态。
 */
void FocVfStart_Reset(void);

/**
 * @brief 执行一次V/F开环强拖控制。
 *
 * @note 本接口由10 kHz中断调用，执行时间有界且不阻塞。
 */
void FocVfStart_ForceDrag(void);


#endif /* FOC_VF_START_H */

#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdbool.h>

/**
 * @brief 初始化应用、编码器、ADC采集链路和系统服务。
 *
 * @return 编码器、测试模块、ADC采集、任务配置和系统时间基均初始化
 *         成功时返回true，否则返回false。
 *
 * @post 成功后，编码器、互补PWM、ADC采集、周期调度和时间基均已启动。
 */
bool System_Init(void);

/**
 * @brief Executes one iteration of the cooperative system scheduler.
 *
 * This function shall be called continuously from the main loop. It does not
 * block and shall not be called from interrupt context.
 *
 * @pre System_Init() has returned true.
 */
void System_MainFunction(void);

#endif /* SYSTEM_H */

/**
 * @file SystemTime.h
 * @brief 系统毫秒时间基接口。
 */
#ifndef PMSM_SYSTEM_TIME_H
#define PMSM_SYSTEM_TIME_H

#include <stdbool.h>
#include <stdint.h>

void SystemTime_Init(void);
void SystemTime_Start(void);
uint32_t SystemTime_GetTimeMs(void);

#endif /* PMSM_SYSTEM_TIME_H */

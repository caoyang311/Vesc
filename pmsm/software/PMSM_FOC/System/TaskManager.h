/**
 * @file TaskManager.h
 * @brief 协作式任务调度器接口。
 */
#ifndef PMSM_TASK_MANAGER_H
#define PMSM_TASK_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

typedef void (*TaskManager_TaskFunction)(void);

typedef struct
{
    TaskManager_TaskFunction Function;
    uint32_t PeriodMs;
    uint32_t StartDelayMs;
} TaskManager_TaskConfigType;

void TaskManager_Init(void);
void TaskManager_Run(void);

#endif /* PMSM_TASK_MANAGER_H */

/**
 * @file TaskConfig.h
 * @brief 系统周期任务配置接口。
 */
#ifndef PMSM_TASK_CONFIG_H
#define PMSM_TASK_CONFIG_H

#include "TaskManager.h"

#define TASK_CONFIG_MAX_TASKS (7U)

const TaskManager_TaskConfigType *TaskConfig_GetTasks(uint16_t *Count);

#endif /* PMSM_TASK_CONFIG_H */

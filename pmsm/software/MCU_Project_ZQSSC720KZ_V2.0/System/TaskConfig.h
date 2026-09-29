#ifndef TASK_CONFIG_H
#define TASK_CONFIG_H

#include "TaskManager.h"

#include <stdint.h>

#define TASK_CONFIG_MAX_TASKS ((uint16_t)7U)

/**
 * @brief Returns the static task configuration table.
 *
 * @param[out] count Receives the number of configured tasks. The pointer
 *                   shall not be NULL.
 *
 * @return Pointer to the read-only task configuration table.
 */
const TaskManager_TaskConfig *TaskConfig_GetTasks(uint16_t *count);

#endif

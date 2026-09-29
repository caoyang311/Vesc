#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

typedef void (*TaskManager_TaskFunction)(void);

typedef struct
{
    TaskManager_TaskFunction function;
    uint32_t period_ms;
    uint32_t start_delay_ms;
} TaskManager_TaskConfig;

/**
 * @brief Initializes task runtime data from the static task configuration.
 *
 * @return true when the configuration is valid; otherwise false.
 *
 * @note Every task shall have a non-null function, a non-zero period, and
 *       period/delay values smaller than the uint32_t half range.
 */
bool TaskManager_Init(void);

/**
 * @brief Executes each due task at most once and advances its deadline.
 *
 * @note This function runs in the main-loop context and shall not be called
 *       concurrently or from an interrupt service routine.
 */
void TaskManager_Run(void);

#endif

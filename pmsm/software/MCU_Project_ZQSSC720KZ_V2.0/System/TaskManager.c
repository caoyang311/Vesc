#include "TaskManager.h"
#include "TaskConfig.h"
#include "SystemTime.h"

#include <stddef.h>
#include <stdint.h>

#define TASK_MANAGER_TIME_HALF_RANGE (UINT32_C(0x80000000))

static const TaskManager_TaskConfig *TaskManager_Tasks;
static uint16_t TaskManager_TaskCount;
static uint32_t TaskManager_Deadlines[TASK_CONFIG_MAX_TASKS];
static bool TaskManager_IsInitialized;

/**
 * @brief Determines whether the current time has reached a deadline.
 *
 * @param[in] now Current monotonic time in milliseconds.
 * @param[in] deadline Absolute deadline in milliseconds.
 *
 * @return true when the deadline is due within the uint32_t half range;
 *         otherwise false.
 */
static bool TaskManager_IsDeadlineDue(uint32_t now, uint32_t deadline)
{
    return ((now - deadline) < TASK_MANAGER_TIME_HALF_RANGE);
}

/**
 * @brief Validates one task configuration entry.
 *
 * @param[in] task Pointer to the task configuration. The pointer shall not
 *                 be NULL.
 *
 * @return true when the function, period, and delay are valid; otherwise
 *         false.
 */
static bool TaskManager_IsTaskValid(const TaskManager_TaskConfig *task)
{
    return ((task->function != NULL) &&
            (task->period_ms > 0U) &&
            (task->period_ms < TASK_MANAGER_TIME_HALF_RANGE) &&
            (task->start_delay_ms < TASK_MANAGER_TIME_HALF_RANGE));
}

/**
 * @brief Initializes task runtime data from the static task configuration.
 *
 * @return true when all task entries fit the runtime storage and are valid;
 *         otherwise false.
 */
bool TaskManager_Init(void)
{
    const uint32_t now = SystemTime_GetTimeMs();
    uint16_t index = 0U;
    bool isValid = true;

    TaskManager_IsInitialized = false;
    TaskManager_Tasks = TaskConfig_GetTasks(&TaskManager_TaskCount);

    if ((TaskManager_Tasks == NULL) ||
        (TaskManager_TaskCount == 0U) ||
        (TaskManager_TaskCount > TASK_CONFIG_MAX_TASKS))
    {
        isValid = false;
    }
    else
    {
        for (index = 0U; index < TaskManager_TaskCount; ++index)
        {
            if (TaskManager_IsTaskValid(&TaskManager_Tasks[index]) == false)
            {
                isValid = false;
                break;
            }

            TaskManager_Deadlines[index] = now +
                                           TaskManager_Tasks[index].start_delay_ms;
        }
    }

    TaskManager_IsInitialized = isValid;
    return isValid;
}

/**
 * @brief Executes each due task at most once and advances its deadline.
 *
 * A missed set of periods is skipped with a constant-time calculation, so
 * execution time does not grow linearly with the historical delay.
 */
void TaskManager_Run(void)
{
    const uint32_t now = SystemTime_GetTimeMs();
    uint16_t index = 0U;
    uint32_t elapsed = 0U;
    uint32_t remaining = 0U;
    if (TaskManager_IsInitialized == true)
    {
        for (index = 0U; index < TaskManager_TaskCount; ++index)
        {
            if (TaskManager_IsDeadlineDue(now, TaskManager_Deadlines[index]) == true)
            {
                elapsed = now - TaskManager_Deadlines[index];/* Elapsed time since last execution */
                remaining = TaskManager_Tasks[index].period_ms -(elapsed % TaskManager_Tasks[index].period_ms);/* Remaining time to next execution */
                TaskManager_Tasks[index].function();
                TaskManager_Deadlines[index] = now + remaining;/* Update deadline */
            }
        }
    }
}

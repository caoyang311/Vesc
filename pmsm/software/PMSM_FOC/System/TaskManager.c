/**
 * @file TaskManager.c
 * @brief 协作式任务调度器实现。
 */
#include "TaskManager.h"

#include "SystemTime.h"
#include "TaskConfig.h"

#define TASK_MANAGER_TIME_HALF_RANGE (UINT32_C(0x80000000))

static const TaskManager_TaskConfigType *TaskManager_Tasks = (const TaskManager_TaskConfigType *)0;
static uint16_t TaskManager_TaskCount = 0U;
static uint32_t TaskManager_Deadlines[TASK_CONFIG_MAX_TASKS] = {0U};
/**
 * @brief 检查任务是否已到期。
 * @param Now 当前时间戳（毫秒）。
 * @param Deadline 任务的截止时间（毫秒）。
 * @return true 如果任务已到期，否则返回 false。
 */
static bool TaskManager_IsDeadlineDue(uint32_t Now, uint32_t Deadline)
{
    return ((Now - Deadline) < TASK_MANAGER_TIME_HALF_RANGE);
}
/**
 * @brief 初始化任务调度器。
 */
void TaskManager_Init(void)
{
    uint32_t now = SystemTime_GetTimeMs();
    uint16_t index = 0U;

    TaskManager_Tasks = TaskConfig_GetTasks(&TaskManager_TaskCount);

    for (index = 0U; index < TaskManager_TaskCount; index++)
    {
        TaskManager_Deadlines[index] =
            now + TaskManager_Tasks[index].StartDelayMs;
    }
}
/**
 * @brief 运行任务调度器。
 */
void TaskManager_Run(void)
{
    uint32_t now = SystemTime_GetTimeMs();
    uint32_t elapsed = 0U;
    uint32_t remaining = 0U;
    uint16_t index = 0U;

    for (index = 0U; index < TaskManager_TaskCount; index++)
    {
        if (TaskManager_IsDeadlineDue(now, TaskManager_Deadlines[index]) == true)
        {
            elapsed = now - TaskManager_Deadlines[index];
            remaining = TaskManager_Tasks[index].PeriodMs -
                        (elapsed % TaskManager_Tasks[index].PeriodMs);
            TaskManager_Tasks[index].Function();
            TaskManager_Deadlines[index] = now + remaining;
        }
        else
        {
            /* 当前任务尚未到期。 */
        }
    }
}

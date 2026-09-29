/**
 * @file TaskConfig.c
 * @brief PMSM_FOC 周期任务配置。
 */
#include "TaskConfig.h"
#include "VescPacket.h"
#include "Watchdog.h"
#include "Motor.h"
#include "Key.h"
#include "Led.h"
#include "MotorApp.h"

static void TaskRun_1ms(void);
static void TaskRun_5ms(void);
static void TaskRun_10ms(void);
static void TaskRun_20ms(void);
static void TaskRun_50ms(void);
static void TaskRun_100ms(void);
static void TaskRun_200ms(void);

static const TaskManager_TaskConfigType TaskConfig_Tasks[] =
{
    {TaskRun_1ms, 1U, 0U},
    {TaskRun_5ms, 5U, 0U},
    {TaskRun_10ms, 10U, 0U},
    {TaskRun_20ms, 20U, 0U},
    {TaskRun_50ms, 50U, 0U},
    {TaskRun_100ms, 100U, 0U},
    {TaskRun_200ms, 200U, 0U}
};

#define TASK_CONFIG_TASK_COUNT \
    (sizeof(TaskConfig_Tasks) / sizeof(TaskConfig_Tasks[0]))

_Static_assert(TASK_CONFIG_TASK_COUNT <= TASK_CONFIG_MAX_TASKS,
               "Task configuration exceeds runtime capacity");

const TaskManager_TaskConfigType *TaskConfig_GetTasks(uint16_t *Count)
{
    *Count = (uint16_t)TASK_CONFIG_TASK_COUNT;
    return TaskConfig_Tasks;
}
/**
 * @brief 1ms 周期任务。
 */
static void TaskRun_1ms(void)
{
    Watchdog_Refresh();
}
/**
 * @brief 5ms 周期任务。
 */
static void TaskRun_5ms(void)
{
    Key_Scan();
    MotorApp_MainFunction();
}
/**
 * @brief 10ms 周期任务。
 */
static void TaskRun_10ms(void)
{
    Motor_MainFunction();
    Led_MainFunction();
}
/**
 * @brief 20ms 周期任务。
 */
static void TaskRun_20ms(void)
{
}
/**
 * @brief 50ms 周期任务。
 */
static void TaskRun_50ms(void)
{
}
/**
 * @brief 100ms 周期任务。
 */
static void TaskRun_100ms(void)
{
}
/**
 * @brief 200ms 周期任务。
 */
static void TaskRun_200ms(void)
{
}

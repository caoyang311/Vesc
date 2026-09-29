#include "TaskConfig.h"
#include "Led.h"
#include "Test.h"
#include "MotorControl.h"
#include "EncoderService.h"

/**
 * @brief Executes the 1 ms cooperative task slot.
 *
 * This main-loop task slot is non-blocking and currently contains no tasks.
 */
static void TaskRun_1ms(void);

/**
 * @brief Executes the 5 ms cooperative task slot.
 *
 * This main-loop task slot is non-blocking and currently contains no tasks.
 */
static void TaskRun_5ms(void);

/**
 * @brief Executes the 10 ms cooperative task slot.
 *
 * This main-loop task slot invokes the LED periodic task and shall not block.
 */
static void TaskRun_10ms(void);

/**
 * @brief Executes the 20 ms cooperative task slot.
 *
 * This main-loop task slot is non-blocking and currently contains no tasks.
 */
static void TaskRun_20ms(void);

/**
 * @brief Executes the 50 ms cooperative task slot.
 *
 * This main-loop task slot is non-blocking and currently contains no tasks.
 */
static void TaskRun_50ms(void);

/**
 * @brief Executes the 100 ms cooperative task slot.
 *
 * This main-loop task slot is non-blocking and currently contains no tasks.
 */
static void TaskRun_100ms(void);

/**
 * @brief Executes the 200 ms cooperative task slot.
 *
 * This main-loop task slot is non-blocking and currently contains no tasks.
 */
static void TaskRun_200ms(void);

static const TaskManager_TaskConfig TaskConfig_Tasks[] =
{
    {TaskRun_1ms, 1U, 0U},
    {TaskRun_5ms, 5U, 0U},
    {TaskRun_10ms, 10U, 0U},
    {TaskRun_20ms, 20U, 0U},
    {TaskRun_50ms, 50U, 0U},
    {TaskRun_100ms, 100U, 0U},
    {TaskRun_200ms, 200U, 0U},
};

#define TASK_CONFIG_TASK_COUNT \
    (sizeof(TaskConfig_Tasks) / sizeof(TaskConfig_Tasks[0]))

_Static_assert(TASK_CONFIG_TASK_COUNT <= TASK_CONFIG_MAX_TASKS,
               "Task configuration exceeds runtime capacity");

/**
 * @brief Returns the static task configuration table.
 *
 * @param[out] count Receives the number of configured tasks. The pointer
 *                   shall not be NULL.
 *
 * @return Pointer to the read-only task configuration table.
 */
const TaskManager_TaskConfig *TaskConfig_GetTasks(uint16_t *count)
{
    *count = (uint16_t)TASK_CONFIG_TASK_COUNT;
    return TaskConfig_Tasks;
}

/**
 * @brief Executes the 1 ms cooperative task slot.
 *
 * This function executes in main-loop context, is non-blocking, and currently
 * contains no tasks.
 */
static void TaskRun_1ms(void)
{
}

/**
 * @brief Executes the 5 ms cooperative task slot.
 *
 * 本函数在主循环上下文中每5 ms调用一次编码器速度更新和电机控制任务，
 * 执行过程不得阻塞。
 */
static void TaskRun_5ms(void)
{
    EncoderService_UpdateSpeed();
    MotorControl_Task5ms();
}

/**
 * @brief Executes the 10 ms cooperative task slot.
 *
 * This function executes in main-loop context, invokes Led_Task(), and shall
 * not block.
 */
static void TaskRun_10ms(void)
{
    Led_Task();
    Test_Task();
    MotorControl_Task10ms();
}

/**
 * @brief Executes the 20 ms cooperative task slot.
 *
 * This function executes in main-loop context, is non-blocking, and currently
 * contains no tasks.
 */
static void TaskRun_20ms(void)
{
}

/**
 * @brief Executes the 50 ms cooperative task slot.
 *
 * This function executes in main-loop context, is non-blocking, and currently
 * contains no tasks.
 */
static void TaskRun_50ms(void)
{
}

/**
 * @brief Executes the 100 ms cooperative task slot.
 *
 * This function executes in main-loop context, is non-blocking, and currently
 * contains no tasks.
 */
static void TaskRun_100ms(void)
{
}

/**
 * @brief Executes the 200 ms cooperative task slot.
 *
 * This function executes in main-loop context, is non-blocking, and currently
 * contains no tasks.
 */
static void TaskRun_200ms(void)
{
}

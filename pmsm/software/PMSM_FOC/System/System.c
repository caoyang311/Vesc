/**
 * @file System.c
 * @brief 系统服务初始化与主循环调度实现。
 */
#include "System.h"

#include "SystemTime.h"
#include "TaskManager.h"
/**
 * @brief 初始化系统服务。
 */
void System_Init(void)
{
    SystemTime_Init();
    TaskManager_Init();
    SystemTime_Start();
}
/**
 * @brief 系统主循环。
 */
void System_MainFunction(void)
{
    TaskManager_Run();
}

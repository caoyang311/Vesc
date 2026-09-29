#include "System.h"
#include "AdcService.h"
#include "Led.h"
#include "SystemTime.h"
#include "TaskManager.h"
#include "Test.h"
#include "MotorControl.h"
#include "EncoderService.h"

static const EncoderService_ConfigType System_EncoderConfig =
{
    4000U,
    4U,
    0.005F,
    0U
};

/**
 * @brief 初始化应用和系统服务。
 *
 * 初始化LED、电机控制、编码器、测试模块和系统时间，完成三个ADC校准
 * 并启动采集链路，验证任务配置，最后启动TIM1和TIM6中断时间基。
 *
 * @return 编码器、测试模块、ADC采集、任务配置和系统时间基均初始化
 *         成功时返回true，否则返回false。
 *
 * @post 成功后，ADC采集、周期调度和1 ms时间基均已启动。
 */
bool System_Init(void)
{
    bool isInitialized = false;

    Led_Init();
    SystemTime_Init();
    MotorControl_Init();
    
    if ((Test_Init() == true) &&
        (EncoderService_Init(&System_EncoderConfig) == true) &&
        (EncoderService_Start() == true) &&
        (AdcService_Calibrate(ADC_SERVICE_UNIT_1) == true) &&
        (AdcService_Calibrate(ADC_SERVICE_UNIT_2) == true) &&
        (AdcService_Calibrate(ADC_SERVICE_UNIT_3) == true) &&
        (AdcService_Start() == true) &&
        (TaskManager_Init() == true))
    {
        isInitialized = SystemTime_Start();
    }

    return isInitialized;
}

/**
 * @brief Runs the cooperative task scheduler in the main-loop context.
 *
 * This function is non-blocking and shall not be called from interrupt
 * context.
 *
 * @pre System_Init() has returned true.
 */
void System_MainFunction(void)
{
    TaskManager_Run();
}

#include "Motor_IF.h"
#include "Motor.h"
#include "FocCurrent.h"
#include "FocPID.h"


typedef enum
{
    MOTOR_IF_STATE_IDLE = 0U,
    MOTOR_IF_STATE_PREPOSITION,
    MOTOR_IF_STATE_FORCE_DRAG
} Motor_IFStateIdType;

static Foc_PhCurrentBiasVoltageType Motor_IF_BiasVoltage;





void Motor_IF_Init(void)
{
    PID_init();
}




void Motor_IF_ControlFunction(void)
{
    if(Motor_GetIsRunning() == true)
    {
    }
}



void Motor_IF_MainFunction(void)
{
    if (Motor_GetIsRunning() == false)
    {
       FocCurrent_CalibrateBiasVoltage();
       FocCurrent_GetBiasVoltage(&Motor_IF_BiasVoltage);
       PID_Reset();
    }
    else
    {
        /* V/F快速控制由ADC注入组转换完成中断回调执行。 */
    }
}

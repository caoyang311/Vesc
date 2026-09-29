#include "SystemTime.h"
#include "tim.h"
#include "can.h"
#include "Test.h"
#include <string.h>
#include "AdcService.h"
#include "Dio.h"
#include "MotorControl.h"
#include "FocIfStart.h"
#include "FocVfStart.h"
#include "EncoderService.h"
#include "usart.h"
/**
 * @brief Handles HAL timer period-elapsed notifications.
 *
 * TIM1更新中断刷新编码器位置并执行快速控制入口，TIM6更新中断推进
 * 毫秒系统时间。其他定时器事件被忽略。本回调在中断上下文执行，必须
 * 保持有界且不阻塞。
 *
 * @param[in] htim Pointer to the elapsed timer handle. The HAL callback
 *                 contract guarantees a valid, non-NULL handle.
 *
 * @pre SystemTime_Init() has completed before TIM6 interrupts are enabled.
 * @post A TIM6 notification increments the system time by one millisecond.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        EncoderService_UpdatePosition();

        switch (MotorControl_GetState())
        {
            case MOTOR_STATE_PREPOSITION:
                if (MotorControl_GetMode() == MOTOR_MODE_IF)
                {
                    FocIfStart_Preposition();
                }
                else
                {
                    FocVfStart_Preposition();
                }
                break;

            case MOTOR_STATE_FORCE_DRAG_START:
                if (MotorControl_GetMode() == MOTOR_MODE_IF)
                {
                    FocIfStart_ForceDrag();
                }
                else
                {
                    FocVfStart_ForceDrag();
                }
                break;

            default:
                break;
        }

    }
    else if (htim->Instance == TIM6)
    {
        SystemTime_Notification1ms();
    }
    else
    {
        /* 其他定时器更新事件不属于本服务回调的处理范围。 */
    }
}


/**
 * @brief Handles HAL CAN FIFO0 message pending notifications.
 *
 * This callback is triggered when a new CAN message is available in FIFO0.
 * It processes the message and stores it in the CAN driver's buffer.
 *
 * @param[in] hcan Pointer to the CAN handle. The HAL callback contract
 *                 guarantees a valid, non-NULL handle.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
    CAN_RxHeaderTypeDef rx_header = {0};
    uint8_t rx_data[8] = {0};

    if (hcan == NULL)
    {
        /* 非法 HAL 回调参数不执行 CAN 访问。 */
    }
    else if (HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,rx_data) != HAL_OK)
    {
        /* HAL 读取失败由 CAN 错误回调统一上报。 */
    }
    else if ((rx_header.IDE != CAN_ID_STD) ||
             (rx_header.DLC > 8))
    {
        /* 当前 CAN 驱动仅接收 DLC 合法的标准帧。 */
    }
    else
    {
        message.id = rx_header.StdId;
        message.len = (uint8_t)rx_header.DLC;
        (void)memcpy(message.data, rx_data, message.len);        
    }
}
/**
 * @brief Handles HAL CAN error notifications.
 *
 * This callback is triggered when a CAN error is detected. It processes the error
 * and updates the CAN driver's state.
 *
 * @param[in] hcan Pointer to the CAN handle. The HAL callback contract
 *                 guarantees a valid, non-NULL handle.
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef * hcan)
{
    if (hcan == NULL)
    {
    }
    else
    {
        
    }
}

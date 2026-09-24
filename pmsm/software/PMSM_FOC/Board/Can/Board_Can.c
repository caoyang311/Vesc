#include "Board_Can.h"

#define BOARD_CAN_RX_QUEUE_SIZE (16U)

static Board_Can_RxMessageType Board_Can_RxQueue[BOARD_CAN_RX_QUEUE_SIZE] = {0};
static volatile uint16_t Board_Can_RxWriteIndex = 0U;
static volatile uint16_t Board_Can_RxReadIndex = 0U;
static volatile uint16_t Board_Can_RxCount = 0U;
static volatile uint8_t Board_Can_Initialized = 0U;

Board_Can_ResultType Board_Can_Init(void)
{
    CAN_FilterTypeDef filter = {0};
    HAL_StatusTypeDef halResult = HAL_ERROR;
    Board_Can_ResultType result = BOARD_CAN_RESULT_HAL_ERROR;
    uint16_t index = 0U;

    for (index = 0U; index < BOARD_CAN_RX_QUEUE_SIZE; index++)
    {
        Board_Can_RxQueue[index].Id = 0U;
        Board_Can_RxQueue[index].IdType = 0U;
        Board_Can_RxQueue[index].RxFrameType = 0U;
        Board_Can_RxQueue[index].DataLength = 0U;
        Board_Can_RxQueue[index].FilterMatchIndex = 0U;
        Board_Can_RxQueue[index].Timestamp = 0U;
    }

    Board_Can_RxWriteIndex = 0U;
    Board_Can_RxReadIndex = 0U;
    Board_Can_RxCount = 0U;
    Board_Can_Initialized = 0U;

    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;

    halResult = HAL_CAN_ConfigFilter(&hcan1, &filter);
    if (halResult == HAL_OK)
    {
        halResult = HAL_CAN_Start(&hcan1);
    }
    else
    {
        /* 保持错误状态。 */
    }

    if (halResult == HAL_OK)
    {
        halResult = HAL_CAN_ActivateNotification(
            &hcan1,
            CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF | CAN_IT_ERROR);
    }
    else
    {
        /* 保持错误状态。 */
    }

    if (halResult == HAL_OK)
    {
        Board_Can_Initialized = 1U;
        result = BOARD_CAN_RESULT_OK;
    }
    else
    {
        Board_Can_Initialized = 0U;
    }

    return result;
}

Board_Can_ResultType Board_Can_Send(uint32_t Id,
                                    uint32_t IdType,
                                    uint32_t DataLength,
                                    const uint8_t *Data)
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t txMailbox = 0U;
    HAL_StatusTypeDef halResult = HAL_ERROR;
    Board_Can_ResultType result = BOARD_CAN_RESULT_HAL_ERROR;

    if (Board_Can_Initialized == 0U)
    {
        result = BOARD_CAN_RESULT_NOT_INITIALIZED;
    }
    else if ((Data == (const uint8_t *)0) || (DataLength > BOARD_CAN_DATA_LENGTH))
    {
        result = BOARD_CAN_RESULT_INVALID_PARAMETER;
    }
    else if ((IdType != CAN_ID_STD) && (IdType != CAN_ID_EXT))
    {
        result = BOARD_CAN_RESULT_INVALID_PARAMETER;
    }
    else
    {
        header.StdId = 0U;
        header.ExtId = 0U;
        header.IDE = IdType;
        header.RTR = CAN_RTR_DATA;
        header.DLC = DataLength;
        header.TransmitGlobalTime = DISABLE;

        if (IdType == CAN_ID_STD)
        {
            header.StdId = Id;
        }
        else
        {
            header.ExtId = Id;
        }

        halResult = HAL_CAN_AddTxMessage(&hcan1, &header, (uint8_t *)Data, &txMailbox);
        if (halResult == HAL_OK)
        {
            result = BOARD_CAN_RESULT_OK;
        }
        else if (halResult == HAL_BUSY)
        {
            result = BOARD_CAN_RESULT_BUSY;
        }
        else
        {
            result = BOARD_CAN_RESULT_HAL_ERROR;
        }
    }

    return result;
}

Board_Can_ResultType Board_Can_Receive(Board_Can_RxMessageType *Message)
{
    Board_Can_ResultType result = BOARD_CAN_RESULT_OK;

    if (Board_Can_Initialized == 0U)
    {
        result = BOARD_CAN_RESULT_NOT_INITIALIZED;
    }
    else if (Message == (Board_Can_RxMessageType *)0)
    {
        result = BOARD_CAN_RESULT_INVALID_PARAMETER;
    }
    else if (Board_Can_RxCount == 0U)
    {
        result = BOARD_CAN_RESULT_NO_DATA;
    }
    else
    {
        *Message = Board_Can_RxQueue[Board_Can_RxReadIndex];
        Board_Can_RxReadIndex++;
        if (Board_Can_RxReadIndex >= BOARD_CAN_RX_QUEUE_SIZE)
        {
            Board_Can_RxReadIndex = 0U;
        }
        Board_Can_RxCount--;
    }

    return result;
}

uint16_t Board_Can_GetReceivedCount(void)
{
    return Board_Can_RxCount;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
    CAN_RxHeaderTypeDef header = {0};
    uint8_t data[BOARD_CAN_DATA_LENGTH] = {0U};
    uint16_t nextIndex = 0U;

    if ((CanHandle != (CAN_HandleTypeDef *)0) && (CanHandle->Instance == CAN1))
    {
        if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &header, data) == HAL_OK)
        {
            nextIndex = Board_Can_RxWriteIndex + 1U;
            if (nextIndex >= BOARD_CAN_RX_QUEUE_SIZE)
            {
                nextIndex = 0U;
            }

            if (Board_Can_RxCount < BOARD_CAN_RX_QUEUE_SIZE)
            {
                if (header.IDE == CAN_ID_STD)
                {
                    Board_Can_RxQueue[Board_Can_RxWriteIndex].Id = header.StdId;
                }
                else
                {
                    Board_Can_RxQueue[Board_Can_RxWriteIndex].Id = header.ExtId;
                }
                Board_Can_RxQueue[Board_Can_RxWriteIndex].IdType = header.IDE;
                Board_Can_RxQueue[Board_Can_RxWriteIndex].RxFrameType = header.RTR;
                Board_Can_RxQueue[Board_Can_RxWriteIndex].DataLength = header.DLC;
                Board_Can_RxQueue[Board_Can_RxWriteIndex].FilterMatchIndex = header.FilterMatchIndex;
                Board_Can_RxQueue[Board_Can_RxWriteIndex].Timestamp = header.Timestamp;

                for (uint16_t index = 0U; index < BOARD_CAN_DATA_LENGTH; index++)
                {
                    Board_Can_RxQueue[Board_Can_RxWriteIndex].Data[index] = data[index];
                }

                Board_Can_RxWriteIndex = nextIndex;
                Board_Can_RxCount++;
            }
            else
            {
                /* 接收队列满时丢弃新报文，保留已有报文。 */
            }
        }
        else
        {
            /* HAL 读取失败时不写入接收队列。 */
        }
    }
    else
    {
        /* 非 CAN1 回调不在本模块处理。 */
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *CanHandle)
{
    if ((CanHandle != (CAN_HandleTypeDef *)0) && (CanHandle->Instance == CAN1))
    {
        /* CAN 错误状态由 HAL 保存在 hcan1.ErrorCode 中。 */
    }
    else
    {
        /* 非 CAN1 错误不在本模块处理。 */
    }
}

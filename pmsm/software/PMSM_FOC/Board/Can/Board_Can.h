#ifndef BOARD_CAN_H
#define BOARD_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "can.h"

#define BOARD_CAN_DATA_LENGTH (8U)

/**
 * @brief CAN1 接收报文。
 */
typedef struct
{
    uint32_t Id;
    uint32_t IdType;
    uint32_t RxFrameType;
    uint32_t DataLength;
    uint8_t Data[BOARD_CAN_DATA_LENGTH];
    uint32_t FilterMatchIndex;
    uint32_t Timestamp;
} Board_Can_RxMessageType;

/**
 * @brief CAN1 模块操作结果。
 */
typedef enum
{
    BOARD_CAN_RESULT_OK = 0U,
    BOARD_CAN_RESULT_NOT_INITIALIZED,
    BOARD_CAN_RESULT_INVALID_PARAMETER,
    BOARD_CAN_RESULT_BUSY,
    BOARD_CAN_RESULT_NO_DATA,
    BOARD_CAN_RESULT_HAL_ERROR
} Board_Can_ResultType;

/**
 * @brief 初始化并启动 CAN1，配置 FIFO0 接收中断。
 *
 * @note CubeMX 生成的 MX_CAN1_Init() 必须在本接口前调用。
 */
Board_Can_ResultType Board_Can_Init(void);

/**
 * @brief 发送一帧 CAN1 数据。
 *
 * @param[in] Id 标准或扩展帧 ID。
 * @param[in] IdType CAN_ID_STD 或 CAN_ID_EXT。
 * @param[in] DataLength 数据长度，范围 0..8 字节。
 * @param[in] Data 数据缓冲区。
 * @return Board_Can_ResultType 操作结果。
 */
Board_Can_ResultType Board_Can_Send(uint32_t Id,
                                    uint32_t IdType,
                                    uint32_t DataLength,
                                    const uint8_t *Data);

/**
 * @brief 从 CAN1 接收队列读取一帧报文。
 *
 * @param[out] Message 接收报文输出对象。
 * @return Board_Can_ResultType 操作结果。
 */
Board_Can_ResultType Board_Can_Receive(Board_Can_RxMessageType *Message);

/**
 * @brief 获取 CAN1 接收队列中的报文数量。
 */
uint16_t Board_Can_GetReceivedCount(void);

/**
 * @brief CAN1 FIFO0 接收中断回调。
 *
 * @param[in] CanHandle CAN 外设句柄。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_CAN_H */

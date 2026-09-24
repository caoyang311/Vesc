/**
 * @file VescPacket.c
 * @brief VESC Packet 串口协议实现。
 */
#include "VescPacket.h"

#include "Uart1.h"
#include "Board_Adc.h"

#define VESC_PACKET_START_SHORT          (2U)   /** @brief 短帧起始字节 */
#define VESC_PACKET_START_LONG           (3U)   /** @brief 长帧起始字节 */
#define VESC_PACKET_END                  (3U)   /** @brief 帧结束字节 */
#define VESC_PACKET_MAX_PAYLOAD_LENGTH   (512U) /** @brief 最大有效载荷长度 */
#define VESC_PACKET_MAX_FRAME_LENGTH     (VESC_PACKET_MAX_PAYLOAD_LENGTH + 6U) /** @brief 最大帧长度 */
#define VESC_PACKET_COMM_FW_VERSION      (0U)     /** @brief 固件版本命令 */
#define VESC_PACKET_COMM_GET_VALUES      (4U)     /** @brief 获取值命令 */
#define VESC_PACKET_COMM_SET_DUTY        (5U)     /** @brief 设置占空比命令 */
#define VESC_PACKET_COMM_SET_CURRENT     (6U)     /** @brief 设置电流命令 */
#define VESC_PACKET_COMM_SET_RPM         (8U)     /** @brief 设置RPM命令 */
#define VESC_PACKET_COMM_GET_MCCONF      (14U)    /** @brief 获取主配置命令 */
#define VESC_PACKET_COMM_SET_MCCONF      (13U)    /** @brief 设置主配置命令 */
#define VESC_PACKET_COMM_GET_APPCONF     (17U)    /** @brief 获取应用配置命令 */
#define VESC_PACKET_COMM_SET_APPCONF     (16U)    /** @brief 设置应用配置命令 */
#define VESC_PACKET_COMM_SET_CURRENT_BRAKE (7U)   /** @brief 设置电流刹车命令 */
#define VESC_PACKET_FW_MAJOR             (0U)     /** @brief 主版本号 */
#define VESC_PACKET_FW_MINOR             (1U)     /** @brief 次版本号 */
#define VESC_PACKET_HW_NAME              "PMSM_FOC" /** @brief 硬件名称 */

static uint8_t VescPacket_Frame[VESC_PACKET_MAX_FRAME_LENGTH] = {0U};/* @brief 帧缓冲区 */
static uint16_t VescPacket_FrameLength = 0U;/* @brief 当前帧长度 */
static uint16_t VescPacket_ExpectedLength = 0U;/* @brief 期望帧长度 */
static uint8_t VescPacket_LengthBytes = 0U;/* @brief 长度字节 */
static uint8_t VescPacket_Initialized = 0U;/* @brief 是否初始化 */
static int32_t VescPacket_Duty = 0;/* @brief 占空比 */
static int32_t VescPacket_Current = 0;/* @brief 电流 */
static int32_t VescPacket_Rpm = 0;/* @brief RPM */

static void VescPacket_ResetFrame(void);
static VescPacket_ResultType VescPacket_ProcessFrame(const uint8_t *Payload, uint16_t Length);
static VescPacket_ResultType VescPacket_SendPayload(const uint8_t *Payload, uint16_t Length);
static void VescPacket_AppendU16(uint8_t *Buffer, uint16_t Value, uint16_t *Index);
static void VescPacket_AppendU32(uint8_t *Buffer, uint32_t Value, uint16_t *Index);
static void VescPacket_AppendS32(uint8_t *Buffer, int32_t Value, uint16_t *Index);
static uint32_t VescPacket_ReadU32(const uint8_t *Buffer);
static void VescPacket_ProcessCommand(const uint8_t *Payload, uint16_t Length, uint8_t *Response, uint16_t *ResponseLength);
static void VescPacket_AppendAdcValue(uint8_t *Response, uint16_t *Index, Adc_ChannelType Channel);
/**
 * @brief 初始化 VESC Packet 协议
 *
 * @return void
 */
void VescPacket_Init(void)
{
    VescPacket_ResetFrame();
    VescPacket_Initialized = 1U;
}
/**
 * @brief 处理 VESC Packet 协议任务
 *
 * @return VescPacket_ResultType
 */
VescPacket_ResultType VescPacket_Task(void)
{
    Uart1_MessageType message = {0};
    VescPacket_ResultType result = VESC_PACKET_RESULT_NO_DATA;
    uint16_t index = 0U;

    if (VescPacket_Initialized == 0U)
    {
        result = VESC_PACKET_RESULT_NOT_INITIALIZED;
    }
    else
    {
        while (Uart1_ReceiveMessage(&message) == UART1_RESULT_OK)
        {
            for (index = 0U; index < message.Length; index++)
            {
                result = packet_process_byte(message.Data[index]);
            }
        }
    }

    return result;
}
/**
 * @brief 处理 VESC Packet 协议字节
 * @brief 帧组成：
 * 1. 起始字节：0x02（短帧起始字节） 或 0x03（长帧起始字节）
 * 2. 数据长度字节：1字节或2字节
 * 3. 数据字节：有效载荷数据
 * 4. CRC16 字节：校验和
 * 5. 结束字节：0x03
 * @param Byte 字节
 * @return VescPacket_ResultType
 */
VescPacket_ResultType packet_process_byte(uint8_t Byte)
{
    VescPacket_ResultType result = VESC_PACKET_RESULT_OK;/* @brief 结果 */
    uint16_t payload_length = 0U;/* @brief 有效载荷长度 */
    uint16_t frame_length = 0U;/* @brief 帧长度 */
    uint16_t crc_received = 0U;/* @brief 接收 CRC */
    uint16_t crc_calculated = 0U;/* @brief 计算 CRC */

    if (VescPacket_Initialized == 0U)
    {
        result = VESC_PACKET_RESULT_NOT_INITIALIZED;
    }
    else if (VescPacket_FrameLength >= VESC_PACKET_MAX_FRAME_LENGTH)/* @brief 帧缓冲区已满 */
    {
        VescPacket_ResetFrame();/* @brief 重置当前帧 */
        result = VESC_PACKET_RESULT_BUFFER_ERROR;/* @brief 缓冲区错误 */
    }
    else
    {
        VescPacket_Frame[VescPacket_FrameLength] = Byte;/* @brief 存储字节 */
        VescPacket_FrameLength++;/* @brief 增加帧长度 */

        if (VescPacket_FrameLength == 1U)/* @brief 第一个字节:区分短帧和长帧 */
        {
            if (Byte == VESC_PACKET_START_SHORT)/* @brief 短帧 */
            {
                VescPacket_LengthBytes = 1U;/* @brief 短帧数据：数据长度占1字节 */
            }
            else if (Byte == VESC_PACKET_START_LONG)/* @brief 长帧 */
            {
                VescPacket_LengthBytes = 2U;/* @brief 长帧数据：数据长度占2字节 */
            }
            else
            {
                VescPacket_ResetFrame();/* @brief 重置当前帧 */
                result = VESC_PACKET_RESULT_FRAME_ERROR;/* @brief 帧错误 */
            }
        }
        else if ((VescPacket_LengthBytes == 1U) && (VescPacket_FrameLength == 2U))/* @brief 短帧数据：第二字节代表有效载荷长度 */
        {
            payload_length = VescPacket_Frame[1];/* @brief 有效载荷长度 */
            if ((payload_length == 0U) || (payload_length > VESC_PACKET_MAX_PAYLOAD_LENGTH))/* @brief 有效载荷长度无效 */
            {
                VescPacket_ResetFrame();/* @brief 重置当前帧 */
                result = VESC_PACKET_RESULT_FRAME_ERROR;/* @brief 帧错误 */
            }
            else
            {
                VescPacket_ExpectedLength = payload_length + 5U;
            }
        }
        else if ((VescPacket_LengthBytes == 2U) && (VescPacket_FrameLength == 3U))/* @brief 长帧数据：第二、三字节代表有效载荷长度 */
        {
            payload_length = ((uint16_t)VescPacket_Frame[1] << 8U) | VescPacket_Frame[2];
            if ((payload_length < 255U) || (payload_length > VESC_PACKET_MAX_PAYLOAD_LENGTH))/* @brief 有效载荷长度无效 */
            {
                VescPacket_ResetFrame();/* @brief 重置当前帧 */
                result = VESC_PACKET_RESULT_FRAME_ERROR;/* @brief 帧错误 */
            }
            else
            {
                VescPacket_ExpectedLength = payload_length + 6U;
            }
        }
        else if ((VescPacket_ExpectedLength > 0U) && (VescPacket_FrameLength >= VescPacket_ExpectedLength))/* @brief 接收到完整帧 */
        {
            frame_length = VescPacket_ExpectedLength;/* @brief 帧长度 */
            payload_length = (VescPacket_LengthBytes == 1U) ? VescPacket_Frame[1] : (((uint16_t)VescPacket_Frame[1] << 8U) | VescPacket_Frame[2]);/* @brief 有效载荷长度 */
            crc_received = ((uint16_t)VescPacket_Frame[frame_length - 3U] << 8U) | VescPacket_Frame[frame_length - 2U];/* @brief 接收 CRC */
            crc_calculated = VescPacket_Crc16(&VescPacket_Frame[1U + VescPacket_LengthBytes], payload_length);/* @brief 计算 CRC */

            if ((VescPacket_Frame[frame_length - 1U] != VESC_PACKET_END) || (crc_received != crc_calculated))/* @brief 帧错误 */
            {
                result = VESC_PACKET_RESULT_FRAME_ERROR;/* @brief 帧错误 */
            }
            else
            {
                result = VescPacket_ProcessFrame(&VescPacket_Frame[1U + VescPacket_LengthBytes], payload_length);/* @brief 处理帧 */
            }
            VescPacket_ResetFrame();/* @brief 重置当前帧 */
        }
        else
        {
            /* 等待完整帧。 */
        }
    }

    return result;
}
/**
 * @brief 计算 CRC16 校验和
 *
 * @param Data 数据指针
 * @param Length 数据长度
 * @return uint16_t CRC16 校验和
 */
uint16_t VescPacket_Crc16(const uint8_t *Data, uint16_t Length)
{
    uint16_t crc = 0U;
    uint16_t index = 0U;
    uint8_t bit = 0U;

    if (Data != (const uint8_t *)0)
    {
        for (index = 0U; index < Length; index++)
        {
            crc ^= (uint16_t)Data[index] << 8U;
            for (bit = 0U; bit < 8U; bit++)
            {
                if ((crc & 0x8000U) != 0U)
                {
                    crc = (uint16_t)((crc << 1U) ^ 0x1021U);
                }
                else
                {
                    crc = (uint16_t)(crc << 1U);
                }
            }
        }
    }

    return crc;
}

/**
 * @brief 重置当前帧
 *
 * @return void
 */
static void VescPacket_ResetFrame(void)
{
    VescPacket_FrameLength = 0U;
    VescPacket_ExpectedLength = 0U;
    VescPacket_LengthBytes = 0U;
}
/**
 * @brief 处理 VESC Packet 协议帧
 *
 * @param Payload 有效载荷指针
 * @param Length 有效载荷长度
 * @return VescPacket_ResultType
 */
static VescPacket_ResultType VescPacket_ProcessFrame(const uint8_t *Payload, uint16_t Length)
{
    uint8_t response[96U] = {0U};
    uint16_t response_length = 0U;
    VescPacket_ResultType result = VESC_PACKET_RESULT_OK;

    if ((Payload == (const uint8_t *)0) || (Length == 0U))
    {
        result = VESC_PACKET_RESULT_INVALID_PARAMETER;
    }
    else
    {
        VescPacket_ProcessCommand(Payload, Length, response, &response_length);
        if (response_length > 0U)
        {
            result = VescPacket_SendPayload(response, response_length);
        }
    }

    return result;
}

static VescPacket_ResultType VescPacket_SendPayload(const uint8_t *Payload, uint16_t Length)
{
    uint8_t frame[VESC_PACKET_MAX_FRAME_LENGTH] = {0U};
    uint16_t index = 0U;
    uint16_t crc = 0U;
    VescPacket_ResultType result = VESC_PACKET_RESULT_OK;

    if ((Payload == (const uint8_t *)0) || (Length == 0U) || (Length > VESC_PACKET_MAX_PAYLOAD_LENGTH))
    {
        result = VESC_PACKET_RESULT_INVALID_PARAMETER;
    }
    else
    {
        if (Length <= 255U)
        {
            frame[index++] = VESC_PACKET_START_SHORT;
            frame[index++] = (uint8_t)Length;
        }
        else
        {
            frame[index++] = VESC_PACKET_START_LONG;
            frame[index++] = (uint8_t)(Length >> 8U);
            frame[index++] = (uint8_t)Length;
        }

        for (uint16_t payload_index = 0U; payload_index < Length; payload_index++)
        {
            frame[index++] = Payload[payload_index];
        }
        crc = VescPacket_Crc16(Payload, Length);
        frame[index++] = (uint8_t)(crc >> 8U);
        frame[index++] = (uint8_t)crc;
        frame[index++] = VESC_PACKET_END;
        if (Uart1_Send(frame, index) != UART1_RESULT_OK)
        {
            result = VESC_PACKET_RESULT_UART_ERROR;
        }
    }

    return result;
}

static void VescPacket_ProcessCommand(const uint8_t *Payload, uint16_t Length, uint8_t *Response, uint16_t *ResponseLength)
{
    uint8_t command = Payload[0U];
    uint16_t index = 0U;
    uint32_t value = 0U;

    Response[index++] = command;

    switch (command)
    {
        case VESC_PACKET_COMM_FW_VERSION:/* @brief 获取固件版本 */
            Response[index++] = VESC_PACKET_FW_MAJOR;
            Response[index++] = VESC_PACKET_FW_MINOR;
            Response[index++] = (uint8_t)'P';
            Response[index++] = (uint8_t)'M';
            Response[index++] = (uint8_t)'S';
            Response[index++] = (uint8_t)'M';
            Response[index++] = (uint8_t)'_';
            Response[index++] = (uint8_t)'F';
            Response[index++] = (uint8_t)'O';
            Response[index++] = (uint8_t)'C';
            Response[index++] = 0U;
            break;

        case VESC_PACKET_COMM_GET_VALUES:
            VescPacket_AppendAdcValue(Response, &index, ADC_CH_ID_POWER_STAGE_TEMP);
            VescPacket_AppendAdcValue(Response, &index, ADC_CH_ID_POWER_STAGE_TEMP);
            VescPacket_AppendS32(Response, 0, &index);
            VescPacket_AppendS32(Response, 0, &index);
            VescPacket_AppendS32(Response, 0, &index);
            VescPacket_AppendS32(Response, 0, &index);
            VescPacket_AppendS32(Response, VescPacket_Duty, &index);
            VescPacket_AppendS32(Response, VescPacket_Rpm, &index);
            VescPacket_AppendAdcValue(Response, &index, ADC_CH_ID_DC_BUS_VOLTAGE);
            VescPacket_AppendS32(Response, 0, &index);
            VescPacket_AppendS32(Response, 0, &index);
            VescPacket_AppendS32(Response, 0, &index);
            VescPacket_AppendS32(Response, 0, &index);
            VescPacket_AppendS32(Response, 0, &index);
            VescPacket_AppendS32(Response, 0, &index);
            Response[index++] = 0U;
            VescPacket_AppendS32(Response, 0, &index);
            Response[index++] = 0U;
            Response[index++] = 0U;
            Response[index++] = 0U;
            Response[index++] = 0U;
            Response[index++] = 0U;
            break;

        case VESC_PACKET_COMM_SET_DUTY:/* @brief 设置占空比 */
            if (Length >= 5U)
            {
                value = VescPacket_ReadU32(&Payload[1U]);
                VescPacket_Duty = (int32_t)value;
            }
            else
            {
                Response[index++] = 0U;
            }
            index = 0U;
            break;

        case VESC_PACKET_COMM_SET_CURRENT:/* @brief 设置电流 */
            if (Length >= 5U)
            {
                value = VescPacket_ReadU32(&Payload[1U]);
                VescPacket_Current = (int32_t)value;
            }
            else
            {
                Response[index++] = 0U;
            }
            index = 0U;
            break;

        case VESC_PACKET_COMM_SET_CURRENT_BRAKE:/* @brief 设置电流刹车 */
            if (Length >= 5U)
            {
                value = VescPacket_ReadU32(&Payload[1U]);
                VescPacket_Current = -(int32_t)value;
            }
            else
            {
                Response[index++] = 0U;
            }
            index = 0U;
            break;

        case VESC_PACKET_COMM_SET_RPM:/* @brief 设置转速 */
            if (Length >= 5U)
            {
                value = VescPacket_ReadU32(&Payload[1U]);
                VescPacket_Rpm = (int32_t)value;
            }
            else
            {
                Response[index++] = 0U;
            }
            index = 0U;
            break;

        case VESC_PACKET_COMM_GET_MCCONF:
        case VESC_PACKET_COMM_SET_MCCONF:
        case VESC_PACKET_COMM_GET_APPCONF:
        case VESC_PACKET_COMM_SET_APPCONF:
            index = 0U;
            break;

        default:
            index = 0U;
            break;
    }

    *ResponseLength = index;
}

/**
 * @brief 追加 ADC 值到响应缓冲区
 *
 * @param Response 响应缓冲区指针
 * @param Index 当前索引指针
 * @param Channel ADC 通道类型
 */
static void VescPacket_AppendAdcValue(uint8_t *Response, uint16_t *Index, Adc_ChannelType Channel)
{
    Adc_ValueType value = 0U;
    if (Adc_ReadChannel(Channel, &value) == ADC_RESULT_OK)
    {
        VescPacket_AppendU16(Response, value, Index);
    }
    else
    {
        VescPacket_AppendU16(Response, 0U, Index);
    }
}
/**
 * @brief 追加 16 位无符号整数到缓冲区
 *
 * @param Buffer 缓冲区指针
 * @param Value 无符号整数值
 * @param Index 当前索引指针
 */
static void VescPacket_AppendU16(uint8_t *Buffer, uint16_t Value, uint16_t *Index)
{
    Buffer[*Index] = (uint8_t)(Value >> 8U);
    (*Index)++;
    Buffer[*Index] = (uint8_t)Value;
    (*Index)++;
}
/**
 * @brief 追加 32 位无符号整数到缓冲区
 *
 * @param Buffer 缓冲区指针
 * @param Value 无符号整数值
 * @param Index 当前索引指针
 */
static void VescPacket_AppendU32(uint8_t *Buffer, uint32_t Value, uint16_t *Index)
{
    Buffer[*Index] = (uint8_t)(Value >> 24U);
    (*Index)++;
    Buffer[*Index] = (uint8_t)(Value >> 16U);
    (*Index)++;
    Buffer[*Index] = (uint8_t)(Value >> 8U);
    (*Index)++;
    Buffer[*Index] = (uint8_t)Value;
    (*Index)++;
}
/**
 * @brief 追加 32 位有符号整数到缓冲区
 *
 * @param Buffer 缓冲区指针
 * @param Value 有符号整数值
 * @param Index 当前索引指针
 */
static void VescPacket_AppendS32(uint8_t *Buffer, int32_t Value, uint16_t *Index)
{
    VescPacket_AppendU32(Buffer, (uint32_t)Value, Index);
}
/**
 * @brief 从缓冲区读取 32 位无符号整数
 *
 * @param Buffer 缓冲区指针
 * @return 无符号整数值
 */
static uint32_t VescPacket_ReadU32(const uint8_t *Buffer)
{
    return ((uint32_t)Buffer[0U] << 24U) |
           ((uint32_t)Buffer[1U] << 16U) |
           ((uint32_t)Buffer[2U] << 8U) |
           (uint32_t)Buffer[3U];
}

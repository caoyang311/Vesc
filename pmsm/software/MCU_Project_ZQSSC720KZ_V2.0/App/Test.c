#include "Test.h"
#include "Dio.h"
#include "Pwm.h"
#include "can.h"
#include <stddef.h>
#include <string.h>
#include "AdcService.h"
#include "MotorControl.h"
#include "usart.h"

#define TEST_PWM_U_INITIAL_COMPARE       (UINT16_C(900U))
#define TEST_PWM_V_INITIAL_COMPARE       (UINT16_C(1800U))
#define TEST_PWM_W_INITIAL_COMPARE       (UINT16_C(2700U))
#define TEST_ADC_TRIGGER_INITIAL_COMPARE (UINT16_C(3600-5))
#define TEST_JUST_FLOAT_CHANNEL_COUNT     (3U)
#define TEST_JUST_FLOAT_VALUE_SIZE        (4U)
#define TEST_JUST_FLOAT_TAIL_SIZE         (4U)
#define TEST_JUST_FLOAT_FRAME_SIZE        (16U)
#define TEST_UART_TIMEOUT_MS              (10U)

Dio_LevelType DSP_P_Level = STD_LOW;
Dio_LevelType DSP_SS_Level = STD_LOW;
Dio_LevelType DSP_SDL_Level = STD_LOW;
Dio_LevelType DSP_SDH_Level = STD_LOW;
Dio_LevelType DSP_FDS_Level = STD_LOW;
Dio_LevelType DSP_XH_Level = STD_LOW;
Dio_LevelType DSP_PUSH_Level = STD_LOW;

can_msg_t message = {0};

static uint16_t Test_PwmCompare[TEST_PWM_CHANNEL_COUNT];
static uint16_t Test_AdcTriggerCompare;

static AdcService_ValueType TEMP_MOTOR_IN = 0;/* 电机温度输入 */
static AdcService_ValueType TEMP_MOS_U = 0;/* MOS温度输入U */
static AdcService_ValueType TEMP_MOS_V = 0;/* MOS温度输入V */
static AdcService_ValueType TEMP_MOS_W = 0;/* MOS温度输入W */

static AdcService_ValueType ZB_ADC_IN = 0;/* 转把电压输入 */
/**
 * @brief 发送原始CAN消息。
 *
 * @param[in] can_id 标准CAN消息标识符，允许范围为0至0x7FF。
 * @param[in] data 指向待发送数据的非NULL指针，缓冲区至少包含dlc字节。
 * @param[in] dlc CAN数据长度，允许范围为1至8字节。
 *
 * @return 发送请求被HAL接受时返回0，否则返回1。
 */
static uint8_t CAN_SendRaw(uint32_t can_id, const uint8_t *data, uint8_t dlc);
static void verify_fpu(void);
/**
 * @brief 初始化测试模块、六路互补PWM输出和ADC触发。
 *
 * 为U、V、W三组互补PWM和TIM1 CH4设置独立测试预装载值，随后启动
 * 六路互补输出和CAN测试接口。ADC触发由AdcService统一启动。
 *
 * @return CAN和PWM输出全部启动成功时返回true，否则返回false。
 *
 * @pre MX_CAN_Init()和MX_TIM1_Init()已经执行完成。
 */
bool Test_Init(void)
{
    bool is_initialized = true;

    Test_PwmCompare[TEST_PWM_CHANNEL_U] = TEST_PWM_U_INITIAL_COMPARE;
    Test_PwmCompare[TEST_PWM_CHANNEL_V] = TEST_PWM_V_INITIAL_COMPARE;
    Test_PwmCompare[TEST_PWM_CHANNEL_W] = TEST_PWM_W_INITIAL_COMPARE;
    Test_AdcTriggerCompare = TEST_ADC_TRIGGER_INITIAL_COMPARE;

    Pwm_SetCompare(PWM_CHANNEL_U,Test_PwmCompare[TEST_PWM_CHANNEL_U]);
    Pwm_SetCompare(PWM_CHANNEL_V,Test_PwmCompare[TEST_PWM_CHANNEL_V]);
    Pwm_SetCompare(PWM_CHANNEL_W,Test_PwmCompare[TEST_PWM_CHANNEL_W]);


    //Pwm_SetAdcTriggerCompare(Test_AdcTriggerCompare);
    //Pwm_StartOutputs();
    //Pwm_StartAdcTrigger();

      Pwm_StopOutputs();
      Pwm_StopAdcTrigger();


    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
        is_initialized = false;
    }
    else if (HAL_CAN_ActivateNotification(
                 &hcan,
                 CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR) != HAL_OK)
    {
        is_initialized = false;
    }
    else
    {
        /* CAN测试接口启动成功。 */
    }
    verify_fpu();
    return is_initialized;
}

/**
 * @brief 执行10 ms测试任务。
 *
 * 读取DIO测试输入，刷新PWM和ADC触发测试预装载值，并处理CAN回环
 * 消息。该函数执行时间有界且不阻塞。
 *
 * @pre Test_Init()已经成功完成。
 */
void Test_Task(void)
{
    can_msg_t tx_msg = {0};
    /* 读取DIO测试输入。 */
    DSP_P_Level = Dio_ReadChannel(DIO_CHANNEL_DSP_P);
    DSP_SS_Level = Dio_ReadChannel(DIO_CHANNEL_DSP_SS);
    DSP_SDL_Level = Dio_ReadChannel(DIO_CHANNEL_DSP_SDL);
    DSP_SDH_Level = Dio_ReadChannel(DIO_CHANNEL_DSP_SDH);
    DSP_FDS_Level = Dio_ReadChannel(DIO_CHANNEL_DSP_FDS);
    DSP_XH_Level = Dio_ReadChannel(DIO_CHANNEL_DSP_XH);
    DSP_PUSH_Level = Dio_ReadChannel(DIO_CHANNEL_DSP_PUSH);

    /* 读取ADC转换结果。 */
    AdcService_ReadChannel(ADC_SERVICE_UNIT_1, ADC_SERVICE_CHANNEL_ADC1_IN6, &TEMP_MOTOR_IN);
    AdcService_ReadChannel(ADC_SERVICE_UNIT_1, ADC_SERVICE_CHANNEL_ADC1_IN7, &TEMP_MOS_U);
    AdcService_ReadChannel(ADC_SERVICE_UNIT_1, ADC_SERVICE_CHANNEL_ADC1_IN9, &TEMP_MOS_V);
    AdcService_ReadChannel(ADC_SERVICE_UNIT_1, ADC_SERVICE_CHANNEL_ADC1_IN14, &TEMP_MOS_W);

    AdcService_ReadChannel(ADC_SERVICE_UNIT_2, ADC_SERVICE_CHANNEL_ADC2_IN8, &ZB_ADC_IN);

    /* 发送CAN消息测试。 */
    if (message.len > 0U)
    {
        switch (message.id)
        {
            case UINT32_C(0x100):
                tx_msg.id = UINT32_C(0x200);
                tx_msg.len = message.len;
                (void)memcpy(tx_msg.data, message.data, tx_msg.len);
                (void)CAN_SendRaw(tx_msg.id, tx_msg.data, tx_msg.len);
                message.len = 0U;
                break;

            default:
                message.len = 0U;
                break;
        }
    }
}

/**
 * @brief 设置指定互补PWM通道组的比较预装载值。
 *
 * @param[in] channel 测试PWM通道。
 * @param[in] preload 比较预装载值，允许范围为0至当前TIM1自动重装载值。
 *
 * @return 参数有效且预装载值完成写入时返回true，否则返回false。
 *
 * @pre Test_Init()已经成功完成。
 */
void Test_SetPwmCompare(Test_PwmChannelType channel, uint16_t preload)
{

    if ((uint32_t)channel < (uint32_t)TEST_PWM_CHANNEL_COUNT)
    {
        Pwm_SetCompare((Pwm_ChannelType)channel, preload);
        Test_PwmCompare[(uint32_t)channel] = preload;
    }
}

/**
 * @brief 设置TIM1 CH4的ADC触发比较预装载值。
 *
 * @param[in] preload ADC触发比较预装载值，允许范围为0至当前TIM1
 *                    自动重装载值。
 *
 * @return 预装载值有效且完成写入时返回true，否则返回false。
 *
 * @pre Test_Init()已经成功完成。
 */
void Test_SetAdcTriggerCompare(uint16_t preload)
{
    Pwm_SetAdcTriggerCompare(preload);
    Test_AdcTriggerCompare = preload;  
}

/**
 * @brief 使用VOFA+ JustFloat协议发送三路浮点数据。
 *
 * 三个IEEE 754单精度浮点数按小端字节序连续排列，随后追加固定帧尾
 * 0x00、0x00、0x80、0x7F。完整帧长度为16字节。
 *
 * @param[in] channel_1 第一路浮点数据。
 * @param[in] channel_2 第二路浮点数据。
 * @param[in] channel_3 第三路浮点数据。
 *
 * @return USART1发送成功时返回true，否则返回false。
 */
bool Test_SendJustFloat(float channel_1,
                        float channel_2,
                        float channel_3)
{
    const float channels[TEST_JUST_FLOAT_CHANNEL_COUNT] =
    {
        channel_1,
        channel_2,
        channel_3
    };
    const uint8_t frame_tail[TEST_JUST_FLOAT_TAIL_SIZE] =
    {
        UINT8_C(0x00),
        UINT8_C(0x00),
        UINT8_C(0x80),
        UINT8_C(0x7F)
    };
    uint8_t frame[TEST_JUST_FLOAT_FRAME_SIZE] = {0};
    uint32_t channel_index = 0U;
    uint32_t frame_offset = 0U;
    bool is_sent = false;

    for (channel_index = 0U;
         channel_index < TEST_JUST_FLOAT_CHANNEL_COUNT;
         channel_index++)
    {
        frame_offset = channel_index * TEST_JUST_FLOAT_VALUE_SIZE;
        (void)memcpy(&frame[frame_offset],
                     &channels[channel_index],
                     TEST_JUST_FLOAT_VALUE_SIZE);
    }

    (void)memcpy(&frame[TEST_JUST_FLOAT_CHANNEL_COUNT *
                        TEST_JUST_FLOAT_VALUE_SIZE],
                 frame_tail,
                 TEST_JUST_FLOAT_TAIL_SIZE);

    if (HAL_UART_Transmit(&huart1,
                          frame,
                          (uint16_t)TEST_JUST_FLOAT_FRAME_SIZE,
                          TEST_UART_TIMEOUT_MS) == HAL_OK)
    {
        is_sent = true;
    }
    else
    {
        /* 串口忙、超时或发生错误时返回false。 */
    }

    return is_sent;
}

/**
 * @brief 发送原始CAN消息。
 *
 * @param[in] can_id 标准CAN消息标识符，允许范围为0至0x7FF。
 * @param[in] data 指向待发送数据的非NULL指针，缓冲区至少包含dlc字节。
 * @param[in] dlc CAN数据长度，允许范围为1至8字节。
 *
 * @return 发送请求被HAL接受时返回0，否则返回1。
 */
static uint8_t CAN_SendRaw(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    CAN_TxHeaderTypeDef tx_hdr = {0};
    uint32_t mailbox = 0U;
    uint8_t result = 1U;

    if ((data != NULL) &&
        (dlc > 0U) &&
        (dlc <= 8U) &&
        (can_id <= UINT32_C(0x7FF)) &&
        (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0U))
    {
        tx_hdr.StdId = can_id;
        tx_hdr.IDE = CAN_ID_STD;
        tx_hdr.RTR = CAN_RTR_DATA;
        tx_hdr.DLC = dlc;
        tx_hdr.TransmitGlobalTime = DISABLE;

        if (HAL_CAN_AddTxMessage(&hcan,
                                 &tx_hdr,
                                 (uint8_t *)data,
                                 &mailbox) == HAL_OK)
        {
            result = 0U;
        }
    }

    return result;
}
/**
 * @brief 验证FPU是否已启用。
 *
 * 该函数检查SCB->CPACR寄存器的FPU访问位，并执行一个简单的浮点运算
 * 来验证FPU是否正常工作。若FPU未启用，可能会触发HardFault异常。
 *
 * @note 调用方应确保在中断上下文中调用此函数时不会触发异常。
 */
static void verify_fpu(void) {
    // 1. 检查硬件使能寄存器
    uint32_t cpacr = SCB->CPACR;
    if ((cpacr & ((3UL << 20) | (3UL << 22))) == ((3UL << 20) | (3UL << 22))) {

        //printf("[OK] CPACR: FPU access enabled\r\n");
        Dio_WriteChannel(DIO_CHANNEL_LED,STD_HIGH);
    } else {
       // printf("[ERROR] CPACR: FPU access NOT enabled\r\n");
        Dio_WriteChannel(DIO_CHANNEL_LED,STD_LOW);
    }
    
    // 2. 执行一个简单的浮点运算，验证是否能正确执行（如果 FPU 未使能会触发 HardFault）
    volatile float a = 3.14159f;
    volatile float b = 2.71828f;
    volatile float c = a * b + 0.5f;
    
    if (c > 0.0f) {

       // printf("[OK] Float operation executed successfully, result = %f\r\n", (double)c);
       Dio_WriteChannel(DIO_CHANNEL_LED,STD_HIGH);
    }
    
    // 3. 检查 FPU 异常标志（可选项）
    uint32_t fpscr = __get_FPSCR();
   // printf("FPSCR = 0x%08X\r\n", fpscr);
   // Dio_WriteChannel(DIO_CHANNEL_LED,STD_LOW);
}
#ifndef TEST_H
#define TEST_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
} can_msg_t;

typedef enum
{
    TEST_PWM_CHANNEL_U = 0,
    TEST_PWM_CHANNEL_V,
    TEST_PWM_CHANNEL_W,
    TEST_PWM_CHANNEL_COUNT
} Test_PwmChannelType;

extern can_msg_t message;

/**
 * @brief 初始化测试模块和六路互补PWM输出。
 *
 * 设置三相PWM及TIM1 CH4的测试预装载值，启动六路互补PWM输出和CAN
 * 测试接口。ADC触发由AdcService统一启动。
 *
 * @return CAN和PWM输出全部启动成功时返回true，否则返回false。
 *
 * @pre MX_CAN_Init()和MX_TIM1_Init()已经执行完成。
 */
bool Test_Init(void);

/**
 * @brief 执行10 ms测试任务。
 *
 * 读取测试输入并处理CAN回环消息。该函数在主循环上下文执行，禁止
 * 阻塞或从中断上下文调用。
 *
 * @pre Test_Init()已经成功完成。
 */
void Test_Task(void);

/**
 * @brief 设置指定互补PWM通道组的比较预装载值。
 *
 * @param[in] channel 测试PWM通道，取值范围为TEST_PWM_CHANNEL_U至
 *                    TEST_PWM_CHANNEL_W。
 * @param[in] preload 比较预装载值，允许范围为0至当前TIM1自动重装载值。
 *
 * @return 参数有效且预装载值完成写入时返回true，否则返回false。
 *
 * @pre Test_Init()已经成功完成。
 */
void Test_SetPwmCompare(Test_PwmChannelType channel, uint16_t preload);

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
void Test_SetAdcTriggerCompare(uint16_t preload);

/**
 * @brief 使用VOFA+ JustFloat协议发送三路浮点数据。
 *
 * 按小端字节序依次发送三个IEEE 754单精度浮点数，帧尾为
 * 0x00、0x00、0x80、0x7F。
 *
 * @param[in] channel_1 第一路浮点数据。
 * @param[in] channel_2 第二路浮点数据。
 * @param[in] channel_3 第三路浮点数据。
 *
 * @return 串口发送成功时返回true，否则返回false。
 *
 * @pre USART1已完成初始化。
 */
bool Test_SendJustFloat(float channel_1,
                        float channel_2,
                        float channel_3);

#endif /* TEST_H */

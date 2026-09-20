#include "Driver_Test.h"
#include "Board_Adc.h"
#include "Dio.h"
#include "Pwm.h"
#include "watchdog.h"

/* ADC 测试的读取周期，单位 ms */
#define DRIVER_TEST_ADC_PERIOD_MS (10U)

/* 供调试器（Watch / Live Watch）观察的全局变量 */
volatile uint16_t Driver_Test_AdcValue[ADC_CH_ID_COUNT] = {0U};
volatile uint8_t  Driver_Test_AdcValid[ADC_CH_ID_COUNT] = {0U};
volatile uint32_t Driver_Test_AdcCycle = 0U;

static void Driver_Test_Dio(void)
{
  Dio_WriteChannel(DIO_CHANNEL_LED0, STD_LOW);
  Dio_WriteChannel(DIO_CHANNEL_LED1, STD_LOW);
  Dio_WriteChannel(DIO_CHANNEL_PM1_CTRL_SD, STD_LOW);
}

static void Driver_Test_Pwm(void)
{
  Pwm_Set_UVW_CCR(2100U, 4200U, 6300U);
  Pwm_Start();
  Pwm_StartAdcTrigger();
  //Pwm_StopAdcTrigger();
  //Pwm_Stop();
}

static void Driver_Test_Watchdog(void)
{
  Watchdog_Reload(1000U);
  Watchdog_Refresh();
}

/*
 * @brief 初始化并开启ADC转换
 *
 * @return void
 */
static void Driver_Test_Adc(void)
{
  Adc_Init();
  (void)Adc_Start();
}

/*
 * @brief ADC周期测试任务：重新装载规则组并读取全部逻辑通道
 *
 * @return void
 */
void Driver_Test_Loop(void)
{
  uint32_t channel = 0U;
  Adc_ValueType value = 0U;

  /* 规则组为单次转换，需每个周期重新装载触发 */
  (void)Adc_Start();

  for (channel = 0U; channel < (uint32_t)ADC_CH_ID_COUNT; channel++)
  {
    if (Adc_ReadChannel((Adc_ChannelType)channel, &value) == ADC_RESULT_OK)
    {
      Driver_Test_AdcValue[channel] = (uint16_t)value;
      Driver_Test_AdcValid[channel] = 1U;
    }
    else
    {
      Driver_Test_AdcValid[channel] = 0U;
    }
  }

  Driver_Test_AdcCycle++;

  HAL_Delay(DRIVER_TEST_ADC_PERIOD_MS);
}

void Driver_Test_Run(void)
{
 
  Driver_Test_Dio();
  Driver_Test_Pwm();
  Driver_Test_Watchdog();
  Driver_Test_Adc();
}


#include "Driver_Test.h"
#include "Dio.h"
#include "Pwm.h"
#include "watchdog.h"


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

void Driver_Test_Run(void)
{
 
  Driver_Test_Dio();
  Driver_Test_Pwm();
  Driver_Test_Watchdog();
}


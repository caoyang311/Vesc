#ifndef DRIVER_TEST_H
#define DRIVER_TEST_H

#include <stdint.h>
#include "Board_Adc.h"

/* 供调试器（Watch / Live Watch）观察的ADC测试结果，索引为Adc_ChannelType */
extern volatile uint16_t Driver_Test_AdcValue[ADC_CH_ID_COUNT];
extern volatile uint8_t  Driver_Test_AdcValid[ADC_CH_ID_COUNT];
extern volatile uint32_t Driver_Test_AdcCycle;

void Driver_Test_Run(void);

/**
 * @brief ADC周期测试任务，需在主循环中反复调用
 */
void Driver_Test_Loop(void);
    
#endif /* DRIVER_TEST_H */

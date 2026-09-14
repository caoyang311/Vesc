#ifndef VERIFY_H
#define VERIFY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "adc.h"
#include "gpio.h"
#include "time.h"

typedef struct
{
  uint32_t adc_regular_half;
  uint32_t adc_regular_complete;
  uint32_t adc_injected_complete;
  uint32_t adc_error;
  uint32_t dma_error;
  uint32_t gpio_toggle;
  uint16_t adc_regular_data[2];
} VerifyStatus;

void Verify_Init(void);
HAL_StatusTypeDef Verify_Start(void);
void Verify_MainFunction(void);
const VerifyStatus *Verify_GetStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* VERIFY_H */

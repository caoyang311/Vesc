#include "verify.h"

static VerifyStatus verify_status = {0};
static uint32_t verify_last_toggle_tick = 0U;

void Verify_Init(void)
{
  verify_status = (VerifyStatus){0};
  verify_last_toggle_tick = Time_GetTickMs();
}

HAL_StatusTypeDef Verify_Start(void)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_ADC_Start_DMA(&hadc1,
                             (uint32_t *)verify_status.adc_regular_data,
                             2U);
  if (status != HAL_OK)
  {
    verify_status.adc_error++;
  }
  else
  {
    status = HAL_ADCEx_InjectedStart_IT(&hadc2);
    if (status != HAL_OK)
    {
      verify_status.adc_error++;
    }
  }

  return status;
}

void Verify_MainFunction(void)
{
  uint32_t tick = Time_GetTickMs();

  if (Time_IsElapsed(verify_last_toggle_tick, 500U) != 0U)
  {
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    verify_status.gpio_toggle++;
    verify_last_toggle_tick = tick;
  }
}

const VerifyStatus *Verify_GetStatus(void)
{
  return &verify_status;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    verify_status.adc_regular_half++;
  }
  else
  {
    /* No action. */
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    verify_status.adc_regular_complete++;
  }
  else
  {
    /* No action. */
  }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC2)
  {
    verify_status.adc_injected_complete++;
  }
  else
  {
    /* No action. */
  }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  if ((hadc->Instance == ADC1) || (hadc->Instance == ADC2))
  {
    verify_status.adc_error++;
  }
  else
  {
    /* No action. */
  }
}

void HAL_DMA_ErrorCallback(DMA_HandleTypeDef *hdma)
{
  if (hdma->Instance == DMA2_Stream0)
  {
    verify_status.dma_error++;
  }
  else
  {
    /* No action. */
  }
}

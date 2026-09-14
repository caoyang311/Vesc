#include "power_stage.h"
#include "gpio.h"
#include "tim.h"

static PowerStage_State power_stage_state = POWER_STAGE_STATE_SAFE_OFF;
static PowerStage_FaultMask power_stage_faults = POWER_STAGE_FAULT_NONE;

static HAL_StatusTypeDef PowerStage_StopPwm(void)
{
  HAL_StatusTypeDef status = HAL_OK;
  HAL_StatusTypeDef channel_status = HAL_OK;

  channel_status = HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  if (channel_status != HAL_OK)
  {
    status = channel_status;
  }
  else
  {
    /* Continue stopping all channels. */
  }

  channel_status = HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  if (channel_status != HAL_OK)
  {
    status = channel_status;
  }
  else
  {
    /* Continue stopping all channels. */
  }

  channel_status = HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  if (channel_status != HAL_OK)
  {
    status = channel_status;
  }
  else
  {
    /* Continue stopping all channels. */
  }

  channel_status = HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  if (channel_status != HAL_OK)
  {
    status = channel_status;
  }
  else
  {
    /* Continue stopping all channels. */
  }

  channel_status = HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  if (channel_status != HAL_OK)
  {
    status = channel_status;
  }
  else
  {
    /* Continue stopping all channels. */
  }

  channel_status = HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3); /* Stop the complementary W-phase output. */
  if (channel_status != HAL_OK)
  {
    status = channel_status;
  }
  else
  {
    /* Continue with the ADC trigger channel. */
  }

  channel_status = HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
  if (channel_status != HAL_OK)
  {
    status = channel_status;
  }
  else
  {
    /* TIM1 CH4 trigger output was stopped. */
  }

  return status;
}

static void PowerStage_DisableGate(void)
{
  HAL_GPIO_WritePin(PM1_CTRL_SD_GPIO_Port, PM1_CTRL_SD_Pin, GPIO_PIN_RESET);
}

static void PowerStage_EnableGate(void)
{
  HAL_GPIO_WritePin(PM1_CTRL_SD_GPIO_Port, PM1_CTRL_SD_Pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef PowerStage_StartPwm(void)
{
  HAL_StatusTypeDef status = HAL_OK;
  HAL_StatusTypeDef channel_status = HAL_OK;

  channel_status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  if (channel_status != HAL_OK)
  {
    status = channel_status;
  }
  else
  {
    /* Continue starting all channels. */
  }

  channel_status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  if (channel_status != HAL_OK)
  {
    status = channel_status;
  }
  else
  {
    /* Continue starting all channels. */
  }

  channel_status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  if (channel_status != HAL_OK)
  {
    status = channel_status;
  }
  else
  {
    /* Continue starting all channels. */
  }

  channel_status = HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  if (channel_status != HAL_OK)
  {
    status = channel_status;
  }
  else
  {
    /* Continue starting all channels. */
  }

  channel_status = HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  if (channel_status != HAL_OK)
  {
    status = channel_status;
  }
  else
  {
    /* Continue starting all channels. */
  }

  channel_status = HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  if (channel_status != HAL_OK)
  {
    status = channel_status;
  }
  else
  {
    /* Continue with the ADC trigger channel. */
  }

  channel_status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  if (channel_status != HAL_OK)
  {
    status = channel_status;
  }
  else
  {
    /* TIM1 CH4 is the ADC injected conversion trigger source. */
  }

  return status;
}

static PowerStage_Duty PowerStage_GetFixedDuty(void)
{
  PowerStage_Duty duty = {0};

  duty.u = 2100U;
  duty.v = 4200U;
  duty.w = 6300U;

  return duty;
}

void PowerStage_Init(void)
{
  HAL_StatusTypeDef pwm_status = HAL_OK;

  power_stage_state = POWER_STAGE_STATE_SAFE_OFF;
  power_stage_faults = POWER_STAGE_FAULT_NONE;
  pwm_status = PowerStage_StopPwm();
  PowerStage_DisableGate();

  if (pwm_status != HAL_OK)
  {
    power_stage_faults |= POWER_STAGE_FAULT_HAL;
    power_stage_state = POWER_STAGE_STATE_FAULT_LATCHED;
  }
  else
  {
    /* Safe-off state is maintained after initialization. */
  }
}

PowerStage_Result PowerStage_Ready(void)
{
  PowerStage_Result result = POWER_STAGE_ERROR;

  if (power_stage_faults != POWER_STAGE_FAULT_NONE)
  {
    result = POWER_STAGE_FAULT_ACTIVE;
  }
  else if (power_stage_state == POWER_STAGE_STATE_SAFE_OFF)
  {
    power_stage_state = POWER_STAGE_STATE_READY;
    result = POWER_STAGE_OK;
  }
  else
  {
    result = POWER_STAGE_INVALID_STATE;
  }

  return result;
}

PowerStage_Result PowerStage_Start(void)
{
  PowerStage_Result result = POWER_STAGE_ERROR;
  HAL_StatusTypeDef pwm_status = HAL_OK;
  PowerStage_Duty fixed_duty = {0};

  if (power_stage_faults != POWER_STAGE_FAULT_NONE)
  {
    result = POWER_STAGE_FAULT_ACTIVE;
  }
  else if (power_stage_state == POWER_STAGE_STATE_READY)
  {
    power_stage_state = POWER_STAGE_STATE_STARTING;
    fixed_duty = PowerStage_GetFixedDuty();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, fixed_duty.u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, fixed_duty.v);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, fixed_duty.w);
    pwm_status = PowerStage_StartPwm();

    if (pwm_status != HAL_OK)
    {
      (void)PowerStage_StopPwm();
      PowerStage_DisableGate();
      power_stage_faults |= POWER_STAGE_FAULT_STARTUP;
      power_stage_faults |= POWER_STAGE_FAULT_HAL;
      power_stage_state = POWER_STAGE_STATE_FAULT_LATCHED;
      result = POWER_STAGE_ERROR;
    }
    else
    {
      PowerStage_EnableGate();
      power_stage_state = POWER_STAGE_STATE_RUNNING;
      result = POWER_STAGE_OK;
    }
  }
  else
  {
    result = POWER_STAGE_INVALID_STATE;
  }

  return result;
}

PowerStage_Result PowerStage_ScopeVerificationStart(void)
{
  PowerStage_Result result = POWER_STAGE_ERROR;

  result = PowerStage_Ready();
  if (result == POWER_STAGE_OK)
  {
    result = PowerStage_Start();
  }
  else
  {
    /* Keep the power stage disabled when it is not safe to start. */
  }

  return result;
}

PowerStage_Result PowerStage_Stop(void)
{
  PowerStage_Result result = POWER_STAGE_ERROR;
  HAL_StatusTypeDef pwm_status = HAL_OK;

  pwm_status = PowerStage_StopPwm();
  PowerStage_DisableGate();

  if (pwm_status != HAL_OK)
  {
    power_stage_faults |= POWER_STAGE_FAULT_HAL;
    power_stage_state = POWER_STAGE_STATE_FAULT_LATCHED;
    result = POWER_STAGE_ERROR;
  }
  else if (power_stage_state == POWER_STAGE_STATE_FAULT_LATCHED)
  {
    result = POWER_STAGE_FAULT_ACTIVE;
  }
  else if ((power_stage_state == POWER_STAGE_STATE_STARTING) ||
           (power_stage_state == POWER_STAGE_STATE_RUNNING) ||
           (power_stage_state == POWER_STAGE_STATE_STOPPING) ||
           (power_stage_state == POWER_STAGE_STATE_READY) ||
           (power_stage_state == POWER_STAGE_STATE_SAFE_OFF))
  {
    power_stage_state = POWER_STAGE_STATE_SAFE_OFF;
    result = POWER_STAGE_OK;
  }
  else
  {
    result = POWER_STAGE_INVALID_STATE;
  }

  return result;
}

PowerStage_Result PowerStage_SetDuty(PowerStage_Duty duty)
{
  PowerStage_Result result = POWER_STAGE_ERROR;
  uint32_t period = 0U;

  period = (uint32_t)htim1.Init.Period;

  if (power_stage_state != POWER_STAGE_STATE_RUNNING)
  {
    result = POWER_STAGE_INVALID_STATE;
  }
  else if (((uint32_t)duty.u > period) ||
           ((uint32_t)duty.v > period) ||
           ((uint32_t)duty.w > period))
  {
    result = POWER_STAGE_INVALID_DUTY;
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty.u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty.v);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty.w);
    result = POWER_STAGE_OK;
  }

  return result;
}

PowerStage_Result PowerStage_ClearFault(void)
{
  PowerStage_Result result = POWER_STAGE_ERROR;
  uint32_t break_flag = 0U;

  break_flag = __HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK);

  if (power_stage_state != POWER_STAGE_STATE_FAULT_LATCHED)
  {
    result = POWER_STAGE_INVALID_STATE;
  }
  else if (break_flag != 0U)
  {
    result = POWER_STAGE_FAULT_ACTIVE;
  }
  else
  {
    if (PowerStage_StopPwm() != HAL_OK)
    {
      PowerStage_DisableGate();
      power_stage_faults |= POWER_STAGE_FAULT_HAL;
      power_stage_state = POWER_STAGE_STATE_FAULT_LATCHED;
      result = POWER_STAGE_ERROR;
    }
    else
    {
      PowerStage_DisableGate();
      power_stage_faults = POWER_STAGE_FAULT_NONE;
      power_stage_state = POWER_STAGE_STATE_SAFE_OFF;
      result = POWER_STAGE_OK;
    }
  }

  return result;
}

PowerStage_State PowerStage_GetState(void)
{
  return power_stage_state;
}

PowerStage_FaultMask PowerStage_GetFaults(void)
{
  return power_stage_faults;
}

void PowerStage_BreakHandler(void)
{
  power_stage_faults |= POWER_STAGE_FAULT_BREAK;
  power_stage_state = POWER_STAGE_STATE_FAULT_LATCHED;
  (void)PowerStage_StopPwm();
  PowerStage_DisableGate();
}

void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim1)
  {
    PowerStage_BreakHandler();
  }
  else
  {
    /* Ignore Break callbacks from other timers. */
  }
}

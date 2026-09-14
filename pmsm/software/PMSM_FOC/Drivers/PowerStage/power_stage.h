#ifndef POWER_STAGE_H
#define POWER_STAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/**
 * @brief Power-stage operating states.
 */
typedef enum
{
  POWER_STAGE_STATE_SAFE_OFF = 0U,
  POWER_STAGE_STATE_READY,
  POWER_STAGE_STATE_STARTING,
  POWER_STAGE_STATE_RUNNING,
  POWER_STAGE_STATE_STOPPING,
  POWER_STAGE_STATE_FAULT_LATCHED
} PowerStage_State;

/**
 * @brief Power-stage operation results.
 */
typedef enum
{
  POWER_STAGE_OK = 0U,
  POWER_STAGE_ERROR,
  POWER_STAGE_INVALID_STATE,
  POWER_STAGE_INVALID_DUTY,
  POWER_STAGE_FAULT_ACTIVE
} PowerStage_Result;

typedef uint32_t PowerStage_FaultMask;

#define POWER_STAGE_FAULT_NONE      (0U)
#define POWER_STAGE_FAULT_BREAK     (1UL << 0U)
#define POWER_STAGE_FAULT_STARTUP   (1UL << 1U)
#define POWER_STAGE_FAULT_PARAMETER (1UL << 2U)
#define POWER_STAGE_FAULT_HAL       (1UL << 3U)

/**
 * @brief Three-phase timer compare values.
 */
typedef struct
{
  uint16_t u;
  uint16_t v;
  uint16_t w;
} PowerStage_Duty;

/**
 * @brief Initialize the power-stage software state.
 *
 * The v0.5 implementation also provides controlled Gate/SD re-enable
 * during a validated PWM restart.
 */
void PowerStage_Init(void);

/**
 * @brief Move the power stage to the ready state.
 * @return Operation result.
 */
PowerStage_Result PowerStage_Ready(void);

/**
 * @brief Start the power-stage outputs.
 * @return Operation result.
 */
PowerStage_Result PowerStage_Start(void);

/**
 * @brief Start the fixed-duty oscilloscope verification output.
 * @return Operation result.
 */
PowerStage_Result PowerStage_ScopeVerificationStart(void);

/**
 * @brief Stop the power-stage outputs and enter safe-off state.
 * @return Operation result.
 */
PowerStage_Result PowerStage_Stop(void);

/**
 * @brief Set the three-phase timer compare values.
 * @param duty Three-phase compare values.
 * @return Operation result.
 */
PowerStage_Result PowerStage_SetDuty(PowerStage_Duty duty);

/**
 * @brief Clear a latched power-stage fault when recovery conditions are met.
 * @return Operation result.
 */
PowerStage_Result PowerStage_ClearFault(void);

/**
 * @brief Get the current power-stage state.
 * @return Current state.
 */
PowerStage_State PowerStage_GetState(void);

/**
 * @brief Get the currently latched power-stage faults.
 * @return Fault bit mask.
 */
PowerStage_FaultMask PowerStage_GetFaults(void);

/**
 * @brief Process a TIM1 Break event.
 */
void PowerStage_BreakHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* POWER_STAGE_H */

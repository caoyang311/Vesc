#include "Board_Adc.h"
#include "Adc_Config.h"

#include <stddef.h>

/**
 * @brief ADC channel configuration structure.
 */
typedef struct
{
    Adc_InstanceType    Instance;
    uint32_t            HwChannel;
    uint32_t            SamplingTime;
    uint8_t             Rank;
    Adc_ChannelKindType Kind;
    uint8_t             Enabled;
} Adc_ChannelConfigType;

/**
 * @brief ADC channel configuration table.
 *
 * Indexed by Adc_ChannelType. Adding a logical channel only requires a new
 * entry here plus the matching hardware mapping macros in Adc_Config.h.
 */
static const Adc_ChannelConfigType Adc_ChannelConfig[ADC_CONFIGURED_CHANNEL_COUNT] =
{
    [ADC_CH_ID_PHASE_U_CURRENT] =
    {
        ADC_CH_PHASE_U_CURRENT_INSTANCE,
        ADC_CH_PHASE_U_CURRENT_HW_CHANNEL,
        ADC_CH_PHASE_U_CURRENT_SAMPLING_TIME,
        ADC_CH_PHASE_U_CURRENT_RANK,
        ADC_CH_PHASE_U_CURRENT_KIND,
        ADC_CH_PHASE_U_CURRENT_ENABLED,
    },
    [ADC_CH_ID_PHASE_V_CURRENT] =
    {
        ADC_CH_PHASE_V_CURRENT_INSTANCE,
        ADC_CH_PHASE_V_CURRENT_HW_CHANNEL,
        ADC_CH_PHASE_V_CURRENT_SAMPLING_TIME,
        ADC_CH_PHASE_V_CURRENT_RANK,
        ADC_CH_PHASE_V_CURRENT_KIND,
        ADC_CH_PHASE_V_CURRENT_ENABLED,
    },
    [ADC_CH_ID_PHASE_W_CURRENT] =
    {
        ADC_CH_PHASE_W_CURRENT_INSTANCE,
        ADC_CH_PHASE_W_CURRENT_HW_CHANNEL,
        ADC_CH_PHASE_W_CURRENT_SAMPLING_TIME,
        ADC_CH_PHASE_W_CURRENT_RANK,
        ADC_CH_PHASE_W_CURRENT_KIND,
        ADC_CH_PHASE_W_CURRENT_ENABLED,
    },
    [ADC_CH_ID_DC_BUS_VOLTAGE] =
    {
        ADC_CH_DC_BUS_VOLTAGE_INSTANCE,
        ADC_CH_DC_BUS_VOLTAGE_HW_CHANNEL,
        ADC_CH_DC_BUS_VOLTAGE_SAMPLING_TIME,
        ADC_CH_DC_BUS_VOLTAGE_RANK,
        ADC_CH_DC_BUS_VOLTAGE_KIND,
        ADC_CH_DC_BUS_VOLTAGE_ENABLED,
    },
    [ADC_CH_ID_POWER_STAGE_TEMP] =
    {
        ADC_CH_POWER_STAGE_TEMP_INSTANCE,
        ADC_CH_POWER_STAGE_TEMP_HW_CHANNEL,
        ADC_CH_POWER_STAGE_TEMP_SAMPLING_TIME,
        ADC_CH_POWER_STAGE_TEMP_RANK,
        ADC_CH_POWER_STAGE_TEMP_KIND,
        ADC_CH_POWER_STAGE_TEMP_ENABLED,
    }
};

_Static_assert((sizeof(Adc_ChannelConfig) / sizeof(Adc_ChannelConfig[0])) ==
                   ADC_CONFIGURED_CHANNEL_COUNT,
               "Adc channel configuration count mismatch");

/**
 * @brief ADC instance handle table, indexed by Adc_InstanceType.
 */
static ADC_HandleTypeDef *const Adc_InstanceHandles[ADC_INSTANCE_COUNT] =
{
    [ADC_INSTANCE_1] = &hadc1,
    [ADC_INSTANCE_2] = &hadc2
};

/**
 * @brief HAL injected rank selectors, indexed by (Rank - 1U).
 */
static const uint32_t Adc_InjectedRankMap[ADC_INJECTED_RANK_MAX] =
{
    ADC_INJECTED_RANK_1,
    ADC_INJECTED_RANK_2,
    ADC_INJECTED_RANK_3,
    ADC_INJECTED_RANK_4
};

/* Module state -------------------------------------------------------------*/

static uint8_t Adc_Initialized = 0U;
static uint8_t Adc_RegularChannelCount[ADC_INSTANCE_COUNT] = {0};
static uint8_t Adc_InjectedChannelCount[ADC_INSTANCE_COUNT] = {0};

/* Results written by the DMA / injected interrupts, indexed by (Rank - 1U). */
static volatile uint16_t Adc_RegularDmaBuffer[ADC_INSTANCE_COUNT][ADC_REGULAR_RANK_MAX] = {{0}};
static volatile Adc_ValueType Adc_InjectedSnapshot[ADC_INSTANCE_COUNT][ADC_INJECTED_RANK_MAX] = {{0}};

/* Freshness flag per logical channel, cleared on stop and set by the callbacks. */
static volatile uint8_t Adc_ChannelValid[ADC_CONFIGURED_CHANNEL_COUNT] = {0};
static volatile uint8_t Adc_ErrorFlag = 0U;

/**
 * @brief Resets the module state and derives the per-instance group bookkeeping.
 *
 * @note No hardware access is performed. The peripherals themselves are still
 *       configured by the CubeMX generated MX_ADCx_Init() functions.
 */
void Adc_Init(void)
{
    uint32_t instance = 0U;
    uint32_t channel = 0U;
    uint32_t rank = 0U;

    for (instance = 0U; instance < (uint32_t)ADC_INSTANCE_COUNT; instance++)
    {
        Adc_RegularChannelCount[instance] = 0U;
        Adc_InjectedChannelCount[instance] = 0U;

        for (rank = 0U; rank < (uint32_t)ADC_REGULAR_RANK_MAX; rank++)
        {
            Adc_RegularDmaBuffer[instance][rank] = 0U;
        }

        for (rank = 0U; rank < (uint32_t)ADC_INJECTED_RANK_MAX; rank++)
        {
            Adc_InjectedSnapshot[instance][rank] = 0U;
        }
    }

    for (channel = 0U; channel < (uint32_t)ADC_CONFIGURED_CHANNEL_COUNT; channel++)
    {
        Adc_ChannelValid[channel] = 0U;

        if (Adc_ChannelConfig[channel].Enabled != 0U)
        {
            if (Adc_ChannelConfig[channel].Kind == ADC_CH_KIND_REGULAR)
            {
                Adc_RegularChannelCount[Adc_ChannelConfig[channel].Instance]++;
            }
            else
            {
                Adc_InjectedChannelCount[Adc_ChannelConfig[channel].Instance]++;
            }
        }
    }

    Adc_ErrorFlag = 0U;
    Adc_Initialized = 1U;
}

/**
 * @brief Stops all conversions and resets the module state.
 */
void Adc_DeInit(void)
{
    if (Adc_Initialized != 0U)
    {
        (void)Adc_Stop();
        Adc_ErrorFlag = 0U;
        Adc_Initialized = 0U;
    }
}

/**
 * @brief Starts the configured channels of every ADC instance.
 *
 * @return ADC_RESULT_OK when every start request was accepted;
 *         ADC_RESULT_NOT_INITIALIZED when Adc_Init() was not called;
 *         ADC_RESULT_HAL_ERROR when the HAL rejected a start request.
 *
 * @note Regular channels are moved by circular DMA, injected channels are
 *       armed and then triggered by TIM1_CH4. This function is not idempotent:
 *       the regular group is configured for single conversion, therefore the
 *       caller shall invoke it again on every sampling period to re-trigger
 *       the regular conversions.
 */
Adc_Result Adc_Start(void)
{
    Adc_Result result = ADC_RESULT_OK;
    uint32_t instance = 0U;

    if (Adc_Initialized == 0U)
    {
        result = ADC_RESULT_NOT_INITIALIZED;
    }
    else
    {
        Adc_ErrorFlag = 0U;

        for (instance = (uint32_t)ADC_INSTANCE_1; instance < (uint32_t)ADC_INSTANCE_COUNT; instance++)
        {
            if (Adc_RegularChannelCount[instance] > 0U)
            {
                /* Re-arming an already running circular DMA reports HAL_BUSY:
                   the software start is issued anyway, so only HAL_ERROR is
                   treated as a failure. */
                if (HAL_ADC_Start_DMA(Adc_InstanceHandles[instance],
                                      (uint32_t *)Adc_RegularDmaBuffer[instance],
                                      (uint32_t)Adc_RegularChannelCount[instance]) == HAL_ERROR)
                {
                    Adc_ErrorFlag = 1U;
                    result = ADC_RESULT_HAL_ERROR;
                }
            }

            if (Adc_InjectedChannelCount[instance] > 0U)
            {
                if (HAL_ADCEx_InjectedStart_IT(Adc_InstanceHandles[instance]) != HAL_OK)
                {
                    Adc_ErrorFlag = 1U;
                    result = ADC_RESULT_HAL_ERROR;
                }
            }
        }
    }

    return result;
}

/**
 * @brief Stops the configured channels of every ADC instance.
 *
 * @return ADC_RESULT_OK when every stop request was accepted;
 *         ADC_RESULT_NOT_INITIALIZED when Adc_Init() was not called;
 *         ADC_RESULT_HAL_ERROR when the HAL rejected a stop request.
 */
Adc_Result Adc_Stop(void)
{
    Adc_Result result = ADC_RESULT_OK;
    uint32_t instance = 0U;
    uint32_t channel = 0U;

    if (Adc_Initialized == 0U)
    {
        result = ADC_RESULT_NOT_INITIALIZED;
    }
    else
    {
        /* Regular groups are stopped first: the injected stop is rejected
           while a regular conversion is still pending on the same instance. */
        for (instance = (uint32_t)ADC_INSTANCE_1; instance < (uint32_t)ADC_INSTANCE_COUNT; instance++)
        {
            if ((Adc_RegularChannelCount[instance] > 0U) &&
                (HAL_ADC_Stop_DMA(Adc_InstanceHandles[instance]) != HAL_OK))
            {
                result = ADC_RESULT_HAL_ERROR;
            }
        }

        for (instance = (uint32_t)ADC_INSTANCE_1; instance < (uint32_t)ADC_INSTANCE_COUNT; instance++)
        {
            if ((Adc_InjectedChannelCount[instance] > 0U) &&
                (HAL_ADCEx_InjectedStop_IT(Adc_InstanceHandles[instance]) != HAL_OK))
            {
                result = ADC_RESULT_HAL_ERROR;
            }
        }

        for (channel = 0U; channel < (uint32_t)ADC_CONFIGURED_CHANNEL_COUNT; channel++)
        {
            Adc_ChannelValid[channel] = 0U;
        }
    }

    return result;
}

/**
 * @brief Reads the latest raw sample of a logical channel.
 *
 * @param[in]  ChannelId Logical channel identifier.
 * @param[out] Value     Destination of the raw conversion result.
 *
 * @return ADC_RESULT_OK on success;
 *         ADC_RESULT_INVALID_PARAMETER when Value is NULL;
 *         ADC_RESULT_NOT_INITIALIZED when Adc_Init() was not called;
 *         ADC_RESULT_INVALID_CHANNEL when the channel is out of range or disabled;
 *         ADC_RESULT_HAL_ERROR when the module latched a conversion error;
 *         ADC_RESULT_NOT_READY when no sample has been captured yet.
 */
Adc_Result Adc_ReadChannel(Adc_ChannelType ChannelId, Adc_ValueType *Value)
{
    Adc_Result result = ADC_RESULT_OK;
    const Adc_ChannelConfigType *config = NULL;

    if (Value == NULL)
    {
        result = ADC_RESULT_INVALID_PARAMETER;
    }
    else if (Adc_Initialized == 0U)
    {
        result = ADC_RESULT_NOT_INITIALIZED;
    }
    else if ((uint32_t)ChannelId >= (uint32_t)ADC_CONFIGURED_CHANNEL_COUNT)
    {
        result = ADC_RESULT_INVALID_CHANNEL;
    }
    else
    {
        config = &Adc_ChannelConfig[ChannelId];

        if (config->Enabled == 0U)
        {
            result = ADC_RESULT_INVALID_CHANNEL;
        }
        else if (Adc_ErrorFlag != 0U)
        {
            result = ADC_RESULT_HAL_ERROR;
        }
        else if (Adc_ChannelValid[ChannelId] == 0U)
        {
            result = ADC_RESULT_NOT_READY;
        }
        else if (config->Kind == ADC_CH_KIND_INJECTED)
        {
            *Value = Adc_InjectedSnapshot[config->Instance][config->Rank - 1U];
        }
        else
        {
            *Value = Adc_RegularDmaBuffer[config->Instance][config->Rank - 1U];
        }
    }

    return result;
}

/**
 * @brief Latches the regular group samples once the DMA transfer completed.
 *
 * @param[in] hadc ADC handle that completed a regular conversion sequence.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    uint32_t channel = 0U;
    const Adc_ChannelConfigType *config = NULL;

    for (channel = 0U; channel < (uint32_t)ADC_CONFIGURED_CHANNEL_COUNT; channel++)
    {
        config = &Adc_ChannelConfig[channel];

        if ((config->Enabled != 0U) &&
            (config->Kind == ADC_CH_KIND_REGULAR) &&
            (Adc_InstanceHandles[config->Instance] == hadc))
        {
            Adc_ChannelValid[channel] = 1U;
        }
    }
}

/**
 * @brief Latches the injected group results as a coherent snapshot.
 *
 * @param[in] hadc ADC handle that completed an injected conversion sequence.
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    uint32_t channel = 0U;
    const Adc_ChannelConfigType *config = NULL;

    for (channel = 0U; channel < (uint32_t)ADC_CONFIGURED_CHANNEL_COUNT; channel++)
    {
        config = &Adc_ChannelConfig[channel];

        if ((config->Enabled != 0U) &&
            (config->Kind == ADC_CH_KIND_INJECTED) &&
            (Adc_InstanceHandles[config->Instance] == hadc))
        {
            Adc_InjectedSnapshot[config->Instance][config->Rank - 1U] =
                (Adc_ValueType)HAL_ADCEx_InjectedGetValue(
                    hadc, Adc_InjectedRankMap[config->Rank - 1U]);
            Adc_ChannelValid[channel] = 1U;
        }
    }
}

/**
 * @brief Latches a conversion error reported by the ADC driver.
 *
 * @param[in] hadc ADC handle that raised the error.
 */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    Adc_ErrorFlag = 1U;
}

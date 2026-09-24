#include "Board_Dac.h"
#include "dac.h"

/**
 * @brief DAC1 和 DAC2 是否已经启动。
 */
static uint8_t Board_Dac_Initialized = 0U;

/**
 * @brief 检查 DAC 原始输出值。
 *
 * @param[in] Value 待检查的 DAC 原始值。
 * @return 0U 值超出 12 位范围；1U 值有效。
 */
static uint8_t Board_Dac_IsValueValid(uint16_t Value)
{
    uint8_t result = 0U;

    if ((uint32_t)Value <= BOARD_DAC_VALUE_MAX)
    {
        result = 1U;
    }
    else
    {
        result = 0U;
    }

    return result;
}

Board_Dac_ResultType Board_Dac_Init(void)
{
    Board_Dac_ResultType result = BOARD_DAC_RESULT_HAL_ERROR;

    if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) == HAL_OK)
    {
        if (HAL_DAC_Start(&hdac, DAC_CHANNEL_2) == HAL_OK)
        {
            Board_Dac_Initialized = 1U;
            result = BOARD_DAC_RESULT_OK;
        }
        else
        {
            (void)HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);
            Board_Dac_Initialized = 0U;
        }
    }
    else
    {
        Board_Dac_Initialized = 0U;
    }

    return result;
}

Board_Dac_ResultType Board_Dac_SetDac1(uint16_t Value)
{
    Board_Dac_ResultType result = BOARD_DAC_RESULT_HAL_ERROR;

    if (Board_Dac_IsValueValid(Value) == 0U)
    {
        result = BOARD_DAC_RESULT_INVALID_PARAMETER;
    }
    else if (Board_Dac_Initialized == 0U)
    {
        result = BOARD_DAC_RESULT_HAL_ERROR;
    }
    else if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)Value) == HAL_OK)
    {
        result = BOARD_DAC_RESULT_OK;
    }
    else
    {
        result = BOARD_DAC_RESULT_HAL_ERROR;
    }

    return result;
}

Board_Dac_ResultType Board_Dac_SetDac2(uint16_t Value)
{
    Board_Dac_ResultType result = BOARD_DAC_RESULT_HAL_ERROR;

    if (Board_Dac_IsValueValid(Value) == 0U)
    {
        result = BOARD_DAC_RESULT_INVALID_PARAMETER;
    }
    else if (Board_Dac_Initialized == 0U)
    {
        result = BOARD_DAC_RESULT_HAL_ERROR;
    }
    else if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint32_t)Value) == HAL_OK)
    {
        result = BOARD_DAC_RESULT_OK;
    }
    else
    {
        result = BOARD_DAC_RESULT_HAL_ERROR;
    }

    return result;
}

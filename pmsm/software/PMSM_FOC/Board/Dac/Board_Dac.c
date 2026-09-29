#include "Board_Dac.h"
#include "dac.h"

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
/**
 * @brief 初始化 DAC1 和 DAC2。
 */
void Board_Dac_Init(void)
{
    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
    HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
}
/**
 * @brief 设置 DAC1 原始输出值。
 *
 * @param[in] Value DAC1 原始值。
 * @return Board_Dac_ResultType 操作结果。
 */
Board_Dac_ResultType Board_Dac_SetDac1(uint16_t Value)
{
    Board_Dac_ResultType result = BOARD_DAC_RESULT_HAL_ERROR;

    if (Board_Dac_IsValueValid(Value) == 0U)
    {
        result = BOARD_DAC_RESULT_INVALID_PARAMETER;
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
/**
 * @brief 设置 DAC2 原始输出值。
 *
 * @param[in] Value DAC2 原始值。
 * @return Board_Dac_ResultType 操作结果。
 */
Board_Dac_ResultType Board_Dac_SetDac2(uint16_t Value)
{
    Board_Dac_ResultType result = BOARD_DAC_RESULT_HAL_ERROR;

    if (Board_Dac_IsValueValid(Value) == 0U)
    {
        result = BOARD_DAC_RESULT_INVALID_PARAMETER;
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

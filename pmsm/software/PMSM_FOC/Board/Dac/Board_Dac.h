#ifndef BOARD_DAC_H
#define BOARD_DAC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief DAC 模块返回状态。
 */
typedef enum
{
    BOARD_DAC_RESULT_OK = 0U,
    BOARD_DAC_RESULT_INVALID_PARAMETER,
    BOARD_DAC_RESULT_HAL_ERROR
} Board_Dac_ResultType;

/**
 * @brief DAC 原始输出值范围。
 *
 * STM32F4 DAC 为 12 位分辨率，允许的输出码值范围为 0..4095。
 */
#define BOARD_DAC_VALUE_MAX (4095U)

/**
 * @brief 启动 DAC1 和 DAC2 输出通道。
 *
 * @return BOARD_DAC_RESULT_OK 启动成功；
 *         BOARD_DAC_RESULT_HAL_ERROR HAL 启动失败。
 */
Board_Dac_ResultType Board_Dac_Init(void);

/**
 * @brief 设置 DAC1 输出值。
 *
 * @param[in] Value 12 位 DAC 原始值，范围 0..4095。
 * @return BOARD_DAC_RESULT_OK 设置成功；
 *         BOARD_DAC_RESULT_INVALID_PARAMETER 输入值超出范围；
 *         BOARD_DAC_RESULT_HAL_ERROR HAL 写入失败。
 */
Board_Dac_ResultType Board_Dac_SetDac1(uint16_t Value);

/**
 * @brief 设置 DAC2 输出值。
 *
 * @param[in] Value 12 位 DAC 原始值，范围 0..4095。
 * @return BOARD_DAC_RESULT_OK 设置成功；
 *         BOARD_DAC_RESULT_INVALID_PARAMETER 输入值超出范围；
 *         BOARD_DAC_RESULT_HAL_ERROR HAL 写入失败。
 */
Board_Dac_ResultType Board_Dac_SetDac2(uint16_t Value);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_DAC_H */

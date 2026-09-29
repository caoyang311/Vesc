/**
 * @file EncoderService.h
 * @brief TIM2增量式编码器位置与速度服务接口。
 */

#ifndef ENCODER_SERVICE_H
#define ENCODER_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

/** @brief 编码器运行状态。 */
typedef enum
{
    ENCODER_STATUS_UNINITIALIZED = 0U,
    ENCODER_STATUS_RUNNING,
    ENCODER_STATUS_CONFIG_ERROR,
    ENCODER_STATUS_HARDWARE_ERROR
} EncoderService_StatusType;

/** @brief 电机旋转方向，从电机输出轴方向观察。 */
typedef enum
{
    ENCODER_DIRECTION_STOP = 0U,
    ENCODER_DIRECTION_COUNTERCLOCKWISE,
    ENCODER_DIRECTION_CLOCKWISE
} EncoderService_DirectionType;

/** @brief 编码器服务配置。 */
typedef struct
{
    uint32_t counts_per_revolution; /**< 四倍频后的每机械转计数。 */
    uint16_t pole_pairs;            /**< 电机极对数。 */
    float speed_sample_period_s;    /**< 速度计算周期，单位s。 */
    uint32_t initial_count;         /**< 软件机械零点对应的TIM2计数。 */
} EncoderService_ConfigType;

/** @brief 编码器位置、速度和方向输出。 */
typedef struct
{
    uint32_t raw_count;                          /**< TIM2原始计数。 */
    int32_t delta_count;                         /**< 一个速度周期内的有符号计数差。 */
    uint32_t mechanical_count;                   /**< 单机械转内计数。 */
    uint16_t mechanical_angle_phase;             /**< uint16_t机械角度相位。 */
    uint16_t electrical_angle_phase;             /**< uint16_t电角度相位。 */
    float electrical_speed_rad_s;                /**< 电角速度，单位rad/s。 */
    float mechanical_speed_rpm;                  /**< 机械转速，单位RPM。 */
    EncoderService_DirectionType direction;      /**< 旋转方向。 */
    EncoderService_StatusType status;            /**< 服务运行状态。 */
} EncoderService_OutputType;

/**
 * @brief 初始化编码器服务。
 *
 * @param[in] config 配置指针，不得为NULL；计数、极对数和周期必须大于零。
 *
 * @return 配置有效时返回true，否则返回false。
 * @post 成功后内部状态已清零，但TIM2编码器接口尚未启动。
 */
bool EncoderService_Init(const EncoderService_ConfigType * config);

/**
 * @brief 启动TIM2的A/B双通道编码器接口。
 *
 * @return HAL启动成功时返回true，否则返回false。
 * @pre EncoderService_Init()已成功完成，TIM2底层初始化已完成。
 */
bool EncoderService_Start(void);

/**
 * @brief 更新机械角度和电角度。
 *
 * 本函数供10 kHz快速控制中断调用，执行时间有界且不阻塞。
 *
 * @pre EncoderService_Start()已成功完成。
 */
void EncoderService_UpdatePosition(void);

/**
 * @brief 更新机械转速、电角速度和旋转方向。
 *
 * 本函数必须由固定5 ms协作任务调用一次。机械转速和电角速度使用最近
 * 10个速度采样的滑动平均结果，窗口填满前按当前有效采样数量计算。
 *
 * @pre EncoderService_Start()已成功完成。
 */
void EncoderService_UpdateSpeed(void);

/**
 * @brief 获取编码器计算结果的一致性快照。
 *
 * @param[out] output 输出结构体指针，不得为NULL。
 *
 * @return 服务正在运行且输出指针有效时返回true，否则返回false。
 */
bool EncoderService_GetOutput(EncoderService_OutputType * output);

#endif /* ENCODER_SERVICE_H */

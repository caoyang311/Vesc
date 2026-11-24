/*
 * 文件: hall_sensor.h
 * 功能: 霍尔传感器接口(三线 UVW -> 编码器 A/B/Z 复用引脚), 电角度与转速解算
 * 创建日期: 2026-10-09
 * 修改人: rui.zhang
 *
 * 引脚复用: 与 ABZ 编码器共用 PA0(A=Hall U) / PA1(B=Hall V) / PA2(Z=Hall W)。
 * 由 motor_param.sensor_type 选择接口: 0=ABZ 编码器(TIM2), 1=霍尔(EXTI 双边沿)。
 * 切换由运行时检测参数变化自动完成(含引脚模式重配), 无需复位。
 */
#ifndef HALL_SENSOR_H_
#define HALL_SENSOR_H_

#include <stdint.h>

/* 电角度 12bit(0~4095), 与编码器路径同口径 */
#define HALL_ANGLE_MAX   4096U
#define HALL_ANGLE_HALF  2048U
/* 无效组合(0/7)连续持续时间门限: 上层据此判霍尔断线, 健康霍尔不会停在无效组合 */
#define HALL_INVALID_FAULT_MS 100U

/* 接口模式(与 motor_param.sensor_type 对应) */
#define SENSOR_TYPE_ABZ   0U
#define SENSOR_TYPE_HALL  1U

typedef struct {
    uint16_t elec_angle;     /* 插值后的电角度(12bit) */
    int32_t  speed_rpm;      /* 机械转速(带方向, 滤波后) */
    int8_t   direction;      /* 1=正转 -1=反转 0=未知 */
    uint8_t  state;          /* 当前霍尔组合(1~6 有效) */
    uint8_t  err_invalid;    /* 无效组合(0/7)事件计数 */
    uint8_t  err_transition; /* 非法转移事件计数 */
    uint16_t invalid_ms;     /* 无效组合连续持续时间(ms), 1kHz 任务累加 */
} Hall_Ctrl;

extern Hall_Ctrl g_hall;

/* 硬件接口配置/释放: 切换 PA0/PA1/PA2 在 EXTI(霍尔) 与 TIM2(编码器) 之间 */
void Hall_IF_Activate(void);     /* EXTI 双边沿接管三脚 */
void Hall_IF_Deactivate(void);   /* 交还 TIM2 编码器模式(重新初始化编码器) */

/* 运行时接口管理: 每拍调用, 检测 sensor_type 变化并完成切换(在 1kHz 慢任务中执行重配) */
void Hall_IF_Poll(uint8_t sensor_type_now);
uint8_t Hall_IF_IsActive(void);     /* 当前接口: 1=霍尔 0=ABZ */

/* 10kHz: 角度插值 + 状态机(由 pwm_isr 在霍尔模式下调用, 写 elec_angle) */
void Hall_Angle_Task(void);

/* 1kHz: 速度解算(边沿周期法 + 一阶滤波 + 零速超时), 写 g_hall.speed_rpm */
void Hall_Speed_Task(void);

/* 外部中断入口(EXTI0/1/2 共用): 边沿时间戳 + 转移查表, 尽量短 */
void Hall_EXTI_Handle(uint32_t pin);

#endif

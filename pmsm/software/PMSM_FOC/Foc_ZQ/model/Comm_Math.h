/*
 * File: Comm_Math.h  (MATLAB 1:1 镜像)
 *
 * Code generated for Simulink model 'MCU_MotorContrl_2024b'.
 * Model version      : 1.1974
 * Simulink Coder     : 24.2 (R2024b)
 *
 * 注意: 与工程已有的通用数学库 Common_Math.h 不同名(此为 MATLAB 斜坡结构体)。
 */

#ifndef Comm_Math_h_
#define Comm_Math_h_
#include "rtwtypes.h"

/* 斜坡计算 */
typedef struct {
  real32_T X_in;        /* 计算输入 x */
  real32_T Y_out;       /* 计算输出 Y */
  real32_T slope_grad;  /* 斜坡梯度 */
  uint16_T timer_count; /* 时间计数 */
  uint16_T grad_timer;  /* 梯度周期 */
} Slope_Calculate;

#endif                                 /* Comm_Math_h_ */

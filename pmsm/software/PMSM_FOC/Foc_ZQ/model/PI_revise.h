/*
 * File: PI_revise.h  (MATLAB 1:1 镜像)
 *
 * Code generated for Simulink model 'MCU_MotorContrl_2024b'.
 * Model version      : 1.1974
 * Simulink Coder     : 24.2 (R2024b)
 *
 * PI 控制器结构体。计算逻辑(PI_Contrl / PI_Contrl_Modern...)在 foc_pi.h 中以宏实现。
 */

#ifndef PI_revise_h_
#define PI_revise_h_
#include "rtwtypes.h"

/* PI 控制结构体 */
typedef struct {
  real32_T Ref;          /* 控制参考值 */
  real32_T Fbk;          /* 反馈量 */
  real32_T Kp;           /* 比例增益 */
  real32_T Ki;           /* 积分增益 */
  real32_T Kaw;          /* Back-Calculation 系数, 推荐 sqrt(Ki/Kp) */
  real32_T b;            /* 设定值权重 0~1: 1=原始行为, 0=无比例冲击 */
  real32_T upper_limit;  /* 输出上限 */
  real32_T lower_limit;  /* 输出下限 */
  real32_T DeadBand;     /* 误差死区 (对称, 绝对值, 0=禁用) */
  real32_T Err;          /* 内部状态(只读): 误差 */
  real32_T Up;           /* 内部状态(只读): 比例项 */
  real32_T Ui;           /* 内部状态(只读): 积分项 */
  real32_T Vl;           /* 上一拍未限幅的输出值 */
  real32_T PI_out;       /* PI 输出 */
  uint8_T  Zero_flag;    /* PI 清零标志 */
} PI_Params;

#endif                                 /* PI_revise_h_ */

/*
 * File: Main_Feedback.h  (MATLAB 1:1 镜像)
 *
 * Code generated for Simulink model 'MCU_MotorContrl_2024b'.
 * Model version      : 1.1974
 * Simulink Coder     : 24.2 (R2024b)
 */

#ifndef Main_Feedback_h_
#define Main_Feedback_h_
#include "rtwtypes.h"

/* 电机主反馈 */
typedef struct {
  uint16_T poles;             /* 电机极对数 */
  real32_T mech_angle;        /* 电机机械角度 0~360 */
  uint16_T elec_angle;        /* 电机电角度 */
  int32_T  meas_Spd;          /* 编码器测速 */
  uint16_T elec_angle_offset; /* 电角度偏移量 */
  int32_T  mech_Spd;          /* 电机实际转速 RPM */
  int32_T  mech_Spd_last;     /* 历史机械转速 RPM */
  uint32_T abs_mech_Spd;      /* 机械转速绝对值 RPM */
  uint16_T move_Sta;          /* 电机运转方向状态 顺/逆 */
  uint8_T  fault_Sta;         /* 故障状态 */
  real32_T angle_Spd;         /* 角速度 */
  uint16_T clear_Err;         /* 清零误差 */
  uint16_T clear_Cnt;         /* 等待计数 */
} Motor_Main_Feedback;

#endif                                 /* Main_Feedback_h_ */

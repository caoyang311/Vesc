/*
 * File: vehicle_info.h  (MATLAB 1:1 镜像)
 *
 * Code generated for Simulink model 'MCU_MotorContrl_2024b'.
 * Model version      : 1.1974
 * Simulink Coder     : 24.2 (R2024b)
 */

#ifndef vehicle_info_h_
#define vehicle_info_h_
#include "rtwtypes.h"

/* 整车相关信息 */
typedef struct {
  real32_T ABS_front_speed;          /* ABS 传感器前轮车速 */
  real32_T ABS_rear_speed;           /* ABS 传感器后轮车速 */
  real32_T vehicle_speed;            /* 综合整车车速 */
  uint8_T  vehicle_flaute_level;     /* 整车故障等级 */
  uint32_T ABS_state;                /* ABS 状态 */
  uint32_T ABS_fault_state;          /* ABS 故障状态 */
  real32_T BMS_battery_level[2];        /* BMSx 电量百分比 */
  real32_T BMS_temp[2];                 /* BMSx 温度 */
  uint32_T BMS_fault_state[2];          /* BMSx 故障状态 */
  uint32_T BMS_charge_state[2];         /* BMSx 充电状态 */
  uint32_T BMS_charge_curr[2];          /* BMSx 允许最大充电电流 */
  uint8_T  BMS_charge_curr_valid;       /* 充电电流有效位: bit0=BMS1已写入 bit1=BMS2已写入;
                                         0=整车未接入, 用缺省值; 显式写入0=BMS禁止回馈 */
  real32_T TBOX_vehicle_attitude[2];    /* 车辆姿态 0:侧倾角 1:俯仰角 */
  real32_T TBOX_panic_stop;                          /* TBOX 紧急停止信号 */
  real32_T BMS_charge_volt_max;      /* BMS 最大充电电压 V (0=未接收, 用额定+2V) */
} Vehicle_Info;

#endif                                 /* vehicle_info_h_ */

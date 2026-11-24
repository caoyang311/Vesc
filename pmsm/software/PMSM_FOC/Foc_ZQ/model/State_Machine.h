/*
 * File: State_Machine.h  (MATLAB 1:1 镜像)
 *
 * Code generated for Simulink model 'MCU_MotorContrl_2024b'.
 * Model version      : 1.1974
 * Simulink Coder     : 24.2 (R2024b)
 */

#ifndef State_Machine_h_
#define State_Machine_h_
#include "rtwtypes.h"

/* 故障状态机 */
typedef struct {
  uint8_T mos_temp_high;            /* MOS温度高 */
  uint8_T motor_temp_high;          /* 电机温度高 */
  uint8_T DC_bus_zero_drift;        /* 直流母线偏移故障 */
  uint8_T phase_Curr_zero_drift;    /* 相电流零偏故障 */
  uint8_T mos_temp_sample_fault;    /* MOS温度采样异常 */
  uint8_T motor_temp_sample_fault;  /* 电机温度采样异常 */
  uint8_T DC_bus_over_volt;         /* 直流母线过压 */
  uint8_T DC_bus_under_volt;        /* 直流母线欠压 */
  uint8_T motor_phase;              /* 缺相 */
  uint8_T over_speed;               /* 超速 */
  uint8_T motor_main_feedback_lost; /* 主反馈丢失状态 */
  uint8_T mos_temp_over;            /* MOS过温状态 */
  uint8_T motor_temp_over;          /* 电机过温状态 */
  uint8_T throttle_fault;           /* 油门转把故障 */
  uint8_T motor_phase_short_circuit;/* 电机相间短路 */
  uint8_T over_Curr_fault;          /* 过流故障 */
  uint8_T mcu_rom_fault;            /* MCU ROM故障 */
  uint8_T mcu_ram_fault;            /* MCU RAM故障 */
  uint8_T mcu_stack_overflow;       /* MCU 栈溢出 */
} Fault_State_Machine;

/* 功能状态机 */
typedef struct {
  uint8_T  unlock_driver_enable;      /* 解锁成功后驱动使能 */
  uint8_T  OTA_enable;                /* OTA使能 */
  uint8_T  self_check_Sta;            /* 自检状态 */
  uint8_T  precharge_Sta;             /* 预充状态 */
  uint8_T  start_Sta;                 /* 启动状态 */
  uint8_T  throttle_Sta;              /* 油门状态 0:空闲 1:有效 2:故障 */
  real32_T throttle_voltage;          /* 油门模拟电压 */
  uint8_T  HCC_enable;                /* 坡道驻车使能 */
  uint32_T HCC_timer_Per;             /* 坡道驻车-自动结束时间 */
  uint8_T  TCS_enable;                /* 牵引力系统使能 */
	uint8_T  TCS_state;                /* 牵引力系统干涉状态 */
  uint8_T  HDC_enable;                /* 陡坡缓降使能 */
  real32_T HDC_anagle_threshlod;      /* 陡坡缓降角度阈值 */
  uint32_T HDC_timer_Per;             /* 陡坡缓降角度持续时间 */
  uint8_T  TTF_enable;                /* 定速巡航使能 */
  uint8_T  TTF_lower_spd;             /* 定速巡航-速度下限 */
  uint8_T  TTF_taeget_spd;            /* 定速巡航-目标速度 */
  uint8_T  AUX_brake_enable;          /* 辅助刹车使能 */
  uint8_T  boost_enable;              /* 助推使能 */
  uint8_T  reverse_enable;            /* 倒车使能 */
  uint8_T  emergency_cycling_Sta;     /* 应急骑行状态 */
  uint8_T  emergency_cycling_Contrl;  /* 应急骑行控制 */
  uint8_T  emergency_cycling_spd_lower;/* 应急骑行-速度下限 */
  uint8_T  anti_theft_Sta;            /* 防盗锁车 */
  uint8_T  ER_mode;                   /* 能量回收工作模式 */
  uint8_T  ER_rate;                   /* 能量回收比率 */
  uint8_T  FD_contrl;                 /* 防盗模式 0:退出 1:进入 */
  uint16_T FD_timer_Per;              /* 防盗模式进入持续时间 ms */
  uint8_T  charge_Sta;                /* 充电状态 */
  uint8_T  catapult_start_Enable;     /* 弹射起步使能 */
  uint8_T  nitogen_boost_Enable;      /* 氮气加速使能 */
  uint8_T  fault_Sta;                 /* 故障状态 */
  uint8_T  fault_recovery_Sta;        /* 手动故障恢复状态 */
  uint8_T  identify_Sta;              /* 在线学习状态 */
  uint8_T  cushion_Sta;               /* 缓存传感器状态 */
  uint8_T  vehicie_attitude_Sta;      /* 车辆姿态状态 */
  real32_T TC_contrl;                 /* 助推控制 0:退出 1:进入助推 */
  uint8_T  handlebar_Contrl;          /* 转把控制 */
  real32_T handlebar_voltage_limt[3]; /* 转把电压上限 0:下限 1:中间 2:上限 */
  uint32_T handlebar_speed_limt[3];   /* 转把速度上限 0:下限 1:中间 2:上限 */
} Function_State_Machine;

/* 钥匙状态机 */
typedef struct {
  uint8_T system_status;       /* 系统状态 */
  uint8_T system_status_comm;  /* 由通讯口改变的系统状态 */
  uint32_T fun_state;          /* 功能状态机(32位状态寄存器, 上位机A5-07读取) 2026-08-11 上位机流程图适配 F1 */
  uint8_T control_status;      /* 控制状态 */
	uint8_T gear_status;      	 /* 挡位状态 */
} Key_State_Machine;

#endif                                 /* State_Machine_h_ */

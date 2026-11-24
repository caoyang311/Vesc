/*
 * File: ADC_sample.h  (MATLAB 1:1 镜像)
 *
 * Code generated for Simulink model 'MCU_MotorContrl_2024b'.
 * Model version      : 1.1974
 * Simulink Coder     : 24.2 (R2024b)
 *
 * 注意: Motor_Analog.PWM_HalfPerMax 字段名与工程 foc_config.h 的同名宏冲突,
 *       故本头(及 foc_data.h)所在的编译单元不得再 include "foc_config.h"。
 */

#ifndef ADC_sample_h_
#define ADC_sample_h_
#include "rtwtypes.h"

/* ADC 采样及零位标定 */
typedef struct {
  uint16_T bus_Curr;            /* 母线电流 ADC */
  uint16_T phaseU_Curr;         /* U相电流 ADC */
  uint16_T phaseV_Curr;         /* V相电流 ADC */
  uint16_T phaseW_Curr;         /* W相电流 ADC */
  uint16_T bus_Volt;            /* 母线电压 ADC */
  uint16_T MOS_Temp;            /* MOS温度 ADC */
  uint16_T mototr_Temp;         /* 电机温度 ADC */
//  uint16_T RP_speed_Volt;       /* 油门/转速电压 ADC */
  uint16_T bus_Curr_offset;     /* 母线电流零偏 ADC */
  uint16_T phaseU_Curr_offset;  /* U相电流零偏 ADC */
  uint16_T phaseV_Curr_offset;  /* V相电流零偏 ADC */
  uint16_T phaseW_Curr_offset;  /* W相电流零偏 ADC */
  uint16_T zero_flag;           /* 零位标定状态 0:未标定 1:已标定 */
  uint16_T handlebar_Volt;            /* 转把电压 ACC */
} ADC_Sample;

/* 中间变量: GPIO 输入 + SVPWM 输出 */
typedef struct {
  uint8_T  brake_gpio;        /* 刹车IO输入 */
  uint8_T  side_stand_gpio;   /* 边撑GPIO输入 */
  uint8_T  gear_bus_gpio[3];  /* 档位总线输入 */
  uint8_T  P_gpio;            /* P档GPIO输入 */
  uint8_T  boost_gpio;        /* 助推器GPIO输入 */
  uint8_T  R_gpio;            /* 倒挡GPIO输入 */
  uint8_T  CCS_gpio;          /* 定速巡航GPIO */
  uint8_T  burglar_gpio;      /* 防盗GPIO输入 */
  uint8_T  ACC_gpio;          /* 钥匙信号IO输入 */
  uint8_T  brake_o;           /* 刹车输出 */
  uint8_T  side_stand_o;      /* 边撑输出 */
  uint8_T  gear_o;            /* 档位输出 */
  uint8_T  P_o;               /* P档输出 */
  uint8_T  boost_o;           /* 助推输出 */
  uint8_T  R_o;               /* R档输出 */
  uint8_T  CCS_o;             /* 定速巡航输出 */
  uint8_T  burglar_o;         /* 防盗输出 */
  uint8_T  ACC_o;             /* 钥匙信号输出 */
  uint16_T SVPTa;             /* SVPWM 输出U相 */
  uint16_T SVPTb;             /* SVPWM 输出V相 */
  uint16_T SVPTc;             /* SVPWM 输出W相 */
} CtrlHwlo_Bus;

/* 电机相关模拟量 */
typedef struct {
  real32_T handlebar_Volt;       /* 油门/转把电压-模拟量 */
  real32_T bus_Curr;       /* 母线电流 */
  real32_T bus_Curr_calc; /* 母线电流(计算值): CM_CalcBusCurrent 由 dq 反算 + 低通 */
  real32_T phaseU_Curr;    /* U相电流 */
  real32_T phaseV_Curr;    /* V相电流 */
  real32_T phaseW_Curr;    /* W相电流 */
  real32_T bus_Volt;       /* 母线电压 */
  real32_T bus_Volt_last;  /* 上一拍母线电压 */
  real32_T svpwm_factor;   /* svpwm 调制系数 */
  real32_T motor_temp;     /* 电机温度 */
  real32_T motor_temp_Rs;  /* 电机温度电阻值/电压值 */
  real32_T mos_temp;       /* MOS温度 */
  real32_T mos_temp_Rs;    /* MOS温度电阻值/电压值 */
  real32_T elec_power;     /* 电功率估算值 */
  uint32_T PWM_freq;       /* 载波频率 */
  uint16_T PWM_HalfPerMax; /* 定时器计数值的一半 */
  real32_T time_base;      /* 时基 10000/PWM_freq */
	real32_T Vd_buf;				 /* Vd 缓存 */
	real32_T Vq_buf;				 /* Vq 缓存 */
	real32_T Ld_est;				 /* 实时估算 Ld[H] */
	real32_T Lq_est;				 /* 实时估算 Lq[H] */
	uint8_T  LdLq_est_valid;		 /* 1=本拍估算有效 */
} Motor_Analog;

#endif                                 /* ADC_sample_h_ */

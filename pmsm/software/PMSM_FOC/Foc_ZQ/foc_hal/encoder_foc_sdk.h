/******************************************************************************
 * 文件: encoder_foc_sdk.h
 * 功能: TIM2 正交编码器 FOC 驱动接口 (含 Z 相检测 / 零位辨识 / PWM 绝对位置校零)
 * 创建日期: 2026-06-22
 * 修改日期: 2026-10-26
 * 修改内容: 运行期编码器线数和电机极对数接口; sensor_pole_pairs仅保留不使用
 * 修改人: rui.zhang
 ******************************************************************************/

/* 默认编码器: 1024线 × 4倍频 = 4096计数/机械圈; 电机极对数 4;
   硬件: PA0=TIM2_CH1, PA1=TIM2_CH2, PA2=Encoder_Z (GPIO输入) */

#ifndef __HW_ENCODER_FOC_SDK_H__
#define __HW_ENCODER_FOC_SDK_H__

#include "main.h"
#include "tim.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ********************* 配置参数 ********************* */
/* 以下为"默认值": 实际生效值走运行期参数 (DW.motor_param.enc_line_count /
   PP_num, CAN 可改), 由 EncFoc_ApplyConfig() 刷新到 g_enc_foc。
   sensor_pole_pairs 仅保留字段, 不参与编码器计算。 */
#define ENC_LINE_COUNT      1024            /* 默认线数 (主反馈分辨率) */
/* 当前中置电机编码器反向; 其他电机可改为0, 或由CAN A2/02 idx17覆盖 */
#define ENCODER_REVERSE_ENABLE 0U
#define ENC_PULSE_NUMBER    (ENC_LINE_COUNT * 4)   /* 默认机械每转计数 4096 */
#define MOTOR_POLE_PAIRS    4                       /* 默认电机极对数 */
#define CNT_TO_RPM_CONST    (float)(60000.0f/ENC_PULSE_NUMBER)   /* 仅默认值参考, 运行期用 PulseNumber */
#define ENC_DELTA_MAX   		1500  /* 1ms 最大合法计数增量: 15000rpm=1024cnt/ms,
                                  取1500(≈22000rpm)留46%裕量; 超出必为计数污染 */
/* 硬件: TIM2 CH1=PA0, CH2=PA1, Z=PA2 */
#define ENC_TIM             TIM2

/* Z相检测参数（磁编码器宽脉冲兼容） */
#define Z_DEBOUNCE_TICKS    5           /* Z 相消抖拍数 (0.5ms @10kHz) */
#define Z_LOCKOUT_COUNTS    500         /* 触发后锁定 500 counts (约45°机械) */
#define ZERO_IDF_FIXED_OFFSET_16        16384LL   /* 90°: 65536 × 90 / 360 */
/* ********************* Z相检测状态机 ********************* */
typedef enum {
    Z_IDLE = 0,       /* 空闲, 等待上升沿 */
    Z_WAIT_LOW,       /* 消抖: 持续高电平确认 */
    Z_WAIT_RISE,      /* 等待下降沿 (确认上升沿完成) */
    Z_LOCKOUT         /* 锁定: 忽略后续脉冲, 防止重复触发 */
} ZPhaseState_t;

/* ********************* 句柄定义 ********************* */
typedef struct {
    uint16_t     PulseNumber;        /* 每转计数 (4096) */
    uint8_t      bElToMecRatio;      /* 极对数 */
    uint8_t      RevertSignal;
    uint16_t     PulseMask;          /* 2 的幂掩码 (PulseNumber-1) */
    uint8_t      PulseShift;         /* 角度左移位数; 0xFF = 非 2 的幂, 走除法 */
    float        SpdCoeffF;         /* 速度换算系数 60000/(16*每转计数), 配置变化时预算(免每拍除法) */
    float        MecDegPerCntF;     /* 机械角换算 360/每转计数(度/计数), 配置变化时预算 */

    volatile int32_t  raw_count;
    volatile uint16_t last_cnt_u16;

    int16_t  hMecAngle;
    int16_t  hElAngle;

    int32_t  DeltaBuffer[16];
    uint8_t  DeltaIndex;
    int16_t  hAvrMecSpeed;          /* 16拍平均的整数计数/ms (调试观察, 速度计算不用它) */
    uint16_t PrevCapture;
    float    hAvrSpeedRPM;

    /* ---- Z相检测 (ZPhase_Process 状态机) ---- */
    ZPhaseState_t z_phase_state;   /* Z相状态机当前状态 */
    uint8_t       z_first_state;   /* 首次找到Z相标志 (0=未找到 1=已找到) */
    uint32_t      z_lockout_cnt;   /* Z相锁定计数基准 */
    uint8_t       z_clear;         /* Z相清零使能 (1=允许清零CNT) */

    /* ---- 零位校准结果 ---- */
    uint8_t       zero_state;      /* 0=未校准 1=已校准 */
    int32_t       zero_offset;     /* 编码器零位偏移 (16bit电角度域 = ABZ cnt×65536/1000) */

    /* ---- PWM 零位基准 (校零时学习, 存 EEPROM; 上电快速恢复用) ---- */
    uint16_t      pwm_zero_ref;    /* 零位点 PWM raw 基准值 (2*PP点mod(4096/PP)圆周平均) */
    uint8_t       pwm_ref_valid;   /* 1=pwm_zero_ref 有效 (EEPROM 校验通过) */
    uint8_t       pwm_dir;         /* PWM 计数相对电角度方向: 0=同向 1=反向 (校零自学习) */
    uint16_t      zero_hyst_cnt;   /* 双向校零迟滞 (ABZ cnt, 质量监控用) */
} EncoderHandle_t;

extern EncoderHandle_t g_enc_foc;


/* 零位基准保存请求: 找零成功时置1, 由 SignalProc_PwmZeroCalib(1ms任务) 处理 */
extern volatile uint8_t g_zero_ref_save_pending;

/* ********************* API ********************* */
void  ENC_Init(void);
void  ENC_CalcAngle(void);              /* 角度计算 (10kHz), 含 zero_offset 校正 */
/* 应用运行期配置: 线数/极对数 (CAN 可改) -> 刷新 PulseNumber/ElToMecRatio/掩码 */
void  EncFoc_ApplyConfig(void);
uint16_t EncFoc_DeltaMax(void);         /* 每拍最大合法计数增量 (随线数) */
uint8_t EncFoc_LinePulse(void);         /* A/B 线边沿活动(读即清): bit0=A bit1=B */
/* 手动零位基准: 用 CAN 下发的 pwm_zero_ref + 当前静止 PWM raw 反算 zero_offset。
   前置: 必须已标零成功过(pwm_ref_valid), 否则无法定方向; 返回 1=已应用(变了)。 */
uint16_t EncFoc_PwmDomain(void);        /* PWM 原始值每极对刻度 = 4096/极对数 */
uint8_t ENC_ManualZeroRef_Apply(uint16_t raw_ref, uint16_t raw_now);
void  ENC_CalcSpeed(void);              /* 速度计算 (1kHz) */

/* Z相检测: 10kHz调用, 消抖状态机检测 Z 相上升沿并清零 CNT */
void  ZPhase_Process(void);

/* 编码器零位辨识: 10kHz调用, ZERO_IDF_ALGORITHM选择IF双向扫点或Vq扫描 */
void  ENC_ZeroIdf_Task(void);
void  ENC_ZeroIdf_Reset(void);            /* 校零状态复位 */

/* 上电快速恢复: 由静止时 PWM raw 均值反推 zero_offset 并写句柄, 返回1=已应用 */
uint8_t ENC_ZeroOffset_FromPwmRaw(uint16_t raw_avg);

float ENC_GetMecAngleDeg(void);

#endif /* __HW_ENCODER_FOC_SDK_H__ */

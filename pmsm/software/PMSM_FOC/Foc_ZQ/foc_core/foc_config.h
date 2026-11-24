/******************************************************************************
 * 文件: foc_config.h
 * 功能: FOC 全局配置 (F303RE 72MHz: 数学常量/PWM/ADC/标定/电机参数/控制模式)
 * 创建日期: 2026-06-22
 * 修改日期: 2026-06-22
 * 修改内容: 建立 FOC 全局配置
 * 修改人: rui.zhang
 ******************************************************************************/

#ifndef FOC_CONFIG_H_
#define FOC_CONFIG_H_

#define Hardware_Type       1         /* 1: STM32 */
#define Hardware_version    1         /* 1: F303 hardware */
#define chiji_FR            0         /* 磁极反向: 0=正, 1=反 */

#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

/* ********************* 数学常量 ********************* */
#define My_PI           3.14159265358979323846f
#define My_PI_2         1.57079632679489661923f
#define My_SQRT3        1.73205080756887f
#define My_SQRT3_2     0.86602540378443f
#define My_SQRT3_3     0.57735026918962f
#define My_div2         0.5f

/* ********************* 系统频率 (F303RE 72MHz) ********************* */
#define SYSTEM_FREQUENCY    72
#define CPU_Freq            (72000000)
#define Delay_1Bns          (CPU_Freq/10000000+5)
#define Delay_1us            (CPU_Freq/1000000+5)
#define Delay_1ms            (CPU_Freq/1000000*1000+5)

/* ********************* PWM 参数 (10kHz 中心对齐) ********************* */     /* 10kHz */
#define PWM_PeriodMax_def        ((uint16_t)(3600))     /* ARR+1 = 3600 @72MHz */
#define PWM_HalfPerMax_def       ((uint16_t)(PWM_PeriodMax_def*0.5))  /* 1800 */
#define PWM_HalfPerMax_1     			((1.0f/(float)PWM_PeriodMax_def))
#define PWM_Deadband         			((uint16_t)(1*SYSTEM_FREQUENCY))  /* 72 ticks ≈ 1us */
#define PWM_DeadbandMin      			((uint16_t)(2*SYSTEM_FREQUENCY))  /* 144 ticks ≈ 2us */

/* ********************* 电机参数 ********************* */
#define Protect_Low_Voltage_V       100.0f
#define Standard_Work_Voltage_V     24.0f
#define Protect_High_Voltage_V      580.0f


#define Sqrt_OF_3           0.57735f
#define Svpwm_Km_def            (Standard_Work_Voltage_V*Sqrt_OF_3)
#define Vdc_s               (Standard_Work_Voltage_V*Sqrt_OF_3*0.96f)
#define PI_MAX_Ud           (Standard_Work_Voltage_V*Sqrt_OF_3*0.9f)
#define PI_MAX_Uq           Vdc_s

/* ********************* 控制模式 ********************* */
/* 注: Speed/Idq/Torq/VF/HighFre/FD 已由 model/MotorCtr_parameter.h 统一定义,
 *     此处仅保留 MotorCtr_parameter.h 未定义的扩展模式 (Stop/Power/Position/IF)。 */
#ifndef Control_Mode_Power
#define Control_Mode_Power      3   /**< 功率控制模式 (预留) */
#endif
#ifndef Control_Mode_Position
#define Control_Mode_Position   5   /**< 位置控制模式 (预留) */
#endif
#ifndef Control_Mode_IF
#define Control_Mode_IF         7   /**< IF 电流/频率控制模式 (预留) */
#endif


/* GPIO 类型 (STM32 HAL 兼容) */
typedef struct {
    GPIO_TypeDef  *GPIOx;
    uint16_t       gpio_num;
    char           direction;
    const char    *src;
} GPIO_TYPE;
/* 零位辨识状态宏 */
#define ST_IDF_ZERO_IDLE  			 0U    /* 校零空闲 */
#define ST_IDF_ZERO_INIT  			 1U    /* 零位辨识初始化 (同 ST_IDF_ZERO) */
#define ST_IDF_ZERO_RUN    			 2U    /* 辨识零位中*/
#define ST_IDF_ZERO_WAIT   			 10U   /* 零位辨识后等待 */
#define ST_IDF_ZERO_TESTs  			 11U   /* 旋转验证-正转 */
#define ST_IDF_ZERO_TESTt  			 12U   /* 旋转验证-停止 */
#define ST_IDF_ZERO_TESTn  			 13U   /* 旋转验证-反转 */
#define ST_IDF_ZERO_CHECK 			 14U   /* 旋转验证后等待 */
#define ST_IDF_ZERO_OK     			 100U  /* 零位辨识成功 */
#define ST_IDF_ZERO_FIRL    		 101U  /* 零位辨识失败 */

/* 零位搜索算法: 0=IF双向扫点, 1=Idq扫描平均Vq最大值 */
#define ZERO_IDF_ALG_IF_SWEEP       0
#define ZERO_IDF_ALG_VQ_SCAN        1
#define ZERO_IDF_ALGORITHM          ZERO_IDF_ALG_IF_SWEEP
#if ((ZERO_IDF_ALGORITHM != ZERO_IDF_ALG_IF_SWEEP) && \
     (ZERO_IDF_ALGORITHM != ZERO_IDF_ALG_VQ_SCAN))
#error "ZERO_IDF_ALGORITHM must be ZERO_IDF_ALG_IF_SWEEP or ZERO_IDF_ALG_VQ_SCAN"
#endif

/* Vq扫描参数(10kHz): 4000点, 每点60拍, 后40拍平均, 结果回退20点 */
#define ZERO_VQ_SCAN_POINTS         4000U
#define ZERO_VQ_SCAN_PERIOD         60U
#define ZERO_VQ_SCAN_SKIP           20U
#define ZERO_VQ_SCAN_BACKOFF        20U
#define ZERO_VQ_SCAN_BUS_THRESHOLD  100.0F
#define ZERO_VQ_SCAN_VD_MAX_LOW     0.15F
#define ZERO_VQ_SCAN_VD_MAX_HIGH    0.28F

/* 电机参数辨识状态宏 */
#define ST_IDF_PARAM_IDLE        0U    /* 参数辨识空闲 */
#define ST_IDF_PARAM_INIT        1U    /* 参数辨识初始化 */
#define ST_IDF_PARAM_RES         2U    /* 定子电阻辨识中 */
#define ST_IDF_PARAM_LdLq        3U    /* 交直轴电感辨识中*/
#define ST_IDF_PARAM_RES_WAIT    10U   /* 电阻辨识后等待 */
#define ST_IDF_PARAM_LdLq_SET    31U   /* 电感辨识-拉到0位 */
#define ST_IDF_PARAM_OK      		 100U  /* 参数辨识成功 */
#define ST_IDF_PARAM_FIRL    		 101U  /* 参数辨识失败 */
typedef struct {
    uint8_t  Identify_STA;
    uint8_t  Identify_STA_step;
    uint16_t Best_Angle_CNT;
    uint16_t Average_CNT;
    float    JILU_Vd;
    float    JILU_Vq;
    float    I_q_cmd;
    float    Vq_max;
    float    Vd_max;
    uint16_t Vq_max_index;
    uint16_t Vd_max_index;
    float    Vq_min;
    float    Vd_min;
    uint16_t Vq_min_index;
    uint16_t Vd_min_index;
    float    Is_dat;
    float    sum_dat;
    float    I_CurQsP;
    float    I_CurQsN;
    float    I_CurQsP_sum;
    float    I_CurQsN_sum;
    float    Ld_sum;
    float    Lq_sum;
    uint16_t L_cnt;
    uint16_t zhong_cnt;
    uint16_t zero_sta;

    /* ---- Z相校零字段 (移植自参考工程) ---- */
		uint16_t Angle_delta;      /* 角度步进增量 */
    uint16_t Angle_buff;       /* 角度缓冲 (0-4095, 12bit电角度) */
    uint16_t PP_cnt;           /* 极对数计数 */
    uint16_t Z_flag;           /* Z相状态: 0=未找到 1=找到 2=确认 */
#define ZERO_IDF_MAX_PP        16U
    int32_t  TIM_cnt_buf[2U * ZERO_IDF_MAX_PP];   /* ABZ: 前PP项正扫, 后PP项反扫 */
    int32_t  PWM_cnt_buf[2U * ZERO_IDF_MAX_PP];   /* PWM raw: 前PP项正扫, 后PP项反扫 */
    uint8_t  sweep_dir;        /* 扫描方向: 0=正扫 1=反扫 */
    uint8_t  qual_fail;        /* 合成质量检查失败标志 (迟滞/散布超限) */
} Best_Angle_Type;

#define Best_Angle_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}

extern Best_Angle_Type BestA_u;

#endif /* FOC_CONFIG_H_ */

/*
 * File: MotorCtr_parameter.h  (MATLAB 1:1 镜像)
 *
 * Code generated for Simulink model 'MCU_MotorContrl_2024b'.
 * Model version      : 1.1974
 * Simulink Coder     : 24.2 (R2024b)
 *
 * 注意: 此处 Control_Mode_* 取值与工程 foc_config.h 不同(MATLAB: Torq=3/VF=5/FD=6)。
 *       仅 foc_ctrl.c 引用本头(不与 foc_config.h 同时 include), 避免宏重定义。
 */

#ifndef MotorCtr_parameter_h_
#define MotorCtr_parameter_h_
#include "rtwtypes.h"

#define HARDWARE_VERSION 							((uint32_t)20U) /* 硬件版本号 */
#define SOFTWARE_VERSION 							((uint32_t)32U) /* 软件版本号 */
#define SYS_CLK_FREQ									 72000000			  /* 系统时钟频率*/		
#define PWM_FREQ										 	 10000			  	/* PWM频率*/		
#define OSCILLOSCOPE_LEN_MAX					 1024						/* 示波器 单通道 最大长度 */
#define OSCILLOSCOPE_CHANNEL_MAX			 2							/* 示波器最大通道数       */
/* Exported data define (custom storage class: Define) */
#define CONVERT_ADC_CONST							 0.000805664062f/* ADC转换系数 */
#define ADC_ZERO_TARGET                1000U          /* ADC模拟量零位校准目标 */
#define BRAKE_CNT_TARGET               50U            /* 刹车GPIO滤波计数目标 ms */
#define CONTINUATE_CNT_TARGET          300.0          /* 持续计数目标 ms */
#define CONVERT_BUS_CURR_CONST         1.0F           /* 母线电流转换系数 */
#define CONVERT_BUS_VOLT_CONST         0.025083125F   /* 母线电压转换系数 */
#define CONVERT_BUS_VOLT_OFFSET        6.5625f   			/* 母线电压偏移值-************add**** */
#define CONVERT_HANDLEBAR_CONST        (CONVERT_ADC_CONST/0.6667f)           /* 转把(ACC)电压转换系数 */
#define CONVERT_MOS_RS_CONST           0.00073242F    /* MOS温度采样电阻转换系数 */
#define CONVERT_MOTOR_RS_CONST         0.732421875F   /* 电机温度采样电阻转换系数 */
/* 相电流霍尔选型: 600A=0.0033V/A, 800A=0.0025V/A (同一 ADC 分压前端, 只换传感器)
   改 HALL_CURR_SENSOR_SEL 后重新编译; A/count = 0.362548828125 × 0.0033 / 灵敏度 */
#define HALL_CURR_SENSOR_600A          0
#define HALL_CURR_SENSOR_800A          1
#define HALL_CURR_SENSOR_SEL           HALL_CURR_SENSOR_800A
#if (HALL_CURR_SENSOR_SEL == HALL_CURR_SENSOR_600A)
#define CONVERT_PHASE_CURR_CONST       0.362548828125F   /* 600A: 0.0033V/A */
#elif (HALL_CURR_SENSOR_SEL == HALL_CURR_SENSOR_800A)
#define CONVERT_PHASE_CURR_CONST       0.478564453125F   /* 800A: 0.0025V/A, 同分压 */
#else
#error "HALL_CURR_SENSOR_SEL: use HALL_CURR_SENSOR_600A or HALL_CURR_SENSOR_800A"
#define CONVERT_PHASE_CURR_CONST       0.362548828125F   /* 600A: 0.0033V/A */
#endif

#define Control_Mode_Stop       			 0U   					/* 停止模式     */
#define Control_Mode_Speed             1U             /* 速度控制模式 */
#define Control_Mode_Idq               2U             /* 电流控制模式 */
#define Control_Mode_Torq              3U             /* 扭矩控制模式 */
#define Control_Mode_Idq_Tset          4U             /* 电流测试模式 */
#define Control_Mode_VF                5U             /* VF控制模式   */
#define Control_Mode_FD                6U             /* 防盗模式 */
#define Control_Mode_HighFre           8U             /* 高频注入辨识模式 */

#define GPIO_ACTIVE_LEVEL              0U             /* GPIO有效电平 */
#define GPIO_CNT_TARGET                10U            /* GPIO滤波计数目标 ms */
#define KEY_ACTIVE_LEVEL               1U             /* 按键有效值 */
#define LONG_CNT_TARGET                2000U          /* 长按计数目标 ms */
#define MATH_1_SQRT3                   0.577350259F   /* 数学常量 1/sqrt(3) */
#define PWM_TIMER_PERIOD               ((SYS_CLK_FREQ/PWM_FREQ)/2)  											/* PWM定时器计数值 (= TIM1 ARR, 中心对齐 10kHz @72MHz) */
/* 占空比上下限: PWM_DUTY_MARGIN_PCT = 最大占空比 0.985 -> 1.5%~98.5% */
#ifndef PWM_DUTY_MARGIN_PCT
#define PWM_DUTY_MARGIN_PCT            0.99f
#endif
#define PWM_DUTY_MIN_LIMIT             (uint16_t)(PWM_TIMER_PERIOD * (1.0f - PWM_DUTY_MARGIN_PCT))
#define PWM_DUTY_MAX_LIMIT             (uint16_t)(PWM_TIMER_PERIOD * PWM_DUTY_MARGIN_PCT)
#define PWM_DEAD_TIME_LIMIT            (uint16_t)(136U)	  //死区2.0e-6f * SYS_CLK_FREQ 136U
#define RATED_VOLTAGE_DEFAULT          72U            /* 默认额定电压 */
#define SHORT_CNT_TARGET               1000U          /* 短按计数目标 ms */
#define STUCK_CNT_TARGET               5000.0         /* 卡死按键持续时间定义 */

/* ********************* 状态寄存器位定义 (来源: document/寄存器表.xlsx) ********************* */
/* ---- B07-B00: 设备/系统状态 ---- */
#define STATUS_DEVICE_POWER_BIT          (1U << 0)       /* B00: 设备开机 0:待机 1:开机 */
#define STATUS_ENABLE_BIT                (1U << 1)       /* B01: 使能(控制器运行) 0:未使能 1:使能 */
#define STATUS_CHARGE_BIT                (1U << 2)       /* B02: 充电状态 0:未充电 1:充电 */
#define STATUS_PRECHARGE_BIT             (1U << 3)       /* B03: 预充状态 0:未预充 1:已预充 */
#define STATUS_SYSTEM_STATE_POS          4U              /* B07-B04: 系统状态 (4bit: 0初始化/1自检/2P档/3Ready/4运行/5故障/6清零/7自学/8下电/9功能安全) */
#define STATUS_SYSTEM_STATE_MASK         (0xFU << STATUS_SYSTEM_STATE_POS)

/* ---- B15-B08: 控制/驾驶状态 ---- */
#define STATUS_CONTROL_MODE_POS          7U              /* B08-B07: 控制模式 (2bit: 0停机/1速度闭环/2扭矩闭环/3开环) */
#define STATUS_CONTROL_MODE_MASK         (3U << STATUS_CONTROL_MODE_POS)
#define STATUS_DRIVE_MODE_POS            9U              /* B10-B09: 驾驶模式 (2bit: 0ECO滑行/1NOR巡航/2SPORT助力) */
#define STATUS_DRIVE_MODE_MASK           (3U << STATUS_DRIVE_MODE_POS)
#define STATUS_BRAKE_STATE_POS           11U             /* B12-B11: 刹车状态 (2bit: 0未刹/1刹车/2驻车) */
#define STATUS_BRAKE_STATE_MASK          (3U << STATUS_BRAKE_STATE_POS)
#define STATUS_ENERGY_RECOVERY_POS       13U             /* B14-B13: 能量回收 (2bit: 0关/1轻/2中/3重) */
#define STATUS_ENERGY_RECOVERY_MASK      (3U << STATUS_ENERGY_RECOVERY_POS)

/* ---- B23-B16: 功能状态 ---- */
#define STATUS_DRIVE_FUNC_STATE_POS      14U             /* B18-B14: 行驶/功能状态 (5bit: 0空闲/1有效触发/2CCS/3HDC/4助推/5应急/6HCC坡道驻车/7制动/8推车) */
#define STATUS_DRIVE_FUNC_STATE_MASK     (0x1FU << STATUS_DRIVE_FUNC_STATE_POS)
#define STATUS_CC_ENABLE_BIT             (1U << 20)      /* B20: 定速巡航功能 0:关闭 1:开启 */
#define STATUS_HDC_ENABLE_BIT            (1U << 21)      /* B21: 陡坡缓降功能 0:关闭 1:开启 */
#define STATUS_TCS_ENABLE_BIT            (1U << 22)      /* B22: TCS牵引力控制 0:关闭 1:开启 */
#define STATUS_HILL_PARK_BIT             (1U << 23)      /* B23: 陡坡驻车功能 0:关闭 1:开启 */

/* ---- B31-B24: 钥匙/安全状态 ---- */
#define STATUS_EMERGENCY_ENABLE_BIT       (1U << 24)     /* B24: 应急骑行 0:关闭 1:开启 */
#define STATUS_TCS_ACTIVE_BIT             (1U << 25)     /* B25: TCS触发状态 0:未触发 1:触发 */
#define STATUS_SIDESTAND_BIT              (1U << 26)     /* B26: 边撑状态 0:放下 1:收起 */
#define STATUS_KEY_POS_POS                28U            /* B31-B28: 钥匙位置 (4bit: 0停止/1P档/2Ready/3前进/4倒车/5刹车/6驻车) */
#define STATUS_KEY_POS_MASK               (0xFU << STATUS_KEY_POS_POS)

/* ********************* IO状态寄存器位定义 (原始GPIO) ********************* */
#define IO_REG_RAW_BRAKE_BIT             (1U << 0)       /* B00: 原始IO-刹车 0:低 1:高 */
#define IO_REG_RAW_SIDESTAND_BIT         (1U << 1)       /* B01: 原始IO-边撑 */
#define IO_REG_RAW_PUSH_BIT              (1U << 2)       /* B02: 原始IO-推车 */
#define IO_REG_RAW_REVERSE_BIT           (1U << 3)       /* B03: 原始IO-倒车 */
#define IO_REG_RAW_GEAR_PLUS_BIT         (1U << 4)       /* B04: 原始IO-档位+ */
#define IO_REG_RAW_GEAR_MINUS_BIT        (1U << 5)       /* B05: 原始IO-档位- */
#define IO_REG_RAW_P_GEAR_BIT            (1U << 6)       /* B06: 原始IO-P档 */
#define IO_REG_RAW_CCS_BIT               (1U << 7)       /* B07: 原始IO-巡航 */
#define IO_REG_RAW_ACC_BIT               (1U << 8)       /* B08: 原始IO-ACC(钥匙) */
#define IO_REG_FUNC_BTN_CCS              (1U << 11)      /* B11: 功能按钮状态-定速巡航 */

/* ********************* 故障寄存器位定义 (document/寄存器表.xlsx "故障寄存器表") ********************* */
//Byte[0:7]
#define FAULT_BIT_DRIVER_HIGH_TEMP   (1U << 0)   // B00: 驱动器温度高
#define FAULT_BIT_MOTOR_HIGH_TEMP    (1U << 1)   // B01: 电机温度高
#define FAULT_BIT_BUS_OFFSET         (1U << 2)   // B02: 直流母线电流零漂移过大(备用)
#define FAULT_BIT_PHASE_OFFSET       (1U << 3)   // B03: 相电流零漂移过大
#define FAULT_BIT_DRIVER_TEMP_SENSOR (1U << 4)   // B04: 驱动器温度采样电路异常
#define FAULT_BIT_MOTOR_TEMP_SENSOR  (1U << 5)   // B05: 电机温度采样电路异常
#define FAULT_BIT_MCU_MEMORY         (1U << 6)   // B06: MCU的存储器异常
#define FAULT_BIT_STALL              (1U << 7)   // B07: 堵转
//Byte[8:15]
#define FAULT_BIT_BUS_OVERVOLTAGE    (1U << 8)   // B10: 直流母线过压
#define FAULT_BIT_BUS_UNDERVOLTAGE   (1U << 9)   // B11: 直流母线欠压
#define FAULT_BIT_PHASE_LOSE         (1U << 10)  // B12: 电机缺相
#define FAULT_BIT_OVERSPEED          (1U << 11)  // B13: 超速
#define FAULT_BIT_MOTOR_FEEDBACK_LOSS (1U << 12) // B14: 电机主反馈丢失
#define FAULT_BIT_DRIVER_OVERTEMP    (1U << 13)  // B15: 驱动器过温(二级)
#define FAULT_BIT_MOTOR_OVERTEMP     (1U << 14)  // B16: 电机过温(二级)
#define FAULT_BIT_THROTTLE_FAULT     (1U << 15)  // B17: 油门转把故障
//Byte[16:23]
#define FAULT_BIT_OVERCURRENT        (1U << 16)  // B20: 过流故障(软件级)
#define FAULT_BIT_PHASE_SHORT        (1U << 17)  // B21: 电机相限短路(硬件级)
#define FAULT_BIT_MCU_ROM_FAULT      (1U << 18)  // B22: MCU的ROM故障(功能已移除, 保留位定义)
#define FAULT_BIT_MCU_RAM_FAULT      (1U << 19)  // B23: MCU的RAM故障(功能已移除, 保留位定义)
#define FAULT_BIT_CPU_STACK_OVERFLOW (1U << 20)  // B24: CPU堆栈溢出
#define FAULT_DRIVER_LEVEL1					 (1U << 21)  // B25/B26 驱动器故障等级1 轻微
#define FAULT_DRIVER_LEVEL2					 (2U << 21)  // B25/B26 驱动器故障等级2 严重
#define FAULT_DRIVER_LEVEL3					 (3U << 21)  // B25/B26 驱动器故障等级3 致命
#endif                                 /* MotorCtr_parameter_h_ */

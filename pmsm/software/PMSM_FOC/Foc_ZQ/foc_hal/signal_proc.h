/******************************************************************************
 * 文件: signal_proc.h
 * 功能: GPIO 输入识别/去抖 + 模拟量转换/滤波 接口
 * 创建日期: 2026-06-22
 * 修改日期: 2026-07-17
 * 修改内容: 按图示规格整改按键/档位识别 (档位方向修正/卡滞瞬时/三击/等待窗口输出0); 输出码宏不变
 * 修改人: rui.zhang
 ******************************************************************************/

/* I/O: 读写 DW.ctrl_hw_io 的 gpio 输入与 _o 输出; DW.adc_slmple -> DW.motor_anal。
 * 调用前需把 HAL GPIO 电平填入 DW.ctrl_hw_io.*_gpio, ADC 原始值填入 DW.adc_slmple。 */

#ifndef SIGNAL_PROC_H_
#define SIGNAL_PROC_H_

/* ******************** 按键输出码 (对齐接口文档, 供外部引用) ******************** */
#define KEY_OUT_NONE       0U      /* 未按下 / 无效 */
#define KEY_OUT_HOLD       1U      /* 按下中 */
#define KEY_OUT_SINGLE     2U      /* 单击(按下后松开) */
#define KEY_OUT_PRESS_2ND  3U      /* 第二下按下 */
#define KEY_OUT_DOUBLE     4U      /* 双击(按下后松开) */
#define KEY_OUT_LONG       5U      /* 长按 */
#define KEY_OUT_PRESS_3RD  6U      /* 第三下按下 */
#define KEY_OUT_STUCK      255U    /* 卡滞 */

/* 档位输出码 */
#define GEAR_OUT_ECO       0U      /* 低速档 */
#define GEAR_OUT_NOR       1U      /* 中速档 */
#define GEAR_OUT_SPORT     2U      /* 高速档 */
#define GEAR_OUT_STUCK_DN  254U    /* 下键卡滞 */
#define GEAR_OUT_STUCK_UP  255U    /* 上键卡滞 */

/* 初始化 GPIO 识别 + 滤波内部状态 (置零, 状态机入初始态) */
void SignalProc_Init(void);

/* GPIO 输入识别/去抖 (ms 级调用): brake/side/gear/boost/P/R/CCS/burglar/ACC */
void SignalProc_GpioIdentify(void);

/* 模拟量转换 + 滤波: ADC 零位校准, (raw-offset)*系数, 一阶 LPF */
void SignalProc_AnalogConvert(void);
/* PWM 零位校准 (上电快速恢复): 调度器 1ms 调用, 完成/放弃后自动空闲 */
void    SignalProc_PwmZeroCalib(void);
/* 重新触发零位校准 (任意状态复位到等待条件态) */
void    SignalProc_PwmZeroCalib_Start(void);
/* 查询零位校准是否完成: 1=完成且零位有效 */
uint8_t SignalProc_IsZeroCalibDone(void);
/* 查询零位校准状态: 0=等待条件 1=采集中 2=完成 3=放弃 */
uint8_t SignalProc_GetZeroCalibState(void);
#endif /* SIGNAL_PROC_H_ */

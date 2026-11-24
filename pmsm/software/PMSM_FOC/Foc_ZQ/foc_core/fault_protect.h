/******************************************************************************
 * 文件: fault_protect.h
 * 功能: 故障检测/保护接口 (过流/短路/过欠压/超速/过温/堵转/转把/自检)
 * 创建日期: 2026-06-22
 * 修改日期: 2026-06-22
 * 修改内容: 新建
 * 修改人: rui.zhang
 ******************************************************************************/

/* Fault_FastTask 在10kHz ISR调用; Fault_Protect_Task 在主循环1kHz调用。
 * 输入: DW.motor_anal / motor_main_feedback.mech_Spd / adc_slmple / faule_protection
 * 输出: DW.comm_contrl.fault_state / fault_power_down_state, faule_protection.LR_state_o */

#ifndef FAULT_PROTECT_H_
#define FAULT_PROTECT_H_
#include "main.h"

/* 下电类故障位掩码(堵转按 LR_mode 单独判断) */
#define FAULT_PD_MASK  (FAULT_BIT_OVERCURRENT      | \
                        FAULT_BIT_PHASE_SHORT      | \
                        FAULT_BIT_BUS_OVERVOLTAGE  | \
                        FAULT_BIT_BUS_UNDERVOLTAGE | \
                        FAULT_BIT_OVERSPEED        | \
                        FAULT_BIT_PHASE_LOSE       | \
                        FAULT_BIT_MOTOR_FEEDBACK_LOSS | \
                        FAULT_BIT_MOTOR_OVERTEMP   | \
                        FAULT_BIT_DRIVER_OVERTEMP  | \
                        FAULT_BIT_CPU_STACK_OVERFLOW)
												

extern uint8_t volatile g_spd_updated; 												 
/* 10kHz ISR: A/B线断、过流、短路快速保护 */
void Fault_FastTask(void);
/* 主循环1kHz: 其余故障检测与状态管理 */
void Fault_Protect_Task(void);
/* 上电/清故障自检: 1200ms窗口按HAL_GetTick计时, 不依赖主循环执行次数 */
void Fault_SelfCheck_Start(void);


/* 固件完整性校验请求: 自检开始时置位, 主循环取走后返回 1(取走即清零) */
uint8_t Fault_FwCheckReqTake(void);

/* CPU 堆栈溢出哨兵: Init 在上电早期调用(栈底写入魔数), Check 返回 1=哨兵已被覆盖 */
void    Stack_Guard_Init(void);
uint8_t Stack_Guard_Check(void);

/* 功率限制(降额): 根据温度/故障状态计算最大 Iq 限值, 用于截断 I_q_target */

#endif /* FAULT_PROTECT_H_ */

/******************************************************************************
 * 文件: foc_data.h
 * 功能: FOC 运行数据块 FOC_Ctrl_Data (MATLAB DW_T 的 FOC 相关子集) 定义
 * 创建日期: 2026-06-22
 * 修改日期: 2026-06-26
 * 修改内容: 新增 CAN 协议映射所需子结构(PI总线/系统/通讯/上传/整车/状态机)
 * 修改人: rui.zhang
 ******************************************************************************/

/* 通过 model/ 前缀引用 MATLAB 结构体头, 避免与工程同名 Host_Contrl.h 冲突。
 * 仅 foc_ctrl.c / foc_pi.c / 故障与 ISR 文件引用本头, 不与 foc_config.h 同一编译单元,
 * 以规避 Motor_Analog.PWM_HalfPerMax 字段名与 foc_config.h 同名宏的冲突。 */

#ifndef FOC_DATA_H_
#define FOC_DATA_H_
#include "MotorCtr_parameter.h"
#include "PI_revise.h"
#include "Host_Contrl.h"
#include "Main_Feedback.h"
#include "ADC_sample.h"
#include "Comm_Math.h"
#include "vehicle_info.h"     /* Vehicle_Info (CAN 协议 A5/A6) */
#include "State_Machine.h"    /* Function_State_Machine / Key_State_Machine (CAN 协议 A6) */

/* FOC 运行数据块 (MATLAB DW_T 子集) */
typedef struct {
  Fault_Protection     faule_protection;     /* 故障保护参数 */
  Comm_Contrl_Type     comm_contrl;          /* 通讯综合控制 */
  Motor_Analog         motor_anal;           /* 电机模拟量 */
  Motor_Parameter      motor_param;          /* 电机参数 */
  PI_Params            pi_position;          /* 位置环路 */
  PI_Params            pi_voltage;           /* 电压环路 (弱磁) */
  PI_Params            pi_ds;                /* 电流环路 Ds */
  PI_Params            pi_qs;                /* 电流环路 Qs */
  PI_Params            pi_speed;             /* 速度环 PI */
  Motor_Main_Feedback  motor_main_feedback;  /* 电机主反馈 */
  ADC_Sample           adc_slmple;           /* ADC 转换结果 */
  CtrlHwlo_Bus         ctrl_hw_io;           /* 硬件对接 (SVPWM 输出) */
  Slope_Calculate      slope_calc_torque;    /* 扭矩控制斜坡 */
  Slope_Calculate      slope_calc_spd;       /* 速度控制斜坡 */

  /* ---- CAN 协议映射子结构 (供 canopen 直接读写, 对应协议接口.xlsx) ---- */
  PI_Parameter_Bus       comm_pi_bus;          /* A1: PI 参数总线 */
  SYS_Parameter          sys_param;            /* A2/A4: 系统级参数 */
  Comm_Parameter         comm_param;           /* A4: 通讯端口参数 */
  Up_Data_Parameter      up_data_param;        /* A5: 上传数据 */
  Vehicle_Info           vehicle_info;         /* A5/A6: 整车信息 */
  Function_State_Machine fun_state_machine;    /* A6: 功能状态机 */
  Key_State_Machine      key_state_machine;    /* A6: 关键状态机 */

  real32_T             Angle_buf;            /* <S31>/Delay : VF 角度累积 (持久) */
  uint8_T              vd_err_cnt;           /* <S59>/Vol_Circle_Limit 计数 (持久) */

  /* ---- 故障保护 fault_protection_mod 的持久计数器 (移植自 MATLAB DW_T) ---- */
  real32_T             handlebar_cnt;        /* 转把电压异常计时 */
  real32_T             LR_Cnt;               /* 堵转计时 */
  real_T               circuilt_cnt[3];      /* 故障去抖计数: 0=自检电流漂移 1=过流 2=超速  */
  real_T               mos_temp_cnt[2];      /* MOS温度: 0=过温 1=高温 计时 */
  real_T               motor_temp_cnt[2];    /* 电机温度: 0=过温 1=高温 计时 */
  real_T               U_Curr_last;          /* 自检: 上一拍 U 相电流 A */
  real_T               V_Curr_last;          /* 自检: 上一拍 V 相电流 A */
  uint16_T             check_Cnt;            /* 自检节拍计数 */
	
	/* 示波器功能--CanOpen */
	int32_T oscilloscope_buf[OSCILLOSCOPE_CHANNEL_MAX][OSCILLOSCOPE_LEN_MAX];
	uint32_T oscilloscope_cnt;
	uint32_T can_send_cnt;
	uint8_T oscilloscope_state;
	uint8_T sys_init_state;
} FOC_Ctrl_Data;

/* 说明: MATLAB DW_T 中的 Merge_e/Merge_l/Merge3_e/Merge3_f/Merge2_n/Merge1_pb/Merge_o
 *       为 Simulink Merge 块的临时信号, 每个控制周期内"先写后读", 无需常驻 RAM。
 *       已优化为 FOC_Ctrl_Task() 内的局部变量(寄存器/栈), 省 RAM、提速。
 *       唯一需跨周期保持的速度斜坡梯度, 用 foc_ctrl.c 内 file-static 保存。 */

extern FOC_Ctrl_Data DW;

#endif /* FOC_DATA_H_ */

/******************************************************************************
 * 文件: foc_globals.c
 * 功能: FOC 全局变量集中定义 (其他文件 extern 引用)
 * 创建日期: 2026-06-22
 * 修改日期: 2026-10-26
 * 修改内容: 母线反算使用同坐标上一拍施加电压; 快速反馈保持10kHz,
 *           普通显示降为1kHz并停用示波器诊断采集
 * 修改人: rui.zhang
 ******************************************************************************/
#include "foc_config.h"
#include "foc_data.h"

#include "canopen.h"
#include "CanOpen_TPDO.h"

#include "MotorCtr_parameter.h" /* Control_Mode_* 枚举 (以 MATLAB 生成为准) */
#include "encoder_pwm_u.h"
#include "encoder_foc_sdk.h"
#include "foc_ctrl.h"
#include "param_store.h"
#include "Common_Math.h"   /* CM_CalcBusCurrent: dq 反算母线电流 */
//#include "Parameter_Idf.h"
/* ******************** 参数识别 ******************** */
Best_Angle_Type BestA_u = Best_Angle_DEFAULTS;
volatile float g_bus_curr_fast = 0.0F;
volatile uint8_t g_bus_curr_status = BUS_FB_STARTUP;
static real32_T s_ibus_fast = 0.0F;
static real32_T s_ibus_calc = 0.0F;
static uint8_t s_ibus_valid_n = 0U;
static uint8_t s_display_div = 0U;
static const real32_T TIRE_RADIUS_M = 0.266F;

void BusCurrFast_Reset(void)
{
    s_ibus_fast = 0.0F;
    s_ibus_calc = 0.0F;
    s_ibus_valid_n = 0U;
    s_display_div = 0U;
    DW.motor_anal.bus_Curr_calc = 0.0F;
    g_bus_curr_fast = 0.0F;
    g_bus_curr_status = BUS_FB_STARTUP;
}
//上传更新函数
void updata_display(void)
{
				real32_T ibus_raw;
//		//数据更新
	    /* 定点转换: 浮点物理量 → int32_T 传输值 */
			DW.up_data_param.motor_speed      = (DW.motor_main_feedback.mech_Spd);
			DW.up_data_param.mech_angle       = (ENC_GetMecAngleDeg());
			/* 上一拍施加电压与同坐标dq反馈反算母线电流; 异常输入不进滤波。
			   快速通道供功率闭环, 慢速通道供显示; 异常时显示最后有效值。 */
				ibus_raw = CM_CalcBusCurrent(DW.motor_anal.Vd_buf, DW.motor_anal.Vq_buf,
				                             DW.pi_ds.Fbk, DW.pi_qs.Fbk,
				                             DW.motor_anal.bus_Volt);
				if ((ibus_raw < 1000000.0F) && (ibus_raw > -1000000.0F)) {
					if (s_ibus_valid_n < BUS_FB_RECOVER_SAMPLES) { s_ibus_valid_n++; }
					if (s_ibus_valid_n >= BUS_FB_RECOVER_SAMPLES) { g_bus_curr_status = BUS_FB_VALID; }
					s_ibus_fast += PWR_CL_FAST_ALPHA * (ibus_raw - s_ibus_fast);
					g_bus_curr_fast = s_ibus_fast;
#if (BUS_CURR_FILT_EN != 0)
					s_ibus_calc += BUS_CURR_FILT_ALPHA * (ibus_raw - s_ibus_calc);
#else
					s_ibus_calc += 0.002F * (ibus_raw - s_ibus_calc);
#endif
				} else {
					s_ibus_valid_n = 0U;
					g_bus_curr_status = BUS_FB_FAULT;
				}
			s_display_div++;
			if (s_display_div < 10U) { return; }
			s_display_div = 0U;
			/* 电机rpm / 减速比 -> 轮端rpm -> km/h, 保留正反转符号 */
			if (DW.motor_param.reduction_ratio > 0.0F) {
				DW.vehicle_info.vehicle_speed =
					((real32_T)DW.motor_main_feedback.mech_Spd * 2.0F * 3.14159265F *
					 TIRE_RADIUS_M * 0.06F) / DW.motor_param.reduction_ratio;
			} else {
				DW.vehicle_info.vehicle_speed = 0.0F;
			}
			DW.up_data_param.wheel_speed = DW.vehicle_info.vehicle_speed;
			DW.motor_anal.bus_Curr_calc = s_ibus_calc;
			DW.up_data_param.bus_current      = (DW.motor_anal.bus_Curr_calc);
			DW.up_data_param.bus_voltage      = (DW.motor_anal.bus_Volt);    /* V→mV */
			/* 实测dq电流对应的电磁转矩(N·m), 不使用母线功率/转速估算 */
			DW.up_data_param.torque_calc = 1.5F * (real32_T)DW.motor_param.PP_num *
				((DW.motor_param.flux_value * DW.pi_qs.Fbk) +
				 ((DW.motor_param.Ld_value - DW.motor_param.Lq_value) *
				  DW.pi_ds.Fbk * DW.pi_qs.Fbk));
			/* 调试用(临时): U 相电流通道改为上报电流圆半径 motor_param.I_max(相电流上限);
			   上位机勾"U相电流"即可确认 A4/2「最大输出电流」/「峰值电流」是否真写进了电流圆。
			   确认完把本行注释、放开下面那行注释即恢复真实 U 相电流 */
		//	DW.up_data_param.phase_u_current = DW.motor_param.I_max;
			DW.up_data_param.phase_u_current  = (DW.motor_anal.phaseU_Curr);
			DW.up_data_param.phase_v_current  = (DW.motor_anal.phaseV_Curr);
			DW.up_data_param.phase_w_current  = (DW.motor_anal.phaseW_Curr);
			DW.up_data_param.motor_temp       = (DW.motor_anal.motor_temp);
			DW.up_data_param.mos_temp         = (DW.motor_anal.mos_temp);

			DW.up_data_param.gpio_status_register  = DW.comm_contrl.gpio_state;
			DW.up_data_param.fault_register   = DW.comm_contrl.fault_state;
			DW.up_data_param.func_register    = DW.key_state_machine.fun_state;
			DW.up_data_param.throttle_voltage =(DW.motor_anal.handlebar_Volt);

			#if 0
			if(DW.comm_param.TPDO1_Enable==2)
			{
				/*********** 示波器功能 ************/
				if(DW.oscilloscope_state==0)
				{
					if(DW.comm_param.TPDO1_Tigger==1)//立即开始采集
						DW.oscilloscope_state=1;
					else if(DW.comm_param.TPDO1_Tigger==2){//启动时开始采集
						if(DW.comm_contrl.control_enable==1&&DW.adc_slmple.zero_flag == 1U&&DW.comm_contrl.control_mode != 0U)
						DW.oscilloscope_state=1;
					}
					DW.oscilloscope_cnt=0;
					DW.oscilloscope_buf[0][DW.oscilloscope_cnt]=TPDO_ReadSig((DW.comm_param.TPDO1_Load>>8)&0xFFU);
					DW.oscilloscope_buf[1][DW.oscilloscope_cnt]=TPDO_ReadSig((DW.comm_param.TPDO1_Load&0xFFU));
				}
				else if(DW.oscilloscope_state==1)
				{
					DW.oscilloscope_cnt++;
					if(DW.oscilloscope_cnt>=OSCILLOSCOPE_LEN_MAX){
						DW.oscilloscope_state=2;//采集数据完成可以开始传输数据
						DW.can_send_cnt=0;
					}
					else{
						DW.oscilloscope_buf[0][DW.oscilloscope_cnt]=TPDO_ReadSig((DW.comm_param.TPDO1_Load>>8)&0xFFU);
						DW.oscilloscope_buf[1][DW.oscilloscope_cnt]=TPDO_ReadSig((DW.comm_param.TPDO1_Load&0xFFU));
					}
				}
			}
			else if(DW.comm_param.TPDO3_Enable==2){}
			else if(DW.comm_param.TPDO4_Enable==2){}
#endif
}
/*系统辨识子函数*/

void system_identification(void)
{
	uint8_t zero_idf_can_send=0;	//零位辨识状态发送状态
	uint8_t idf_can_send=0;				//辨识状态发送状态
	can_msg_t msg = {0};
	static uint8_t zero_identify_state_last=0;
	static uint8_t identify_state_last=0;
	static uint16_t wait_cnt=0,wait_cnt1=0;//等待计时
	static uint8_t speed_ke_saved=0;    /* 旋转验证期改过加速斜率: 需还原 */
	/***********************辨识状态更新*******************************/
   // Can_state = CommCan_Send(0x434, can_tx_data, 2);
	//零位辨识状态
	/* 下电类故障期间终止辨识: 复位辨识状态与 PWM 采样器, 清电流指令/模式/使能 */
	if ((DW.comm_contrl.fault_power_down_state != 0U) &&
	    (DW.comm_contrl.zero_identify_state != ST_IDF_ZERO_IDLE)) {
	    ENC_ZeroIdf_Reset();                       /* 中止 IF 校零内部状态 + PWM 采样器 */
	    DW.comm_contrl.I_d_target = 0.0F;
	    DW.comm_contrl.I_q_target = 0.0F;
	    DW.comm_contrl.contorl_mode_can = Control_Mode_Stop;
	    DW.comm_contrl.control_enable = 0U;
	    DW.comm_contrl.zero_identify_ctrl = 0U;    /* 撤销辨识请求, 不留"续上"的种子 */
	    DW.comm_contrl.zero_identify_state = ST_IDF_ZERO_IDLE;
	    wait_cnt = 0U; wait_cnt1 = 0U;
	    if (speed_ke_saved != 0U) { DW.comm_contrl.speed_Ke_Acc = DW.comm_contrl.speed_Ke_Buf; speed_ke_saved = 0U; }
	}

	if(DW.comm_contrl.zero_identify_ctrl==2)
	{
		if(DW.comm_contrl.zero_identify_state==ST_IDF_ZERO_IDLE){//零位辨识初始化
			/* ---- 步骤0: 参数初始化 ---- */
			DW.comm_contrl.I_d_target = 0.0F;
			DW.comm_contrl.I_q_target = DW.comm_contrl.zero_identify_curr;
			DW.comm_contrl.contorl_mode_can = Control_Mode_Stop;
			DW.comm_contrl.control_enable=0;
#if (ZERO_IDF_ALGORITHM == ZERO_IDF_ALG_VQ_SCAN)
			DW.comm_contrl.contorl_mode_sub = 0U;
#endif
		  g_enc_foc.z_clear 		= 1U;
			//清除数据
			BestA_u.Z_flag				= 0;
			BestA_u.PP_cnt				= 0;
			BestA_u.Identify_STA	= 0;
			BestA_u.Vd_min				= 0xffffff;
			BestA_u.Vq_min				= 0xffffff;
			BestA_u.Angle_delta		= 16U;	/* 双向扫步进(源工程整定值, 原16过小扫圈过慢) */
			BestA_u.Angle_buff		= 0;	/* 按极对数双向扫点: 清空上次扫描中间量 */
			BestA_u.sweep_dir		= 0;	/* 0=正扫 1=反扫 */
			BestA_u.qual_fail		= 0;	/* 合成质量检查标志 */
			ENC_ZeroIdf_Reset();			/* 复位当前算法的找零状态与PWM采样器 */
			/* Z 相状态机复位: 上一轮若停在 Z_WAIT_LOW/Z_WAIT_RISE/Z_LOCKOUT 中间态,
			   本轮 Z 检测就从错的状态起步 -> Z 基准/采样点错位。原来只清 Z_flag/PP_cnt,
			   没清 z_phase_state。 */
			g_enc_foc.z_phase_state = Z_IDLE;
			//注意: 不清 g_enc_foc.zero_state/zero_offset, 校零失败时保留旧零位可用
			g_enc_foc.z_clear     = 0;
			wait_cnt=0;
			DW.comm_contrl.zero_identify_state=ST_IDF_ZERO_INIT;
		}
		else if(DW.comm_contrl.zero_identify_state==ST_IDF_ZERO_INIT){//初始化完成开始工作
			//开始工作
			DW.comm_contrl.contorl_mode_can = Control_Mode_Idq;
			DW.comm_contrl.control_enable=1;
			DW.comm_contrl.zero_identify_state=ST_IDF_ZERO_RUN;
		}
		else if(DW.comm_contrl.zero_identify_state==ST_IDF_ZERO_WAIT)//校完零后等待
		{
			if(wait_cnt==0) {//初始化
				DW.comm_contrl.contorl_mode_can = Control_Mode_Stop;
				DW.comm_contrl.control_enable=0;
			}
			else if(wait_cnt>=1000)//暂停1000ms
			{
				wait_cnt=0;
				wait_cnt1=0;
				/* 验证前从 EEPROM 重读零位基准(pwm_zero_ref/pwm_dir/zero_hyst_cnt/pwm_ref_valid);
				勿在此调 ENC_Init() —— 它会 memset g_enc_foc */
				ParamStore_ZeroRef_Load();
				//旋转验证-正转测试
				DW.comm_contrl.zero_identify_state=ST_IDF_ZERO_TESTs;
				DW.comm_contrl.contorl_mode_sub=0;
				DW.comm_contrl.speed_target =DW.comm_contrl.identify_speed;
				DW.comm_contrl.speed_Ke_Buf=DW.comm_contrl.speed_Ke_Acc;
				DW.comm_contrl.speed_Ke_Acc=1000;/* 验证期斜率(RPM/s 真值): 1000rpm 需 1s < 2s 窗 */
				speed_ke_saved=1;
				DW.comm_contrl.contorl_mode_can = Control_Mode_Speed;
				DW.comm_contrl.control_enable=1;
				DW.comm_contrl.zero_identify_result=0;
			}
			wait_cnt++;
		}
		else if(DW.comm_contrl.zero_identify_state==ST_IDF_ZERO_TESTs)
		{
			if(++wait_cnt>=2000)
			{
				//正转验证停止
				wait_cnt=0;
				DW.comm_contrl.zero_identify_state = ST_IDF_ZERO_TESTt;
				DW.comm_contrl.contorl_mode_can = Control_Mode_Stop;
				DW.comm_contrl.control_enable=0;
			}
			else if(DW.comm_contrl.zero_identify_result==0){//正转速度验证
				if(DW.motor_main_feedback.mech_Spd>=DW.comm_contrl.speed_target&&DW.motor_main_feedback.mech_Spd<=DW.comm_contrl.speed_target+200) {
					if(++wait_cnt1>=100)
					{
						wait_cnt1=0;
						DW.comm_contrl.zero_identify_result=1;
					}
				}
			}
		}
		else if(DW.comm_contrl.zero_identify_state==ST_IDF_ZERO_TESTt)
		{
			if(++wait_cnt>=1000)
			{
				ParamStore_ZeroRef_Load();   /* 反转验证前再对齐一次(同上) */
				//旋转验证-反转测试
				wait_cnt=0;
				wait_cnt1=0;
				DW.comm_contrl.zero_identify_state=ST_IDF_ZERO_TESTn;
				DW.comm_contrl.contorl_mode_sub=0;
				DW.comm_contrl.speed_target =-DW.comm_contrl.identify_speed;
				/* 反转验证同样走加速档(幅值增大): 临时斜率保持, 减速斜率不动 */
				DW.comm_contrl.contorl_mode_can = Control_Mode_Speed;
				DW.comm_contrl.control_enable=1;
			}
		}
		else if(DW.comm_contrl.zero_identify_state==ST_IDF_ZERO_TESTn)
		{
			if(++wait_cnt>=2000)
			{
				//反转验证停止(并还原验证期的临时加速斜率)
				wait_cnt=0;
				if (speed_ke_saved != 0U) { DW.comm_contrl.speed_Ke_Acc = DW.comm_contrl.speed_Ke_Buf; speed_ke_saved = 0U; }
				DW.comm_contrl.zero_identify_state = ST_IDF_ZERO_CHECK;
				DW.comm_contrl.contorl_mode_can = Control_Mode_Stop;
				DW.comm_contrl.control_enable=0;
			}
			else if(DW.comm_contrl.zero_identify_result==1){//反转速度验证
				if(DW.motor_main_feedback.mech_Spd<=DW.comm_contrl.speed_target&&DW.motor_main_feedback.mech_Spd>=DW.comm_contrl.speed_target-200) {
					if(++wait_cnt1>=100)
					{
						wait_cnt1=0;
						DW.comm_contrl.zero_identify_result=2;
					}
				}
			}
		}
		else if(DW.comm_contrl.zero_identify_state==ST_IDF_ZERO_CHECK)
		{
			if(++wait_cnt>=100)
			{
				//输出结果然后复位
				wait_cnt=0;
				if(DW.comm_contrl.zero_identify_result==2)//校零成功
					DW.comm_contrl.zero_identify_state = ST_IDF_ZERO_OK;
				else
					DW.comm_contrl.zero_identify_state = ST_IDF_ZERO_FIRL;
				DW.motor_main_feedback.elec_angle_offset =(uint16_T)(g_enc_foc.pwm_zero_ref & 0xFFFFU);
				DW.comm_contrl.control_enable=0;
				DW.comm_contrl.I_d_target = 0.0F;
				DW.comm_contrl.I_q_target = 0.0F;
				DW.comm_contrl.contorl_mode_can = Control_Mode_Stop;
				DW.comm_contrl.zero_identify_ctrl=0;
				wait_cnt=0;
			}
		}
		else if(DW.comm_contrl.zero_identify_state==ST_IDF_ZERO_FIRL)//合成质量检查失败(迟滞/散布超限, 由ENC_ZeroIdf_Synth直接置位)
		{
			//必须停注入并退出辨识, 否则电机会一直Idq注入
			DW.comm_contrl.control_enable=0;
			DW.comm_contrl.I_d_target = 0.0F;
			DW.comm_contrl.I_q_target = 0.0F;
			DW.comm_contrl.contorl_mode_can = Control_Mode_Stop;
			DW.comm_contrl.zero_identify_ctrl=0;
			wait_cnt=0;
		}
	}
	else{
		if(DW.comm_contrl.zero_identify_state!=ST_IDF_ZERO_IDLE){
		ENC_ZeroIdf_Reset();   /* 取消辨识时一并中止采样器(原来漏了, 会残留占用解码) */
			DW.comm_contrl.I_d_target = 0.0F;
			DW.comm_contrl.I_q_target = 0.0F;
			DW.comm_contrl.contorl_mode_can = Control_Mode_Stop;
			DW.comm_contrl.control_enable=0;
			wait_cnt=0;
			DW.comm_contrl.zero_identify_state=ST_IDF_ZERO_IDLE;

		}
	}
	//参数辨识状态
	if(DW.comm_contrl.identify_ctrl==2)
	{
		if(DW.comm_contrl.identify_state==ST_IDF_PARAM_INIT){//参数辨识初始化
			/* ---- 步骤0: 参数初始化 ---- */
			DW.comm_contrl.identify_state=ST_IDF_PARAM_INIT;
		}
	}
	else{
		if(DW.comm_contrl.identify_state!=ST_IDF_PARAM_IDLE){
		DW.comm_contrl.control_enable = 0U;   /* 参数辨识取消也要关使能(原来只清模式/电流) */
			DW.comm_contrl.I_d_target = 0.0F;
			DW.comm_contrl.I_q_target = 0.0F;
			DW.comm_contrl.contorl_mode_can = Control_Mode_Stop;
			DW.comm_contrl.zero_identify_state=ST_IDF_PARAM_IDLE;
			wait_cnt=0;
		}
	}
	//校零辨识状态更新后发送状态到上位机
	if(zero_identify_state_last!=DW.comm_contrl.zero_identify_state){
		zero_identify_state_last=DW.comm_contrl.zero_identify_state;
		zero_idf_can_send=1;
	}
	if(identify_state_last!=DW.comm_contrl.identify_state){
		identify_state_last=DW.comm_contrl.identify_state;
		idf_can_send=1;
	}
	//零位辨识状态上传
	if (zero_idf_can_send == 1) {
			msg.id  = TPDO2_COB_ID;
			msg.len = 8;
			msg.data[0] = 0xA7;
			msg.data[1] = 0x01;
			msg.data[2] = 0x00;
			msg.data[3] = 0x03;
			msg.data[7] = zero_identify_state_last;
			can_send_buffer_put(&msg);
	}
	//参数辨识状态上传
	if (idf_can_send == 1) {
			msg.id  =  TPDO2_COB_ID;
			msg.len = 8;
			msg.data[0] = 0xA7;
			msg.data[1] = 0x02;
			msg.data[2] = 0x00;
			msg.data[3] = 0x02;
			msg.data[7] = identify_state_last;
			can_send_buffer_put(&msg);
	}
}
//状态寄存器写更新载入读取
uint32_t state_reg_func(void)
{
	uint32_t state_buf=0;
	uint32_t ctrl_mode_buf=0;
	//bit 0 开机状态
	state_buf=DW.comm_contrl.control_Run_state&0x01;
	//bit 1 动力解锁
	state_buf|=(DW.comm_contrl.control_enable&0x01)<<1;   /* 2026-08-11 上位机流程图适配 F2: 原误用=覆盖Bit0, 改|= */
	//bit 2 故障状态
	if(DW.comm_contrl.fault_state!=0)
		state_buf|=0x04;
	//bit 3 预充状态
	if(DW.fun_state_machine.precharge_Sta!=0)
		state_buf|=0x08;
	//bit 4~7 系统状态
	state_buf|=(DW.key_state_machine.system_status&0x0f)<<4;
	//bit 8~9 控制状态
	if(DW.comm_contrl.control_mode==0)
		ctrl_mode_buf=0;
	else	if(DW.comm_contrl.control_mode==1)
		ctrl_mode_buf=1;
	else	if(DW.comm_contrl.control_mode==2||DW.comm_contrl.control_mode==3)
		ctrl_mode_buf=2;
	else
		ctrl_mode_buf=3;
	state_buf|=(ctrl_mode_buf&0x03)<<8;
	//bit 10~11 骑行模式状态
	state_buf|=(DW.ctrl_hw_io.gear_o&0x03)<<10;
	//bit 12~13 刹车状态
	state_buf|=(DW.ctrl_hw_io.brake_o&0x03)<<12;
	//bit 14~15 能量状态
	state_buf|=(DW.fun_state_machine.ER_mode&0x03)<<14;
	//bit 16~19 骑行状态
	state_buf|=(DW.key_state_machine.control_status&0x0f)<<16;
	//bit 20 定速巡航状态
	state_buf|=(DW.fun_state_machine.TTF_enable&0x01)<<20;
	//bit 21 陡坡缓降状态
	state_buf|=(DW.fun_state_machine.HDC_enable&0x01)<<21;
	//bit 22 TCS状态
	state_buf|=(DW.fun_state_machine.TCS_enable&0x01)<<22;
  //bit 23 坡道驻车状态
	state_buf|=(DW.fun_state_machine.HCC_enable&0x01)<<23;
	//bit 24 应急骑行状态
	state_buf|=(DW.fun_state_machine.emergency_cycling_Sta&0x01)<<24;
	//bit 25 TCS干涉状态  /* 2026-08-11 上位机流程图适配: 原注释误写"应急骑行状态", 按上位机位定义更正 */
	state_buf|=(DW.fun_state_machine.TCS_state&0x01)<<25;
	//bit 26 边撑状态
	state_buf|=(DW.ctrl_hw_io.side_stand_o&0x01)<<26;
	//bit 27 设备自检状态(0:未完成 1:完成)
	state_buf|=(DW.fun_state_machine.self_check_Sta&0x01)<<27;
	//bit 28~31 挡位状态(0~6)
	state_buf|=(DW.key_state_machine.gear_status&0x07)<<28;
	return state_buf;
}
//GPIO寄存器写更新载入读取
uint8_t gpio_state[9];//GPIO电平状态
uint32_t gpio_handle(void)
{
		uint32_t gpio_state_buf=0;
		int i=0;
	  /* GPIO 原始电平采集 -> 交 SignalProc_GpioIdentify 去抖/识别。 */
		DW.ctrl_hw_io.CCS_gpio         = (uint8_T)HAL_GPIO_ReadPin(IO_IN_CCS_GPIO_Port,    IO_IN_CCS_Pin);
		DW.ctrl_hw_io.P_gpio           = (uint8_T)HAL_GPIO_ReadPin(IO_IN_P_GPIO_Port,      IO_IN_P_Pin);
		DW.ctrl_hw_io.side_stand_gpio  = (uint8_T)HAL_GPIO_ReadPin(IO_IN_SS_GPIO_Port,      IO_IN_SS_Pin);
		DW.ctrl_hw_io.gear_bus_gpio[0] = (uint8_T)HAL_GPIO_ReadPin(IO_IN_SDH_GPIO_Port,    IO_IN_SDH_Pin);
		DW.ctrl_hw_io.gear_bus_gpio[1] = (uint8_T)HAL_GPIO_ReadPin(IO_IN_SDL_GPIO_Port,    IO_IN_SDL_Pin);
		DW.ctrl_hw_io.boost_gpio       = (uint8_T)HAL_GPIO_ReadPin(IO_IN_PUSH_GPIO_Port,   IO_IN_PUSH_Pin);
		DW.ctrl_hw_io.brake_gpio       = (uint8_T)HAL_GPIO_ReadPin(IO_IN_H_BACK_GPIO_Port, IO_IN_H_BACK_Pin);
		DW.ctrl_hw_io.R_gpio           = DW.ctrl_hw_io.boost_gpio;
		DW.ctrl_hw_io.burglar_gpio     = (uint8_T)HAL_GPIO_ReadPin(IO_IN_FD_GPIO_Port,     IO_IN_FD_Pin);
		DW.ctrl_hw_io.ACC_gpio    		 = 0;
		gpio_state[0] = DW.ctrl_hw_io.brake_gpio;//刹车
	  gpio_state[1] = DW.ctrl_hw_io.side_stand_gpio;//边撑
	  gpio_state[2] = DW.ctrl_hw_io.boost_gpio ;//推车
		gpio_state[3] = DW.ctrl_hw_io.R_gpio;//倒车
    gpio_state[4] = DW.ctrl_hw_io.gear_bus_gpio[0];
    gpio_state[5] = DW.ctrl_hw_io.gear_bus_gpio[1];
	  gpio_state[6] = DW.ctrl_hw_io.P_gpio;
    gpio_state[7] = DW.ctrl_hw_io.CCS_gpio;
    gpio_state[8] = DW.ctrl_hw_io.burglar_gpio;
		for(i=0;i<9;i++)
		{
			gpio_state_buf|=gpio_state[i]<<i;
		}
		return gpio_state_buf;
}
//固化逻辑执行
void strore_handle(void)
{
	uint8_t ret=0,store_state=0;
	can_msg_t msg = {0};
	//1、收到了固化消息，开始固化
	if(DW.sys_param.save_status==1)
	{
		if (DW.comm_contrl.control_enable==0U)
		{
			DW.sys_param.save_status=2;//固化中
		}
		else
		{
			DW.sys_param.save_status=4;//固化失败-电机正在运行(只读)
		}
		ret=1;
	}
	//2、固化代码
	else if(DW.sys_param.save_status==2)
	{
		store_state=ParamStore_OnCommitCmd(PARAM_COMMIT_CHANGED);//读取存储状态 PARAM_COMMIT_CHANGED PARAM_COMMIT_ALL
		if(store_state==0)
			DW.sys_param.save_status=3;//固化成功
		else if(store_state==1)
			DW.sys_param.save_status=4;//存储器损坏
		else
			DW.sys_param.save_status=5;//存储中断
		ret=1;
	}
	//3、状态复位
	else if(DW.sys_param.save_status==2)
	{
		DW.sys_param.save_status=0;
	}
	//上传一次状态
	if(ret==1)
	{
			msg.id  = TPDO2_COB_ID;
			msg.len = 8;
			msg.data[0] = 0xA4;
			msg.data[1] = 0x05;
			msg.data[2] = 0x00;
			msg.data[3] = 0x00;
			msg.data[7] = DW.sys_param.save_status;
			can_send_buffer_put(&msg);
	}
}
//紧急报文发送
void EMCY_TrySend(uint32_t cob_id)
{
		can_msg_t msg;
		
				msg.id = EMCY_COB_ID;
				msg.len = 8U;
				memset(msg.data, 0, 8U);
//获取数据
				msg.data[0] = (uint8_t)((cob_id >> 24) & 0xFF);
				msg.data[1] = (uint8_t)((cob_id >> 16) & 0xFF);
				msg.data[2] = (uint8_t)((cob_id >> 8) & 0xFF);
				msg.data[3] = (uint8_t)(cob_id & 0xFF);
//CAN发送		
				can_send_prio_put(&msg);   /* 高优先级: 环形缓冲满也不丢 */
}
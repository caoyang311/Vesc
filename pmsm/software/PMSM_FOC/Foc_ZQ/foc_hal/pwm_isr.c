/******************************************************************************
 * 文件: pwm_isr.c
 * 功能: TIM1 10kHz 更新中断, 驱动整条电机控制链 (采样->滤波->编码器->Z相->故障->FOC->PWM)
 * 创建日期: 2026-06-22
 * 修改日期: 2026-10-26
 * 修改内容: 普通显示与派生计算降为1kHz, 示波器诊断停用;
 *           增加DWT本次/峰值ISR负载率统计(70%预算5040 cycles@72MHz)
 * 修改人: rui.zhang
 ******************************************************************************/

/* 10kHz 每拍流程:
 * 1) 读 ADC 注入采样
 * 2) SignalProc_AnalogConvert
 * 3) ENC_CalcAngle -> 写入 elec_angle (编码器)
 * 4) ZPhase_Process: 10kHz, 首次找到Z相前持续检测
 * 5) 1kHz慢任务: PWM自检窗 + ENC_CalcSpeed + 温度/功率采集 + PI 参数生效
 * 6) 控制模式仲裁 (CAN / 功能模式); 位置基准未建立时归零
 * 7) 故障检测/保护 + EMCY 上报
 * 8) ENC_ZeroIdf_Task: 10kHz, 辨识中最终覆盖 control_mode + elec_angle + 电流指令
 * 9) 启停逻辑 + FOC_Ctrl_Task + PWM 输出
 */

#include "stm32f3xx_hal.h"
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "foc_ctrl.h"
#include "foc_data.h"
#include "fault_protect.h"

#include "signal_proc.h"
#include "encoder_foc_sdk.h"
#include "hall_sensor.h"     /* 霍尔接口(与 ABZ 复用引脚) */
#include "encoder_pwm_u.h"
#include "Parameter_Idf.h"
#include "foc_pi.h"
/* 协议相关 */
#include "canopen.h"

uint32_t volatile ISR_cnt=0;
volatile uint32_t g_pwm_isr_cycles = 0U;
volatile uint32_t g_pwm_isr_max_cycles = 0U;
volatile uint32_t g_pwm_isr_load_pct = 0U;
volatile uint32_t g_pwm_isr_load_max_pct = 0U;
volatile uint32_t g_sensor_cycles = 0U;
volatile uint32_t g_fault_fast_cycles = 0U;
volatile uint32_t g_foc_ctrl_cycles = 0U;
volatile uint32_t g_display_cycles = 0U;
uint16_t volatile g_emcy_cnt = 0U;   /* 故障上报次数(调试观察: 到底有没有报出去) */
uint8_t volatile g_spd_updated = 0U;   /*fault_protect 使用 */
//上传参数更新
void updata_display(void);
/* 定时器更新中断回调 (重写 HAL 弱函数): 仅 TIM1 分支执行控制链 */
/* ---- PWM 自检窗: 未运行 40ms 窗 / 运行中 20ms 窗, 间隔 1s; 找零采样器占用时让位 ---- */
static void PwmSelfCheckWin_1ms(void)
{
    static uint16_t s_gap = 0U;
    static uint16_t s_win = 0U;
    uint8_t  running = ((DW.comm_contrl.control_mode != 0U) &&
                        (DW.comm_contrl.control_enable == 1U)) ? 1U : 0U;
    uint16_t win_len = (running != 0U) ? 20U : 40U;
    uint16_t gap_len = 1000U;   /* 窗间隔 1s */
    static uint8_t s_run_last = 0U;
    static uint8_t s_pend_win = 0U;   /* 使能沿 / 找零结束: 请求立即补开一次窗 */
    /* 使能沿立即补开一窗 */
    if ((running != 0U) && (s_run_last == 0U)) { s_pend_win = 1U; }
    s_run_last = running;

    if (EncPwmSamp_IsActive() != 0U) {
        /* 找零采样器占用: 让位, 结束后补一窗 */
        s_gap = 0U; s_win = 0U; s_pend_win = 1U;
    } else if (((DW.comm_contrl.zero_identify_state != 0U) &&
                (DW.comm_contrl.zero_identify_state < 100U)) ||
               (DW.comm_contrl.identify_state != 0U)) {
        /* 辨识进行中: 本调度让位并保持解码开启 */
        Encoder_Decode_Enable();
        s_gap = 0U; s_win = 0U; s_pend_win = 1U;
    } else if (s_pend_win != 0U) {
        s_pend_win = 0U;
        s_gap = gap_len;              /* 下一拍即开窗(约 1ms 后) */
    } else if (s_win != 0U) {
        if (++s_win >= win_len) { s_win = 0U; s_gap = 0U; Encoder_Decode_Disable(); }
    } else if (++s_gap >= gap_len) {
        s_gap = 0U; s_win = 1U; Encoder_Decode_Enable();
    }
}
volatile uint32_t start_cycle, end_cycle, elapsed_cycle;
volatile float elapsed_us=0;
static float abs_f(float v) { return (v < 0.0f) ? -v : v; }
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint8_t speed_div  = 0U;   /* 10kHz -> 1kHz 分频 */
		static uint8_t last_power_state=0;
	static uint8_t s_pwm_started = 0U;   /* 零窗结束后输出已开启(1)/未开启(0) */
		static uint8_t s_clr_last = 0U;   /* 清故障上升沿检测 */
    if (htim->Instance == TIM1)
    {
        uint32_t t_section;
			  /* DWT 周期计数器是霍尔接口的自由运行时基, 此处不得清零(只记起点做差) */
//				start_cycle = DWT->CYCCNT;
//        g_sensor_cycles = 0U;
//        g_fault_fast_cycles = 0U;
//        g_foc_ctrl_cycles = 0U;
//        g_display_cycles = 0U;

		//	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,0);
				ISR_cnt++;
        /* SVPWM 半周期基准 PWM_HalfPerMax = 1799 (对应 TIM1 ARR 3599) */

        /******** 1) ADC 注入采样原始值 ********
              ADC2: rank1=U相电流(PB2) rank2=转把电压(PC2) rank3=母线电压(PC4)
              ADC3: rank1=V相电流(PB1) rank2=W相电流(PB0)                       */
        DW.adc_slmple.phaseU_Curr			 = (uint16_T)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
        DW.adc_slmple.handlebar_Volt   = (uint16_T)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
        DW.adc_slmple.bus_Volt 			   = (uint16_T)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3);
        DW.adc_slmple.phaseW_Curr			 = (uint16_T)HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1);
        DW.adc_slmple.phaseV_Curr			 = (uint16_T)HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_2);

        /* ******** 2) ADC 转换 + 滤波 (含温度电阻 -> 温度) ******** */
        SignalProc_AnalogConvert();

        /********* 3) 转子电角度更新 (10kHz, 编码器读数) ********
         校零完成后 ENC_CalcAngle 内部自动应用 zero_offset 校正; 16bit 电角度 -> 0~4095 计数 */
        if (Hall_IF_IsActive() != 0U) {
            Hall_Angle_Task();            /* 霍尔路径: 含扇区内插值, 直接写 elec_angle */
        } else {
        ENC_CalcAngle();
				if(DW.comm_contrl.zero_identify_state!=ST_IDF_ZERO_RUN)
        DW.motor_main_feedback.elec_angle =(uint16_T)((((uint16_t)g_enc_foc.hElAngle + 32768U) >> 4) & 0x0FFFU);
        }
        

        /* ******** 4) Z相检测 (10kHz, 首次找到Z相前持续运行) ********
              参考工程: Z相上升沿清零 ENC_TIM->CNT 建立零位参考。
              找到首次 Z 相后 (z_first_state==1) 不再调用, 仅在辨识中由 ENC_ZeroIdf_Task 调用。 */
        if ((g_enc_foc.z_first_state == 0U) && (Hall_IF_IsActive() == 0U)
#if (ZERO_IDF_ALGORITHM == ZERO_IDF_ALG_VQ_SCAN)
            && (DW.comm_contrl.zero_identify_state != ST_IDF_ZERO_RUN)
#endif
           ) {
            ZPhase_Process();             /* Z 相检测属 ABZ 接口; 霍尔模式跳过(Z 脚即霍尔 W) */
        }

        /* ******** 5) 1kHz 慢任务 (每 10 拍) ******** */
        if (++speed_div >= 10U) {
        	/* PWM 自检窗: 按 1ms 调用 */
        	PwmSelfCheckWin_1ms();
            speed_div = 0U;
            /* 接口管理与速度分发: 霍尔与 ABZ 共用 g_enc_foc.hAvrSpeedRPM 出口,
               下游 mech_Spd/故障层消费无需改动 */
            Hall_IF_Poll(DW.motor_param.sensor_type);
            if (Hall_IF_IsActive() != 0U) { Hall_Speed_Task(); }
            else                          { ENC_CalcSpeed(); }
						g_spd_updated = 1U;   /* 文件级 uint8_T, fault_protect 消费 */
						/* 四舍五入: 直接截断会再带 -0.5rpm 的固定偏置 */
						DW.motor_main_feedback.mech_Spd = (int32_T)(g_enc_foc.hAvrSpeedRPM +
						                                   ((g_enc_foc.hAvrSpeedRPM >= 0.0f) ? 0.5f : -0.5f));
						/* A5/02/03 电角度偏移: 辨识算出的 zero_offset 运行值(16bit 电角度域, 不折算);
           EEPROM 存的是 pwm_zero_ref/pwm_dir/zero_hyst_cnt (A5/02/04~06) */
            /* PWM 绝对位置辨识 (g_pwm_idf_enable 置 1 时采集, 采满自动关闭, 结果已滤波) */
           // Encoder_PosIdf_Task();
            /* comm_pi_bus (CAN A1 参数总线) -> 实际 PI 环, 1kHz 生效 */
            PI_Apply_CommParams();
            FOC_MotorParams_Poll();

            /* 温度 ADC(电机PC0/控制器PC1): 已移至主循环 Temp_Sensor_Update() 采集+解析,
               不在此 10kHz 中断内做 (含 logf 等耗时运算, 且无需高频更新)。 */
        }
//        g_sensor_cycles = DWT->CYCCNT - start_cycle;

				/* ******** 6) 故障检测/保护 (产出 fault_state / fault_power_down_state) ******** */
				if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) != RESET) {
					DW.faule_protection.short_circuit = 1U;
				}
				/* fault_clear 脉冲期间同步清 TIM1 Break 标志 */
		
				if (DW.faule_protection.fault_clear == 1U) {
					__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
					__HAL_TIM_MOE_ENABLE(&htim1);   /* 新增: 恢复 OC4 输出, 让 ADC 触发立即恢复 */
					/* 清故障后作废 zero_state 并重启找零 */
					if (s_clr_last == 0U) {
						g_enc_foc.zero_state = 0U;
						SignalProc_PwmZeroCalib_Start();
					}
					s_clr_last = 1U;
				} else {
					s_clr_last = 0U;
				}
				if ((DW.adc_slmple.zero_flag == 1U)
				    || (DW.faule_protection.self_check == 1U)
				    || (DW.comm_contrl.fault_state != 0U))
				{
					//运行时不允许清除故障
					if(DW.faule_protection.fault_clear==1)
					{
						if (DW.comm_contrl.control_mode != 0U && DW.comm_contrl.control_enable==1U)
							DW.faule_protection.fault_clear=0;
					}
//					t_section = DWT->CYCCNT;
					Fault_FastTask();
//					g_fault_fast_cycles = DWT->CYCCNT - t_section;
				}

					/* 故障上报: **故障状态变化时只发一次**(紧急报文不重复发送);
					   不要求"已使能" —— 下电类故障会在同一拍把 control_mode 清 0, 加使能门就永远报不出去;
					   报文走高优先级发送槽, 环形缓冲满也不丢(can_send_prio_put)。 */
					if (DW.comm_contrl.fault_state_lsat != DW.comm_contrl.fault_state)
					{
						uint32_T fs_now = DW.comm_contrl.fault_state;
						DW.comm_contrl.fault_state_lsat = fs_now;
						if (fs_now != 0U)
						{
							g_emcy_cnt++;                 /* 观察量: 上报次数 */
							EMCY_TrySend(fs_now);         /* 只发这一次 */
						}
					}


        /* ******** 7) 控制模式仲裁 ******** */
				if (DW.comm_contrl.fault_power_down_state == 1U) {
          DW.comm_contrl.control_mode = 0U;
					DW.comm_contrl.contorl_mode_can=0;
					DW.comm_contrl.contorl_mode_fun=0;
					DW.comm_contrl.control_enable=0U;
        }
//				else if ((g_enc_foc.zero_state == 0U) &&
//				         (DW.comm_contrl.identify_state == 0U) &&
//				         !((DW.comm_contrl.zero_identify_state != 0U) && (DW.comm_contrl.zero_identify_state < 100U)) &&
//				         (DW.comm_contrl.control_mode != (uint16_T)Control_Mode_VF) &&
//				         (DW.comm_contrl.control_mode != (uint16_T)Control_Mode_FD) &&
//				         (DW.comm_contrl.control_mode != (uint16_T)Control_Mode_HighFre)) {
//					/* 位置基准未建立(且非开环/非辨识): mode 归零, 使"输出已关断"对上层可见 */
//					DW.comm_contrl.control_mode = 0U;
//				}
				else{
					if (DW.comm_contrl.sys_run_mode == 0U) {
						DW.comm_contrl.control_mode = DW.comm_contrl.contorl_mode_can;
					} else {
						DW.comm_contrl.control_mode = DW.comm_contrl.contorl_mode_fun;
					}
				}
        /* ******** 8) 启停逻辑 + FOC ******** */
				if (DW.comm_contrl.control_mode == 0U||DW.comm_contrl.control_enable==0U) {
						if(last_power_state==1)
						{
							/* 关闭输出 */
							last_power_state=0;
							motor_stop();
							s_pwm_started = 0U;
							//DW.comm_contrl.fault_power_down_state=0;//锁存，不自动恢复
						}
				} 
				/*
									        && ((g_enc_foc.zero_state != 0U)
					            || (DW.comm_contrl.control_mode == (uint16_T)Control_Mode_VF)
					            || (DW.comm_contrl.control_mode == (uint16_T)Control_Mode_FD)
					            || (DW.comm_contrl.control_mode == (uint16_T)Control_Mode_HighFre)
					            || ((DW.comm_contrl.zero_identify_state != 0U) &&
					                (DW.comm_contrl.zero_identify_state < 100U))
					            || (DW.comm_contrl.identify_state != 0U))
				*/
				else if((DW.comm_contrl.control_enable==1U)
					        && (DW.faule_protection.self_check==0U)
					        && (DW.comm_contrl.fault_state & FAULT_PD_MASK)==0U
					        /* 互锁: zero_state==0 时不开输出; 开环模式与辨识期间豁免 */){
					if(last_power_state==0) {
						 last_power_state=1;
							/* 零位校准防污染: 校准窗内不开输出 —— 电机悬空时相电流真为零,
							   采到的才是纯偏移。此前窗内 PWM 预置三相同占空比(零矢量),
							   旋转电机的反电势经零矢量短路打出真实电流, 偏移随转速/位置漂移。
							   带速再使能(高于基速后体二极管整流无法用关断规避): 跳过本窗
							   沿用上次偏移, 直接放行走 FOC */
							if (abs_f((float)DW.motor_main_feedback.mech_Spd) > 500.0f) {
								DW.adc_slmple.zero_flag = 1U;
								s_pwm_started = 1U;
							} else {
								DW.adc_slmple.zero_flag = 0U;
							}
					 }
					if (DW.adc_slmple.zero_flag == 1U) { /* 零窗结束/跳过: 零偏已就绪 */
						 /* ******** 9) 编码器零位辨识 (10kHz载波频率) *********/
						if(DW.comm_contrl.zero_identify_state==ST_IDF_ZERO_RUN){
							ENC_ZeroIdf_Task();
						}
						if(DW.comm_contrl.identify_state!=ST_IDF_PARAM_IDLE){
						}
						/* ******** 10) FOC 控制算法 (产出 DW.ctrl_hw_io.SVPTa/b/c) ******** */
/* 输出限幅在 FOC_Ctrl_Task 需求级(Pwr_Limit_Curr, MTPA/弱磁之前)执行 */
//						t_section = DWT->CYCCNT;
						FOC_Ctrl_Task();
//						g_foc_ctrl_cycles = DWT->CYCCNT - t_section;
				 }
						/* 零窗结束(或带速跳过)后首次到达此处才真正开输出 */
						if (s_pwm_started == 0U) { s_pwm_started = 1U; motor_start(); }
						/* ******** 11) 输出 PWM ********/
	            htim->Instance->CCR3 = DW.ctrl_hw_io.SVPTa;
	            htim->Instance->CCR2 = DW.ctrl_hw_io.SVPTb;
	            htim->Instance->CCR1 = DW.ctrl_hw_io.SVPTc;
	      }
				/****** 更新数据**********/
			//	if(DW.comm_param.TPDO1_Enable!=0||DW.comm_param.TPDO3_Enable!=0||DW.comm_param.TPDO4_Enable!=0)
//        t_section = DWT->CYCCNT;
        updata_display();
//        g_display_cycles = DWT->CYCCNT - t_section;
//				end_cycle = DWT->CYCCNT;
//        elapsed_cycle = end_cycle - start_cycle;
//        g_pwm_isr_cycles = elapsed_cycle;
//        g_pwm_isr_load_pct = (elapsed_cycle * 100U) / (SystemCoreClock / 10000U);
//        if (elapsed_cycle > g_pwm_isr_max_cycles) {
//            g_pwm_isr_max_cycles = elapsed_cycle;
//            g_pwm_isr_load_max_pct = g_pwm_isr_load_pct;
//        }
        /* 70%预算: 72MHz/10kHz时为5040 cycles */
	    }
	}

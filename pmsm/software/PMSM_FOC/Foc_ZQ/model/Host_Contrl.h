/*
 * File: Host_Contrl.h  (MATLAB 1:1 镜像)
 *
 * Code generated for Simulink model 'MCU_MotorContrl_2024b'.
 * Model version      : 1.1974
 * Simulink Coder     : 24.2 (R2024b)
 *
 * 注意: 与工程已有的 CAN 协议 Host_Contrl.h 同名不同义。
 *       本文件(model/Host_Contrl.h)定义 MATLAB 模型的通讯/电机参数结构体,
 *       仅供 FOC 算法层用显式前缀 #include "model/Host_Contrl.h" 引用。
 */

#ifndef Host_Contrl_h_
#define Host_Contrl_h_
#include "rtwtypes.h"

/* 通讯综合控制器 */
typedef struct {
  uint8_T  sys_run_mode;           /* 系统运行模式 0:调试 1:摩托模式 2:摩托测试 */
  uint8_T  mos_temp_sensor;        /* MOS温度传感器类型 0:KTY84 1:PT100 2:PT1000 3:KTY83 4:NTC-10K */
  uint16_T control_enable;         /* 控制使能 */
  uint16_T control_mode;           /* 控制模式 */
  uint16_T contorl_mode_fun;       /* 控制模式-调试参数设定 */
  uint16_T contorl_mode_can;       /* 控制模式-CAN设定 */
  uint16_T fault_power_down_state; /* 故障下电状态 */
  uint8_T  contorl_mode_sub;       /* 控制模式子类 0:Id=0 1:MTPA+弱磁 2:查表 */
  uint8_T  control_Run_state;      /* 系统运行状态 0:停止 1:运行 */
  uint16_T motor_direction;        /* 电机方向 */
  real32_T speed_target;           /* 速度控制目标 */
  real32_T speed_Ke_Acc;           /* 速度斜坡Ke-加速 */
  real32_T speed_Ke_Dece;          /* 速度斜坡Ke-减速 */
  real32_T speed_Ke_Buf;           /* 速度斜坡Ke缓存 */
  real32_T I_d_target;             /* Idq模式 Id目标 */
  real32_T I_q_target;             /* Idq模式 Iq目标 */
  real32_T position_target;        /* 位置控制目标 */
  real32_T VF_freq;                /* VF频率 */
  real32_T VF_Vq_Vol;              /* VF Vq电压 */
	real32_T VF_Vd_Vol;              /* VF Vd电压 */
  real32_T I_max;                  /* 弱磁最大电流幅值 A */
	real32_T Te_target;              /* 扭矩控制目标 N.m */
  uint32_T FW_speed_node;          /* 已废弃: 无消费者(闭环弱磁走电压利用率, 查表走 Id 表);
                                 仅保留 CAN 入口与 EEPROM 槽位, 值不再固化
                                 (槽位在 s_items 中部, 删除会使老板子已固化参数错位) */
  real32_T zero_Ke;                /* 回零斜坡 */
	uint8_T  identify_ctrl;          /* 在线辨识控制 */
  uint8_T  identify_state;         /* 在线辨识状态 */
	uint8_T  zero_identify_ctrl;     /* 零位辨识控制 */
  uint8_T  zero_identify_state;    /* 零位辨识状态 */
	uint8_T  zero_identify_result;   /* 零位辨识结果 */
  real32_T identify_curr;          /* 辨识注入电流 */
  real32_T zero_identify_curr;     /* 零位辨识注入电流 */
  real32_T identify_speed;         /* 辨识验证转速 */
  uint16_T fault_recover;          /* 故障恢复 */
  uint32_T fault_state;            /* 故障状态 */
	uint32_T fault_state_lsat;       /* 上一次的故障状态 */
	uint32_T gpio_state;             /* IO状态 */
  uint16_T fault_stop_flag;        /* 故障急停标志 */
  uint16_T save_flag[4];           /* 保存标志 */
  uint16_T updata_enable;          /* 上传使能 */
  uint16_T aoto_Upload_Flag;       /* 自动上传标志 */
  uint16_T zero_ref_manual;         /* 手动零位基准: 新的 pwm_zero_ref (0~4095) */
  uint8_T  zero_ref_manual_set;     /* 1=CAN 已下发, 待线程内反算 zero_offset 并落盘 */
} Comm_Contrl_Type;

/* 通讯反馈状态 */
typedef struct {
  uint8_T function_Sta[2];
  uint8_T Acc_Sta;
  uint8_T Dece_Sta;
  uint8_T BG_Sta;
  uint8_T Stop_sta;
  uint8_T control_enable_sta;
  uint8_T torque_set_sta;
  uint8_T discharge_control_Sta[2];
  uint8_T speed_set_Sta;
  uint8_T speed_read_Sta;
  uint8_T read_active_Curr[2];
  uint8_T motor_direction_Sta;
  uint8_T control_mode_Sta;
  uint8_T Idf_parameter_init_Sta;
  uint8_T Idf_parameter_init_Buf;
  uint8_T analog_Sta[2];
  uint8_T curr_read_offset_Sta[2];
  uint8_T curr_mode_Sta;
  uint8_T motor_parameter_Sta[2];
  uint8_T filter_Sta[2];
  uint8_T zero_position_Sta;
  uint8_T comprehensive_enable_Sta;
  uint8_T read_enable_Sta;
  uint8_T driver_status_Sta;
  uint8_T save_sta;
  uint8_T sys_Info_Sta[2];
  uint8_T driver_Info_Sta[2];
  uint8_T Internal_Data_sta[2];
  uint8_T Comm_Sta[2];
  uint8_T recorder_config_Sta[2];
  uint8_T recorder_state_Sta;
  uint8_T recorder_read_sta;
  uint8_T PI_adjust_Sta[4];
  uint8_T PI_lmit_Sta[2];
  uint8_T fault_clear_Sta;
  uint8_T fault_read_Sta;
  uint8_T protection_threshold_Sta[2];
  uint8_T jump_Fb_Sta;
  uint8_T error_Sta;
  uint8_T expert_read_Sta;
  uint8_T expert_sta;
  uint8_T error_up_sta[10];
} Comm_Feedback_State;

/* 通讯端口参数 */
typedef struct {
  uint16_T Node_ID;
  uint8_T  CAN_baud_rate;
  uint16_T CAN_baud_rate_flag;
  uint8_T  UART_baud_rate;
  uint8_T  UART_UART_check;
  uint16_T P_heartbeat;
  uint16_T C_heartbeat;
  uint16_T P_heartbeat_Per;
  uint16_T C_heartbeat_Per;
  uint16_T C_heartbeat_Ack;
  uint16_T C_heartbeat_Timeout;
  uint16_T C_node_ID;
  uint16_T TPDOx_Per;
  uint16_T TPDOx_Cnt;
  uint16_T TPDO1_Enable;
  uint16_T TPDO3_Enable;
  uint16_T TPDO4_Enable;
  uint16_T TPDO1_Load;
  uint16_T TPDO3_Load;
  uint16_T TPDO4_Load;
  uint8_T TPDO1_Tigger;
  uint8_T TPDO3_Tigger;
  uint8_T TPDO4_Tigger;

} Comm_Parameter;

/* 故障保护参数 */
typedef struct {
  uint16_T over_Volt_thresh;            /* 过压阈值 */
  uint16_T under_Volt_thresh;           /* 欠压阈值 */
  uint16_T over_Volt_recover;           /* 过压自恢复点 V (恢复档门限, 0/非法时按 阈值-2V) */
  uint16_T under_Volt_recover;          /* 欠压自恢复点 V (恢复档门限, 0/非法时按 阈值+2V) */
  uint16_T Curr_thresh;                 /* 过流阈值 */
  real32_T over_Mos_temp_thresh;        /* MOS过温阈值 */
	real32_T over_Mos_temp_high_thresh;   /* MOS高温阈值 */
	real32_T over_Mos_temp_recover_thresh;/* MOS过温恢复阈值(回滞) */
  uint32_T over_Mos_temp_thresh_Per;    /* MOS过温确认时间 ms */
  real32_T over_motor_temp_thresh;      /* 电机过温阈值 */
	real32_T over_motor_temp_high_thresh; /* 电机高温阈值 */
	real32_T over_motor_temp_recover_thresh; /* 电机过温恢复阈值(回滞) */
  uint32_T over_motor_temp_thresh_Per;  /* 电机过温确认时间 ms */
  uint16_T LRC_thresh;                  /* 堵转电流阈值 */
  uint16_T over_Volt_Per;               /* 过压检测时间 ms */
  uint16_T over_Volt_Cnt;               /* 过压检测计数 ms */
  uint16_T under_Volt_Per;              /* 欠压检测时间 ms */
  uint16_T under_Volt_Cnt;              /* 欠压检测计数 ms */
  uint32_T over_speed_thresh;           /* 超速阈值 RPM */
  uint16_T speed_MAX_Per;               /* 转速最大变化值 RPM */
  uint16_T hearbeat_timeout;            /* 心跳超时阈值 ms */
  uint16_T hearbeat_timeout_Cnt;        /* 心跳超时计数 */
  uint32_T LR_speed_thresh;             /* 堵转转速阈值 */
  uint32_T LR_Per;                      /* 堵转检测时间 ms */
  uint8_T  LR_state;                    /* 堵转使能 0:禁止 1:使能 */
  uint8_T  LR_mode;                     /* 堵转控制模式 1:电流限制 其他:关闭 */
  uint16_T LR_limit_Curr;               /* 堵转限制电流 */
  real32_T fbk_loss_spd_th;             /* 主反馈丢失-速度记忆阈值 RPM */
  uint16_T fbk_loss_fast_Per;           /* 主反馈丢失-编码器死寂确认时间 ms */
  real32_T fbk_loss_curr_th;            /* 主反馈丢失-慢速档电流阈值 A(大电流强灌证据) */
  real32_T phase_lose_act_curr;         /* 缺相-断单相包络噪声底 A(仅抗噪, 判别靠不平衡比) */
  real32_T phase_lose_env_nf;           /* 缺相-包络噪声底扣除 A(死相包络≈0.8σ, 不扣会漏报) */
  real32_T phase_lose_imbal_k;          /* 缺相-不平衡比(最小相包络×K < 最大相) */
  real32_T phase_lose_min_spd;          /* 缺相-断单相最低转速 RPM(0=不限制, 置0会被冻结直流伪像误报) */
  uint16_T phase_lose_set_Per;          /* 缺相-断单相确认时间 ms */
  real32_T phase_lose_err_th;           /* 缺相-推不动判据: 双轴电流环误差阈值 A */
  real32_T phase_lose_vq_margin;        /* 缺相-推不动: 电压超出反电势的余量阈值 V(=8 即 Vq 比反电势高 8V 即算推不动) */
  uint16_T phase_lose_multi_Per;        /* 缺相-断多相确认时间 ms */
  uint16_T fbk_lose_pwm_Per;            /* 主反馈丢失: 自检窗内"无新帧"判定时间 ms(0=关) */
  real32_T fbk_lose_curr_hi;            /* 主反馈丢失: "电流与转速矛盾"的大电流门 A */
  real32_T phase_lose_nocurr;           /* 缺相-断多相"确实没有电流"阈值 A */
  real32_T phase_lose_push_imax;        /* 缺相: 电压证据的"小电流"门 A(=限流10%) */
  real32_T phase_lose_env_hi;           /* 缺相-断多相: 包络上限 A(按限流 10% 取) */
  uint8_T  LR_state_o;                  /* 堵转状态输出 */
  real32_T hardware_temp_upper_limit[2]; /* 硬件温度电路异常上限阈值 0:MOS 1:电机 */
	real32_T hardware_temp_lower_limit[2]; /* 硬件温度电路异常下限阈值 0:MOS 1:电机 */
  uint32_T hardware_temp_thresh_Per[2]; /* 硬件温度异常确认时间 0:MOS 1:电机 */
  uint32_T hardware_temp_thresh_Cnt[2]; /* 硬件温度异常计数 0:MOS 1:电机 */
  real32_T hardware_phase_curr;         /* 相电流采样零漂阈值 */
  real32_T handlebar_voltage[2];        /* 转把电压范围 0:上限 1:下限 */
  uint8_T  fault_clear;                 /* 故障清除 */
  uint8_T  self_check;                  /* 自检状态 */
  uint8_T  power_check;                 /* 上电状态 */
  uint8_T  short_circuit;               /* 短路检测 */
  uint8_T  MCU_flash_state;             /* 固件完整性校验结果: 1=校验不可信(自检据此置 MCU 存储器异常) */
  uint8_T  handlebar_state;             /* 转把状态 */
} Fault_Protection;

/* 电机参数 */
typedef struct {
  uint8_T  motor_type;       /* 电机类型 */
  uint8_T  sensor_type;      /* 传感器类型 0:编码器 1:霍尔  2:旋变 */
  uint8_T  temp_sensor;      /* 温度传感器类型 0:KTY84 1:PT100 2:PT1000 3:KTY83 4:NTC-10K */
  uint16_T PP_num;           /* 电机极对数 */
  uint16_T enc_line_count;    /* 主反馈分辨率: 编码器线数 (默认 1024; 每转计数=线数×4) */
  uint16_T sensor_pole_pairs; /* 保留字段, 不参与计算; 电机极对数统一使用 PP_num */
  uint16_T rated_volt;       /* 额定电压 */
  uint32_T rated_power;      /* 额定功率 */
  uint16_T rated_torque;     /* 额定扭矩 */
  uint32_T rated_speed;      /* 额定转速 */
  uint32_T peak_speed;       /* 峰值转速 */
  uint32_T peak_power;       /* 峰值功率 */
  uint16_T peak_curr;        /* 峰值电流 */
  real32_T Ld_value;         /* d轴电感 H */
  real32_T Lq_value;         /* q轴电感 H */
  real32_T flux_value;       /* 磁链值 Wb(V.S) */
  real32_T EMF_constant;     /* 反电势常数 VLL_Peak/Krpm */
  real32_T Rs_value;         /* 相(线)电阻 欧 */
  real32_T I_max;            /* 弱磁最大电流幅值 A */
  real32_T reduction_ratio;  /* 减速比 */
  int32_T  hall_phase_shift; /* 霍尔相位偏移(12bit 电角度, -2048~2047; A7/03 idx0) */
  real32_T Vdc_square;       /* (Vdc*0.57735)^2 */
	real32_T Vdc_svpwm_buf;    /* (Vdc*0.57735) */
	uint8_T  cooling_method;   /* 冷却方式*/
  uint16_T encoder_reverse;  /* ABZ编码器方向: 0=正向 1=反向(A2/02 idx17, 霍尔不使用) */
} Motor_Parameter;

/* PI 控制参数总线 (移植自 MATLAB 重生成版, 对应 CAN 协议 A1 组) */
typedef struct {
  real32_T speed_Kp;            /* 速度环 Kp */
  real32_T speed_Kp_mul[5];     /* 速度环 Kp 分段 */
  real32_T speed_Ki;            /* 速度环 Ki */
  real32_T speed_Ki_mul[5];     /* 速度环 Ki 分段 */
  real32_T speed_max;           /* 速度环输出限幅 */
  real32_T Id_Kp;               /* Id Kp */
  real32_T Id_Kp_mul[5];        /* Id Kp 分段 */
  real32_T Id_Ki;               /* Id Ki */
  real32_T Id_Ki_mul[5];        /* Id Ki 分段 */
  real32_T Id_max;              /* Id 输出限幅 */
  real32_T Iq_Kp;               /* Iq Kp */
  real32_T Iq_Kp_mul[5];        /* Iq Kp 分段 */
  real32_T Iq_Ki;               /* Iq Ki */
  real32_T Iq_Ki_mul[5];        /* Iq Ki 分段 */
  real32_T Iq_max;              /* Iq 输出限幅 */
  real32_T position_Kp;         /* 位置环 Kp */
  real32_T position_Ki;         /* 位置环 Ki */
  real32_T position_max;        /* 位置环输出限幅 */
  real32_T voltage_Kp;          /* 电压环(弱磁) Kp */
  real32_T voltage_Ki;          /* 电压环(弱磁) Ki */
  real32_T voltage_max;         /* 电压环输出限幅 */
} PI_Parameter_Bus;

/* 系统级别参数 */
typedef struct {
  uint8_T  temp_sensor;
  uint8_T  sensor_type;
  uint16_T bus_curr_max;
  uint16_T out_curr_max;
  uint16_T rated_volt;
  uint16_T feedback_Sta[2];
  uint32_T up_timer;
  uint32_T ID_read;
  uint16_T send_feedback_Sta;
  uint32_T update_timer;
  uint32_T update_date;
  uint16_T jump_Fb_Sta;
  uint16_T jump_Sta;
  uint16_T up_data_Err_Sta;
  uint16_T up_data_Err_flag;
  uint16_T Can_transmit_Flag[2];
  uint16_T Can_transmit_Cnt[2];
  uint16_T Can_connect_Sta[2];
  uint16_T power_delay_Sta;
  uint32_T power_delay_Cnt;
  uint16_T Idf_paramter_init_flag;
  real32_T Idf_limt_Iq;
  uint16_T random_number;
  uint16_T expert_RX_Cnt;
  uint32_T expert_RX_Buf[2];
  uint32_T expert_TX_Buf[2];
  uint16_T Init_flag;
	uint8_T save_status;			/*  存储状态机 */
	uint8_T sync_status;			/*  同步状态 */
} SYS_Parameter;

/* 上传数据 */
typedef struct {
    /* ---- 编号 0~9: 状态与采样数据 ---- */
    real32_T  motor_speed;        /* 0: 电机机械转速 [RPM] */
    real32_T  wheel_speed;        /* 1: 车轮速度 [KM/H] */
    real32_T  mech_angle;         /* 2: 电机机械角度 [度] */
    real32_T  bus_current;        /* 3: 母线电流 [ A] */
    real32_T  bus_voltage;        /* 4: 母线电压 [V] */
    real32_T  phase_u_current;    /* 5: U相电流 [ A] */
    real32_T  phase_v_current;    /* 6: V相电流 [ A] */
    real32_T  phase_w_current;    /* 7: W相电流 [ A] */
    real32_T  motor_temp;         /* 8: 电机温度 [℃] */
    real32_T  mos_temp;           /* 9: MOS(铝基板)温度 [℃]  */
    real32_T  mos_temp2;          /*    控制器温度2 (第二路 MOS 采样) [℃] */
    real32_T  mos_temp3;          /*    控制器温度3 (第三路 MOS 采样) [℃] */

    /* ---- 编号 10~12: 寄存器 ---- */
    uint32_T gpio_status_register;    /* 10: IO状态寄存器 */
    uint32_T fault_register;     /* 11: 故障寄存器 */
    uint32_T func_register;      /* 12: 功能寄存器 */

    /* ---- 编号 13~19: 控制量与反馈 ---- */
    real32_T  id_current;         /* 13: Id电流 [ A] */
    real32_T  iq_current;         /* 14: Iq电流 [ A] */
    real32_T  vd_voltage;         /* 15: Vd电压 [V] */
    real32_T  vq_voltage;         /* 16: Vq电压 [V] */
    real32_T  target_speed;       /* 17: 目标转速 [RPM]  */
    real32_T  torque_calc;        /* 18: 扭矩计算 [N.M]  */
    real32_T  throttle_voltage;   /* 19: 转把电压 [V] */
		real32_T  id_ref_current;     /* 20: Id电流 [ A] */
		real32_T  iq_ref_current;     /* 21: Iq电流 [ A] */
    uint32_T  fbk_debug;        /* 22: 主反馈诊断字: [7:0]来源码(FB_SRC_*) [15:8]A线静默拍(饱和255) [23:16]B线静默拍 [27:24]B线沿数低4位 [31:28]A线沿数低4位 */
    volatile uint8_T    ready;    /* 新数据就绪标志 */
} Up_Data_Parameter;

#endif                                 /* Host_Contrl_h_ */

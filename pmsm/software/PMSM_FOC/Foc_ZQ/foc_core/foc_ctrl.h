/******************************************************************************
 * 文件: foc_ctrl.h
 * 功能: FOC 核心控制 API (初始化 / 控制任务 / 清零)
 * 创建日期: 2026-06-22
 * 修改日期: 2026-10-27
 * 修改内容: 快速母线闭环与连续q权限; 母线反馈状态、统一电流圆接口;
 *           调制比例与回构同源可选, 当前为UNITY限流验证配置
 * 修改人: rui.zhang
 ******************************************************************************/

/* FOC 运行数据 FOC_Ctrl_Data / DW 在 foc_data.h 定义, 本头不泄漏(与 foc_config.h 兼容) */

#ifndef FOC_CTRL_H_
#define FOC_CTRL_H_
#include "stdint.h"

/* 电流环前馈 V2 开关: 1=指令值前馈+电压圆余量分配+输出角超前; 0=PI 输出直进 iPark */
#define FOC_FEED_FORWARD_EN    1
/* 输出角超前: 采样->计算->PWM 中点等效 1.5 拍(1 拍 = 100us) */
#define FOC_FF_DELAY_TICKS     1.5F
/* 反馈角时间戳补偿: 电流采样在 CC4 触发时刻(计数中点), 角度戳在 UEV 中断时刻,
   Park 前把角度回退 we*T 到采样时刻(we 带符号+低通)。UEV 相位(峰/谷)未定,
   0.5 拍为两相均值, 台架可按高速单位电流转矩定标 0.25/0.5/0.75 */
#define FOC_FB_DELAY_EN        1
#define FOC_FB_DELAY_TICKS     0.5F
/* ---- 功能编译开关(台架按需裁剪, 改 0/1 重编译) ---- */
/* 低速减速值收敛(过零保护): 干预斜坡的减速度值而非输出 —— 减速档梯度随当前转速
   线性收缩(300rpm 起收敛, 保底 5% 防停死), 惯性平滑落到目标不穿零。输出端钳位
   方案会让速度环与钳位互相拉扯(斩波/咔咔), 已弃用移除 */
#define SPD_DEC_SOFT_EN        0
/* 可达性调速器: 电压不可达、d轴耗尽母线预算或反馈异常时只降低速度参照;
   普通母线/相电流闭环已直接约束q, 不再重复触发速度退让 */
#define PWR_GOV_EN             0
/* 母线计算值滤波: 1=启用; α 越小越强(10kHz: 0.0005≈200ms, 0.002≈50ms) */
#define BUS_CURR_FILT_EN       1
#define BUS_CURR_FILT_ALPHA    0.0005F
/* dq 电流反馈滤波: 默认关闭, 会引入相位滞后; 需要时再开 */
#define CURR_FB_FILT_EN        0
#define CURR_FB_FILT_ALPHA     0.5F
/* MTPA 后快速母线反算闭环 Is_cap */
#define PWR_CL_FAST_ALPHA      0.05F
/* KP/INTEGRAL 按 10kHz 每拍口径整定, 改 PWM/FOC 调用频率须等比重整 */
#define PWR_CL_KP              0.02F
#define PWR_CL_INTEGRAL        0.002F
#define PWR_CL_RELEASE         0.001F
/* 单拍 Is_cap 收紧上限(20A/ms); q 指令另经连续权限, 不直接跟随平方根边界跳变 */
#define PWR_CL_MAX_SLEW        2.0F
/* 分级释放: 母线低于限值超过门槛时快速回升(τ≈10ms), 近限值仍用慢放(τ≈100ms)防 hunt */
#define PWR_CL_FAST_REL_TH     10.0F
#define PWR_CL_FAST_REL        0.01F
#define PWR_CL_DEADBAND        2.0F
#define PWR_CL_MIN_CAP         5.0F
#define PWR_LIMIT_BUS_SCALE    1.0F
/* 限制动作连续化: 收紧约20ms、恢复约50ms; 非转矩/方向退出用约1ms衰减 */
#define FOC_Q_LIMIT_ATTACK     0.005F
#define FOC_Q_LIMIT_RELEASE    0.002F
#define FOC_Q_PASSIVE_DECAY    0.90F
/* 母线反算状态: 启动等待1ms有效样本, 异常值不进入滤波器 */
#define BUS_FB_STARTUP         0U
#define BUS_FB_VALID           1U
#define BUS_FB_FAULT           2U
#define BUS_FB_RECOVER_SAMPLES 10U
/* 母线功率限制总开关: 最终参考端单层快速闭环, 不使用电机参数或饱和表 */
#define BUS_PWR_GUARD_EN       1
/* 电流环饱和增益调度: 生效增益 = 基础增益 x L_sat(id,iq)/L_nominal, 查既有 6x6 饱和表
   (电机适配本就配置该表, 不新增整定量)。电感深饱和后固定增益=带宽随 1/L 上升 -> 极限环
   (台架数据10: |IqRef|>250A 后 Iq 实际 ±100A 振荡, 相电流顶过流阈值锁存断输出)。
   只降不升; GAIN_MIN 为调度下限, 防极端饱和点增益归零 */
#define CURR_PI_SAT_SCHED_EN   1
#define CURR_PI_SAT_GAIN_MIN   0.30F
/* 调度指数(增量电感修正): 饱和表是弦电感(ψ/i), 电流环增益跟增量电感(dψ/di)走,
   深饱和增量<弦(Froelich 平方律)。1=按弦调度(数据12: 稳定到 ~320A), 2=平方修正(默认) */
#define CURR_PI_SAT_EXP        2.0F
/* 最终参考端统一电流圆、d轴电压可行性投影和电压椭圆, q指令连续收放 */
#define VOLT_ELLIPSE_CLAMP_EN  1
/* 1=允许按可达性连续收紧Iq, 0=只置限制状态和速度退让 */
#define VOLT_ELLIPSE_Q_CUT_EN  0
/* 调制电压比例: 输出归一化与CCR回构必须同源。
   ORIGINAL=修改前比例km=Vbus/√3; UNITY=相电压1:1比例km=Vbus/2。
   当前为UNITY限流台架验证配置, ORIGINAL保留作回退。 */
#define FOC_MOD_SCALE_ORIGINAL 0
#define FOC_MOD_SCALE_UNITY    1

#ifndef FOC_MOD_SCALE_MODE
#define FOC_MOD_SCALE_MODE     FOC_MOD_SCALE_UNITY
#endif
#if (FOC_MOD_SCALE_MODE != FOC_MOD_SCALE_ORIGINAL) && \
    (FOC_MOD_SCALE_MODE != FOC_MOD_SCALE_UNITY)
#error "FOC_MOD_SCALE_MODE must be FOC_MOD_SCALE_ORIGINAL or FOC_MOD_SCALE_UNITY"
#endif
/* 低压归一化下限随比例同步; 欠压时输出自然欠调制, 由B11欠压保护兜底。 */
#if (FOC_MOD_SCALE_MODE == FOC_MOD_SCALE_UNITY)
#define SVPWM_KM_FLOOR         18.0F
#else
#define SVPWM_KM_FLOOR         20.8F
#endif
/* 实时 Ld/Lq 估算滤波(10kHz): 先快后慢两级 LPF + 单点尖峰限幅;
   快级 τ≈10ms, 慢级 τ≈50ms; 仅诊断输出, 不进控制/表。 */
#define LDLQ_EST_FAST_ALPHA    0.01F
#define LDLQ_EST_SLOW_ALPHA    0.002F
#define LDLQ_EST_OUTLIER_FRAC  0.20F
/* 弱磁环按 10kHz 调用, pi_voltage.Ki 与此口径绑定; 改调用率须同步折算 Ki */


extern uint32_t volatile ISR_cnt;
extern volatile float g_bus_curr_fast;
extern volatile uint8_t g_bus_curr_status;
/* 参数校验拒绝闭环启动时置1; 有效参数再次进入控制任务后清零 */
extern volatile uint8_t g_foc_param_invalid;
void BusCurrFast_Reset(void);
float FOC_CurrentCircle(void);
void FOC_MotorParams_Poll(void);

/* FOC 初始化 (DW 默认值 + 清零), 在 PI_Pare_init() 之后调用 */
void FOC_Ctrl_Init(void);
/* 实时 Ld/Lq 估算(仅诊断, 不改控制/表): 稳态方程 + LPF, 结果写 DW.motor_anal */
void FOC_LdLq_Estimate(void);

/* 调制系数随额定电压派生; rated_volt 变更(CAN 写/EEPROM 装载)后调用 */
void FOC_UpdateSvpwmFactor(void);

/* FOC 控制任务, 在 10kHz PWM/ADC 中断中调用 */
void FOC_Ctrl_Task(void);

/* FOC 清零 (停止控制时调用) */
void FOC_Ctrl_Clear(void);

/* 停止 FOC: 关闭 PWM 输出, 占空比清零, 清 PI, 速度环斜坡计数清零 */
void motor_stop(void);

/* 启动电机: 三相占空比预置 40% 后开启 PWM 输出 */
void motor_start(void);

/* 输出限幅: 位于MTPA/弱磁之前, 仅执行电流圆和温度降额, 回馈功率由VCU管理 */
float Pwr_Limit_Curr(float iq_dem);


/**********foc globals************/
/*系统辨识子函数*/
void system_identification(void);
//上传更新函数
void updata_display(void);

//状态寄存器写更新载入读取
uint32_t state_reg_func(void);
//GPIO寄存器写更新载入读取
uint32_t gpio_handle(void);
//固化逻辑执行
void strore_handle(void);
//紧急报文发送
void EMCY_TrySend(uint32_t cob_id);
#endif /* FOC_CTRL_H_ */

/******************************************************************************
 * 文件: fault_protect.c
 * 功能: 故障检测/保护 (过流/短路/过欠压/超速/过温/堵转/缺相/主反馈丢失/转把/自检)
 * 创建日期: 2026-06-22
 * 修改日期: 2026-10-26
 * 修改内容: ISR保留A/B线断/过流/短路快速保护, 完整慢故障任务移至主循环1kHz;
 *           故障位在慢任务尾部合并, 停用fbk_debug诊断打包
 * 修改人: rui.zhang
 ******************************************************************************/

/* 故障位定义见 canopen.h FAULT_BIT_*:
 * B00=MOS高温 B01=电机高温 B03=相电流零漂 B04=MOS温度电路 B05=电机温度电路
 * B06=Flash B07=堵转 B08=过压 B09=欠压 B11=超速 B13=MOS过温 B14=电机过温
 * B15=转把 B16=过流 B17=短路 B18=ROM B19=RAM
 * 可自恢复: 过压/欠压/超速/电机过温/MOS过温/堵转(LR_mode==1)/转把
 * 锁存(仅 fault_clear 或重新上电): 其余; 下电状态由下电类故障位派生 */


#include "foc_data.h"
#include "foc_ctrl.h"
#include <math.h>
#include "fault_protect.h"
#include "Common_Math.h"   /* SinCos_Table: 基波同步解调 */
#include "encoder_foc_sdk.h"   /* 取 ENC_TIM (TIM2) */
#include "encoder_pwm_u.h"   /* g_enc.angle_x10: PWM 绝对角度(独立基准) */
#include "hall_sensor.h"     /* g_hall.invalid_ms: 霍尔无效组合持续时间 */

/* ---- 故障分级掩码 (寄存器表.xlsx 故障状态组) ---- */
#define FAULT_LV1_MASK  (FAULT_BIT_DRIVER_HIGH_TEMP | FAULT_BIT_MOTOR_HIGH_TEMP | \
                         FAULT_BIT_BUS_OFFSET | FAULT_BIT_PHASE_OFFSET |          \
                         FAULT_BIT_DRIVER_TEMP_SENSOR | FAULT_BIT_MOTOR_TEMP_SENSOR)
#define FAULT_LV2_MASK  (FAULT_BIT_BUS_OVERVOLTAGE | FAULT_BIT_BUS_UNDERVOLTAGE | \
                         FAULT_BIT_PHASE_LOSE | FAULT_BIT_OVERSPEED |             \
                         FAULT_BIT_MOTOR_FEEDBACK_LOSS |                          \
                         FAULT_BIT_DRIVER_OVERTEMP | FAULT_BIT_MOTOR_OVERTEMP |   \
                         FAULT_BIT_THROTTLE_FAULT)
#define FAULT_LV3_MASK  (FAULT_BIT_OVERCURRENT | FAULT_BIT_PHASE_SHORT |          \
                         FAULT_BIT_CPU_STACK_OVERFLOW)
												 
/* ---- 超速参数 (本任务 10kHz 调用, 1 拍 = 100us; 参数值按拍折算) ---- */
#define OVSPD_SET_TICKS     10U   /* 置位确认 1ms: 防传感器尖峰误触发 */
#define OVSPD_CLR_TICKS    100U   /* 恢复确认 10ms */
#define OVSPD_HOLD_TICKS   10U  	 /* 最小保持 1ms: 保证 EMCY/TPDO/上位机可见 */

/* ---- 转把故障参数 (本任务 10kHz, 1 拍 = 100us) ---- */
#define HANDLEBAR_FAULT_TICKS (2000U)   /* 置位与清位确认时间(拍) = 200ms */
#define HANDLEBAR_HYST_V      (0.05F)   /* 清位回滞 V: 须回到限值以内 0.05V 才允许清位 */

/* ---- 快慢分层: ISR快速保护与主循环1kHz慢任务, 故障位在慢任务尾部合并 ----
   s_fault_slow 仅由主循环慢任务写; ISR快速故障位在合并前重新读取, 防止覆盖 */
#define FAULT_SLOW_DIV_TICKS 10U      /* 10kHz / 10 = 1kHz */
#define FAULT_SLOW_MASK  (FAULT_BIT_DRIVER_OVERTEMP  | FAULT_BIT_MOTOR_OVERTEMP  | \
                         FAULT_BIT_DRIVER_HIGH_TEMP  | FAULT_BIT_MOTOR_HIGH_TEMP  | \
                         FAULT_BIT_THROTTLE_FAULT    | FAULT_BIT_STALL           | \
                         FAULT_BIT_DRIVER_TEMP_SENSOR | FAULT_BIT_MOTOR_TEMP_SENSOR)

/* ---- 缺相 B1.2 参数与状态 ----
   缺单相判定停用, 基波/包络量仅保留诊断;
   断2~3相/未接: 几乎无电流且施加电压扣除反电势后仍有余量。
   U/W 为实测相, V 由 -(Iu+Iw) 计算; 不使用三相和判据; 锁存后断几相共用位 B10 */

#define PHASE_LOSE_ENV_ALPHA  0.005F   /* 1kHz 采样: 1-(1-5e-4)^10, τ=200ms 不变 */
/* 确认计数漏泄分频: 证据断续时每 N 拍漏 1(而非清零), 需为真占比 >1/N 才涨得上去(0x03=每4拍漏1) */
#define PHASE_LOSE_LEAK_MASK   0x03U
/* 缺相公共武装延时: 等包络建立, 防运行初期的电流环瞬态误入确认 */
#define PHASE_LOSE_ARM_TICKS  (200U * FAULT_TICKS_PER_MS)
/* 直流阻断 τ=0.5s(远慢于电频率, 只吃直流); 每次使能重新等收敛(上一轮直流工作点会让开头包络偏斜) */
#define PHASE_LOSE_DC_K        2.0e-3F  /* 1kHz 采样: 1-(1-2e-4)^10, τ=0.5s 不变 */
#define PHASE_LOSE_DC_SETTLE   (1000U * FAULT_TICKS_PER_MS)
#define FAULT_TICKS_PER_MS    10U      /* 本任务 10kHz(中央对齐 UEV 2×PWM), ms→拍 */
static volatile uint32_t s_phase_lose_cnt  = 0U;   /* 断单相确认计数 (32位: 确认时间可调至数秒不溢出) */
static volatile uint32_t s_phase_multi_cnt = 0U;   /* 断多相确认计数 */
static volatile real32_T s_phase_env[3] = { 0.0F, 0.0F, 0.0F };  /* U/V/W 相电流包络 (A), volatile 便于调试观察 */
/* 相电流基波幅值(同步解调, 用实时电角度): 旋转档断相判别用 */
static volatile real32_T s_pl_fre[3]  = { 0.0F, 0.0F, 0.0F };  /* 基波同相分量 */
static volatile real32_T s_pl_fim[3]  = { 0.0F, 0.0F, 0.0F };  /* 基波正交分量 */
static volatile real32_T s_pl_famp[3] = { 0.0F, 0.0F, 0.0F };  /* 基波幅值(峰值 A) */
static volatile real32_T s_pl_fmax = 0.0F;                    /* 基波最大相 */
static volatile real32_T s_pl_fmin = 0.0F;                    /* 基波最小相 */
static volatile uint32_T s_pl_inhibit = 0U;                  /* 缺相抑制剩余拍数(换向/低速过渡) */
static volatile uint32_T s_pl_fund_cnt = 0U;                /* 基波档漏泄确认拍数 */
static volatile uint8_T  s_pl_above = 0U;                  /* 转速曾在抑制门之上(边沿触发用) */

/* ---- 缺相判据中间证据 (static volatile, 可用调试器 watch) ---- */
static volatile real32_T s_pl_err_d     = 0.0F;  /* Ref_d-Fbk_d (A) 带符号 */
static volatile real32_T s_pl_err_q     = 0.0F;  /* Ref_q-Fbk_q (A) 带符号 */
static volatile real32_T s_pl_vmag2     = 0.0F;  /* Vd^2+Vq^2 */
static volatile uint8_T  s_pl_pi_active = 0U;    /* 速度/转矩/非标定 Idq 闭环 */
static volatile uint8_T  s_pl_push_fail = 0U;    /* 电流误差或低电流电压证据 */
static volatile uint8_T  s_pl_imbal     = 0U;    /* 三相包络不平衡 */
/* 包络辅助档要求同一相持续为最小 */
#define PHASE_LOSE_HOLD_TICKS (200U * FAULT_TICKS_PER_MS)
#define PHASE_LOSE_ALIVE_ENV  0.30F   /* 包络辅助档: 最大相包络须 >= 0.30A */
#define PHASE_LOSE_DEAD_ENV   0.80F   /* 包络辅助档: 最小相包络须 < 0.80A */
#define PHASE_LOSE_FUND_ALPHA  0.005F   /* 1kHz 采样: 同步解调对混叠免疫(信号与参考同时刻采样) */
#define PHASE_LOSE_FUND_ALIVE  1.50F   /* 基波幅值门 A: 轻载/噪声下基波很小, 抬高免得比值凑数 */
#define PHASE_LOSE_FUND_K      5.0F    /* 基波档专用不平衡比: 比包络档(imbal_k)更严, 过零/偏置伪像约 2~3 */
#define PHASE_LOSE_FUND_TICKS  (300U * FAULT_TICKS_PER_MS)  /* 基波档确认 300ms: 过渡态证据不可信 */
/* 换向/过渡抑制: |转速| 低于此值时暂停缺相判定(相电流会合法塌陷), 回升后再静默一段时间 */
#define PHASE_LOSE_ARM_SPD      50.0F     /* 抑制门: 换向/停车必经此值; 必须低于 phase_lose_min_spd 免挡住低速判定 */
#define PHASE_LOSE_INHIB_TICKS  (500U * FAULT_TICKS_PER_MS)  /* 速度回升后再静默 500ms(包络/解调器约 200ms 才收敛) */
#define PHASE_LOSE_SLOW_SPD  200.0F  /* 多相电压类证据只在中低速认可 */
#define PHASE_LOSE_MOTION_CNT 20      /* 运动证据基准: 50ms 内编码器净变化 >= 20 count 
                                         (4096 计数/圈时). 已改为按运行期分辨率换算, 见
                                         PhaseLose_MotionCnt() —— 默认参数下与原值等价 */
#define PHASE_LOSE_MOTION_HOLD 5U     /* 运动证据保持窗口数(5*50ms), 免抖动翻转 */
/* 运动检测参数 */
#define FBK_MOTION_TICKS     (50U * FAULT_TICKS_PER_MS)   /* 运动检测窗口 50ms */
#define FBK_MOTION_DEG_MIN   300    /* PWM 角 50ms 内变化 >=30.0° 视为在转 */

/* ---- 主传感器丢失之一: PWM 绝对位置数据失效 ----
   开窗后 frames/edges 均不增长才累计; 校零静默、关窗及计数重锚定见 Fp_FbkWatch */
#define FBK_LOSE_PWM_CNT_MAX   (2000U)   /* '无活动'计数饱和值(200ms), 判定门限见 fbk_lose_pwm_Per */

/* 找零/校零静默窗与辨识状态判别 */
#define FB_IDF_STATE_RESULT_MIN  100U   /* 辨识状态 >=100 = 已出结果(OK=100/FIRL=101) */
#define FB_QUIET_TICKS  (300U * FAULT_TICKS_PER_MS)  /* 找零/校零期间及其后 300ms 不判主反馈(校准会改写零位偏移) */
#define FBK_POSCHK_PWM_MIN 300     /* 窗口内 PWM 角变化 >=30.0deg 视为在动(0.1deg) */
#define FBK_POSCHK_CNT_MAX 10      /* 窗口内 A/B 净变化 <=10 count 视为没动(健康在转时远超此值) */
#define FBK_POSCHK_NZ_MIN  10U    /* 窗口内角度至少更新 10 拍(≈10ms)才算"持续在动", 挡单帧毛刺 */
#define FBK_POSCHK_CONF_WIN 2U    /* 位置不一致: 连续两个窗口都成立才判(解码毛刺瞬时, 真断线持续) */
/* ---- 过压/欠压自恢复门限: 参数为 0 或不在阈值正确一侧时回退到 2V 回滞 ----
   这样 CAN 未配置(或写成非法值)时行为与旧版一致, 不会因为参数是 0 就恒判"已恢复"。 */
static real32_T BusOvRec_Thresh(void)
{
    real32_T r = (real32_T)DW.faule_protection.over_Volt_recover;
    real32_T t = (real32_T)DW.faule_protection.over_Volt_thresh;
    if ((r <= 0.0F) || (r >= t)) { r = t - 2.0F; }
    return r;
}
static real32_T BusUvRec_Thresh(void)
{
    real32_T r = (real32_T)DW.faule_protection.under_Volt_recover;
    real32_T t = (real32_T)DW.faule_protection.under_Volt_thresh;
    if ((r <= 0.0F) || (r <= t)) { r = t + 2.0F; }
    return r;
}
/* ---- 过温恢复门限: 恢复点必须严格低于过温阈值 ----
   否则(把过温阈值调到恢复点之下, 如 30℃ 现场把阈值设成 27℃)会出现
   "置位 -> 下一拍就满足恢复 -> 再置位"的 20ms 来回翻转, 故障寄存器反复变化 = 反复上报。
   此时回退到 阈值-5℃, 与过压/欠压自恢复门限同样的处理方式。 */
#define OV_TEMP_HYST  5.0F
static real32_T MosOvRec_Thresh(void)
{
    real32_T r = DW.faule_protection.over_Mos_temp_recover_thresh;
    real32_T t = DW.faule_protection.over_Mos_temp_thresh;
    if ((r >= t) || (r <= -40.0F)) { r = t - OV_TEMP_HYST; }
    return r;
}
static real32_T MotorOvRec_Thresh(void)
{
    real32_T r = DW.faule_protection.over_motor_temp_recover_thresh;
    real32_T t = DW.faule_protection.over_motor_temp_thresh;
    if ((r >= t) || (r <= -40.0F)) { r = t - OV_TEMP_HYST; }
    return r;
}

static volatile uint8_T  s_min_ph    = 0U;   /* 当前包络最小相 (0/1/2) */
static volatile uint16_T s_min_hold  = 0U;
/* 缺相判据中间证据量 */
static volatile uint8_T  s_pl_dead    = 0U;    /* 静止档: 检出死相(某相无交流且无直流) */
static volatile real32_T s_pl_imag    = 0.0F;  /* 推不动判据: 实测 dq 电流幅值 A */
static volatile uint8_T  s_pl_v_evid  = 0U;    /* 电压证据本体(大电压/小电流) */
static uint16_t s_mot_pwm_last = 0U;
static uint16_t s_mot_cnt_last = 0U;
static int32_T  s_mot_pwm_acc  = 0;
static int32_T  s_mot_cnt_acc  = 0;
static uint16_T s_mot_div      = 0U;
static uint16_T s_mot_hold     = 0U;   /* 转子运动证据保持窗口数 */
/* 主传感器丢失 */
static volatile uint32_T s_pwm_lose_cnt   = 0U;   /* 窗口内"无活动"累计拍数 */
static volatile uint32_T s_pwm_frames_last = 0U;  /* 窗口内最后看到的帧计数(g_enc.frames 快照) */
static volatile uint32_T s_pwm_edges_last  = 0U;  /* 窗口内最后看到的边沿计数(g_enc.edges 快照) */
static volatile uint8_T  s_pwm_en_last  = 0U;   /* 上一拍 PWM 解码开关状态 */
/* 位置不一致证据: 只判'有活动/无活动', 不做角度数值比较 */
static uint16_T s_pos_pwm_last = 0U;
static uint16_T s_pos_cnt_last = 0U;
static int32_T  s_pos_pwm_acc  = 0;
static int32_T  s_pos_cnt_acc  = 0;
static uint16_T s_pos_div      = 0U;
static uint8_T  s_pos_en_last  = 0U;   /* 自检窗开关上一拍状态(抓关闭沿) */
static volatile uint8_T s_poschk_hit = 0U;   /* 位置不一致连续命中的窗口数 */

/* ---- 故障来源码: 记录实际置位路径 (static volatile, 上电清零后锁存) ---- */
#define FB_SRC_PWM_LOST   0x0002U   /* PWM 绝对位置数据失效(超时/无数据) */
#define FB_SRC_POS_MISMATCH 0x0008U   /* PWM 绝对角在动 而 A/B 计数不动 */
#define FB_SRC_LINE_A    0x0010U   /* A/B 单线断: 静默线 = TIM2_CH1(PA0/A相) */
#define FB_SRC_LINE_B    0x0020U   /* A/B 单线断: 静默线 = TIM2_CH2(PA1/B相) */
#define FB_SRC_HALL_INVALID 0x0040U   /* 霍尔模式: 无效组合(0/7)持续超时 */
#define PL_SRC_NO_CURR    0x0001U   /* 判据二: 在推却几乎无电流 */
#define PL_SRC_IMBAL      0x0002U   /* 判据一: 有转速 + 相包络不平衡 */
#define PL_SRC_FUND      0x0004U   /* 判据一(基波档): 基波幅值不平衡 */
static volatile uint16_T s_fb_src = 0U;
/* A/B 单线断证据(每拍更新): 各线"距上次边沿"的拍数 + 各线自"对侧线上次边沿"起的边沿次数 */
static volatile uint32_T s_ln_stale[2] = {0U, 0U};   /* 距上次边沿的拍数(100us/拍) */
static volatile uint8_T  s_ln_peer[2]  = {0U, 0U};   /* 自对侧线上次边沿起, 本线的边沿次数 */
static volatile uint16_T s_pl_src = 0U;
/* 电压证据的电流上限: 超过则 I*R 压降即可解释 |V| */
static uint8_t           s_pl_leak_div  = 0U;    /* 漏泄分频: 每 4 拍漏 1, 抗证据抖动 */
/* 转速异常跌落记忆 (拍): 单次更新跌落 >= FBK_LOSS_DROP_TH rpm 记为异常,
   用于区分反馈丢失(有跌落)与堵转(无跌落) */
#define FBK_LOSS_DROP_TH    50U     /* 单次更新最大可信跌落 rpm (机械不可能超过;
                                       ENC_CalcSpeed 为16帧滑窗, 断线时单帧约94rpm) */
#define FBK_DROP_MEM_TICKS  (200U * FAULT_TICKS_PER_MS)   /* 跌落事件记忆 200ms */
static uint16_t          s_fbk_drop_mem = 0U;
/* 转速参考失效诊断: 静止 CNT 不动不直接作为 A/B 断线或缺相置位证据 */
static uint8_t           s_spd_invalid = 0U;
static uint16_T          s_fb_quiet_ticks = 0U;  /* 找零/校准静默窗(拍): >0 时清反馈/缺相确认 */
static uint8_T           s_fb_calib_run   = 0U;  /* 1=零位/参数辨识进行中 */
static uint8_T           s_fb_openloop    = 0U;  /* 1=开环(VF/防盗/高频注入) */
static uint8_T           s_ln_motion_armed = 0U; /* 故障前已具备闭环运动证据, 过流后限时补记线断根因 */
static uint16_T          s_ln_oc_ticks     = 0U; /* 过流后的补记窗口(拍), 复用线边沿 200ms 新鲜度上限 */

/* ---- CPU 堆栈溢出 (栈底哨兵法) ----
   startup_stm32f303xe.s 定义栈底符号 Stack_Mem, 栈 2KB 向下生长;
   在栈底写入魔数, 一旦被覆盖即说明栈深度已逼近上限, 置位三级故障 */
#define STACK_GUARD_MAGIC   0x5A5A5A5AUL
#define STACK_GUARD_WORDS   8U              /* 栈底 32 字节作哨兵 */
#define STACK_GUARD_PERIOD  1000U           /* 校验节拍: 每 1000 拍(100ms)一次 */
static uint16_t s_stack_guard_cnt = 0U;

extern uint32_t Stack_Mem;                  /* startup 中 EXPORT 的栈底地址 */

/* 上电早期调用: 在栈底写入哨兵魔数 */
void Stack_Guard_Init(void)
{
    uint32_t *p = &Stack_Mem;
    uint32_t  i;
    for (i = 0U; i < STACK_GUARD_WORDS; i++) { p[i] = STACK_GUARD_MAGIC; }
}

/* 校验栈底哨兵: 1=已被覆盖(栈接近溢出) 0=完好 */
uint8_t Stack_Guard_Check(void)
{
    const uint32_t *p = &Stack_Mem;
    uint32_t i;
    for (i = 0U; i < STACK_GUARD_WORDS; i++) {
        if (p[i] != STACK_GUARD_MAGIC) { return 1U; }
    }
    return 0U;
}

/* ---- 主反馈丢失 B1.4 参数与状态: 编码器活跃度 + 速度记忆(1s) ---- */
#define SPD_MEM_TICKS         (1000U * FAULT_TICKS_PER_MS)  /* 速度记忆窗口 1s(拍) */

static volatile  uint16_t s_fbk_cnt_last    = 0U;   /* 上拍 CNT 低16位(活跃度比较) */
static volatile  uint32_t s_cnt_still_ticks = 0U;   /* CNT 连续零变化时长(拍) */
static volatile  uint32_t s_spd_mem_tick    = 0U;   /* 距上次 |转速|>=spd_th 的时长(拍) */
static volatile  uint8_T  s_run_last        = 0U;   /* 运行态边沿 */
static uint16_T          s_mode_last       = 0U;   /* 上一拍控制模式, 切换时重建缺相证据 */
static uint32_t          s_run_ticks       = 0U;   /* 入运行态时长(拍), 缺相武装用 */
static volatile real32_T s_dc_u            = 0.0F; /* U 通道 DC 跟踪(每拍无条件更新) */
static volatile real32_T s_dc_w            = 0.0F; /* W 通道 DC 跟踪 */
static volatile uint8_T  s_dc_ready       = 0U;   /* 直流跟踪已收敛 */
static volatile uint8_T  s_fwchk_req      = 0U;   /* 固件完整性校验请求: 自检窗口开始时置位, 主循环取走执行 */
static uint32_t          s_dc_conv_ticks  = 0U;   /* 直流跟踪累计拍数, 运行进入/清故障/零漂校准时与 s_dc_u/w 同清 */

static volatile  uint16_t s_ovspd_hold = 0U;      /* 超速最小保持计数 */
static volatile  int32_t  s_spd_prev    = 0;      /* 上拍转速 (合理性门) */
static volatile  uint8_t  s_spd_prev_ok = 0U;

/* 固件完整性校验请求: 主循环取走(取走后清零), 返回 1 = 需要执行一次校验 */

uint8_t Fault_FwCheckReqTake(void)
{
    if (s_fwchk_req == 0U) { return 0U; }
    s_fwchk_req = 0U;
    return 1U;
}

/* 运动证据门限: 按当前主反馈分辨率换算 (基准 20 count @ 4096 计数/圈)。
   分辨率由 CAN 参数 enc_line_count 决定 -> g_enc_foc.PulseNumber 跟随更新;
   下限 5 count, 防分辨率很小时门限退化成 0 而失去意义。默认 4096 时结果 = 原宏 20。 */
static uint16_T PhaseLose_MotionCnt(void)
{
    uint32_T pn = (uint32_T)g_enc_foc.PulseNumber;
    uint32_T c  = (pn * (uint32_T)PHASE_LOSE_MOTION_CNT) / 4096U;
    if (c < 5U)     { c = 5U; }
    if (c > 0xFFFFU){ c = 0xFFFFU; }
    return (uint16_T)c;
}

static volatile uint32_T s_fault_slow = 0U;      /* 慢分支故障缓存(只由慢分支写, 尾部汇总) */

/* *************************************************************
 * 判据子函数按快慢职责分别由 Fault_FastTask/Fault_Protect_Task 调用,
 * 各自持有确认计数与证据量(file-scope static); 参数名沿用原局部变量名。
 * 快路径(10kHz ISR): A/B线断、过流、短路; 慢路径(主循环1kHz): 其余判据。
 * 汇总: fault_state_buf = (fault_state_buf & ~FAULT_SLOW_MASK) | s_fault_slow。
 * ************************************************************* */


/* 运行态判据(1kHz): 证据层(DC/包络/基波) + 主反馈三判据 + 缺相两判据 + 超速 + 堵转 */
static uint32_T Fp_RunCriteria(uint32_T fault_state_buf, real32_T base_slow)
{
    real32_T u = fabsf(DW.motor_anal.phaseU_Curr);
    real32_T v = fabsf(DW.motor_anal.phaseV_Curr);
    real32_T w = fabsf(DW.motor_anal.phaseW_Curr);
    uint8_T  LR_state_o = DW.faule_protection.LR_state_o;

    /* 霍尔模式: 无效组合(0/7)持续 100ms -> 主反馈丢失(霍尔线断/短路)。
       安排在缺相判据之前, 让同一拍的缺相/堵转看到该位后不再重复置位。
       健康霍尔在任何正常工况下都不会停在 0/7, 且 100ms 远长于换相/上电抖动。 */
    if ((DW.motor_param.sensor_type == SENSOR_TYPE_HALL) &&
        (s_fb_openloop == 0U) && (s_fb_calib_run == 0U) &&
        (DW.comm_contrl.control_mode != (uint16_T)Control_Mode_Idq_Tset) &&
        (g_hall.invalid_ms >= HALL_INVALID_FAULT_MS)) {
        fault_state_buf |= FAULT_BIT_MOTOR_FEEDBACK_LOSS;
        s_fb_src |= FB_SRC_HALL_INVALID;
    }

				/* **** 3.0 共享证据层: 电流包络 + 编码器活跃度 + 速度记忆 (缺相/反馈丢失/堵转共用) **** */
					{
					    real32_T env_max, env_min;
					    float    spdf = (float)DW.motor_main_feedback.mech_Spd;
    real32_T aspd    = fabsf(spdf);   /* |转速| 一次算好 */
    real32_T curr_th = DW.faule_protection.fbk_loss_curr_th;
    real32_T imbal_k = DW.faule_protection.phase_lose_imbal_k;
					    uint16_T cnt_now;

					    /* 电流包络 (进入运行态时已清零, 免滞留旧值) */
					    if (s_run_ticks < PHASE_LOSE_ARM_TICKS) { s_run_ticks += FAULT_SLOW_DIV_TICKS; }
    real32_T uc, vc, wc;
    /* 直流阻断: 每拍无条件更新 U/W 通道 DC 并扣除, V 用扣除后的 U/W 重算; τ=0.5s */
					    real32_T iu_raw = DW.motor_anal.phaseU_Curr;
					    real32_T iw_raw = DW.motor_anal.phaseW_Curr;
					    if (s_dc_conv_ticks < PHASE_LOSE_DC_SETTLE) {
					    	s_dc_conv_ticks += FAULT_SLOW_DIV_TICKS;
					    	if (s_dc_conv_ticks >= PHASE_LOSE_DC_SETTLE) { s_dc_ready = 1U; }
					    }
					    s_dc_u += PHASE_LOSE_DC_K * (iu_raw - s_dc_u);
					    s_dc_w += PHASE_LOSE_DC_K * (iw_raw - s_dc_w);
					    uc = DW.motor_anal.phaseU_Curr - s_dc_u;
					    wc = DW.motor_anal.phaseW_Curr - s_dc_w;
					    vc = -(uc + wc);
					    s_phase_env[0] += PHASE_LOSE_ENV_ALPHA * (fabsf(uc) - s_phase_env[0]);
					    s_phase_env[1] += PHASE_LOSE_ENV_ALPHA * (fabsf(vc) - s_phase_env[1]);
					    s_phase_env[2] += PHASE_LOSE_ENV_ALPHA * (fabsf(wc) - s_phase_env[2]);
					    
					    /* ---- 相电流基波幅值(同步解调): 包络是宽带量, 软件计算相 V=-(U+W) 把两个实测通道
					       的采样干扰(串扰/增益失配)加在一起 -> 断 V 相时包络仍剩 1.3A(比值 1.9, 过不了 3:1);
					       基波只认与电角度同步的分量, 断相时最小相幅值直接塌到干扰电平。
					       参考相位只影响旋转, 不影响幅值, 三相差 120°(=4096/3)即可 */
					    {
					        Ang_SinCos scf;
					        real32_T   cf[3], sf[3], ph[3];
					        uint16_T   a_ = DW.motor_main_feedback.elec_angle & 0x0FFFU;
					        uint8_T    k_;
					        scf.table_Angle = a_;                                  SinCos_Table(&scf); cf[0] = scf.table_Cos; sf[0] = scf.table_Sin;
					        scf.table_Angle = (uint16_T)((a_ + 1365U) & 0x0FFFU);  SinCos_Table(&scf); cf[1] = scf.table_Cos; sf[1] = scf.table_Sin;
					        scf.table_Angle = (uint16_T)((a_ + 2731U) & 0x0FFFU);  SinCos_Table(&scf); cf[2] = scf.table_Cos; sf[2] = scf.table_Sin;
					        /* 辨识/找零期间电角度被强制(Idq_Tset 下 elec_angle=0), 冻结解调器免得状态卷绕 */
					        if ((DW.adc_slmple.zero_flag == 1U) && (s_fb_calib_run == 0U) &&
						            (s_fb_openloop == 0U) && (s_fb_quiet_ticks == 0U) &&
						            (s_pl_inhibit == 0U) && (aspd >= PHASE_LOSE_ARM_SPD)) {
    					        ph[0] = uc; ph[1] = vc; ph[2] = wc;
    					        for (k_ = 0U; k_ < 3U; k_++) {
    					            s_pl_fre[k_] += PHASE_LOSE_FUND_ALPHA * (ph[k_] * cf[k_] - s_pl_fre[k_]);
    					            s_pl_fim[k_] += PHASE_LOSE_FUND_ALPHA * (ph[k_] * sf[k_] - s_pl_fim[k_]);
    					            s_pl_famp[k_] = 2.0F * sqrtf(s_pl_fre[k_] * s_pl_fre[k_] + s_pl_fim[k_] * s_pl_fim[k_]);
    					        }
					        }
					        s_pl_fmax = s_pl_famp[0];
					        s_pl_fmin = s_pl_famp[0];
					        for (k_ = 1U; k_ < 3U; k_++) {
					            if (s_pl_famp[k_] > s_pl_fmax) { s_pl_fmax = s_pl_famp[k_]; }
					            if (s_pl_famp[k_] < s_pl_fmin) { s_pl_fmin = s_pl_famp[k_]; }
					        }
					    }
							/* ---- 换向/过渡抑制(边沿触发, 会自行到期) ----
								 只在"转速曾高于门限、随后掉下来"时武装一次, 静默 500ms 后自动重新武装:
								 换向/急减速时相电流会合法塌到近 0(包络三路塌陷速度不同、基波解调在频率过零处退化成零偏),
								 过渡期可能凑出 >3 的比值。必须边沿触发 —— 若每拍续期, 静止/堵转工况(转速从未上过门限)
								 会把判据一/判据二/死相档永久挡住, 连"不接电机推不动"都报不出来 */
							if (aspd >= PHASE_LOSE_ARM_SPD) {
									s_pl_above = 1U;                        /* 曾经在门限以上 */
							} else if (s_pl_above != 0U) {
									s_pl_above   = 0U;                      /* 刚掉下来: 武装一次 */
									s_pl_inhibit = PHASE_LOSE_INHIB_TICKS;
							}
							if (s_pl_inhibit > 0U) {
									s_pl_inhibit = (s_pl_inhibit > FAULT_SLOW_DIV_TICKS) ?
																 (s_pl_inhibit - FAULT_SLOW_DIV_TICKS) : 0U;
									s_phase_lose_cnt  = 0U;
									s_phase_multi_cnt = 0U;
									s_min_hold        = 0U;
									s_pl_dead         = 0U;
									s_pl_fund_cnt     = 0U;
									s_pl_fre[0] = s_pl_fre[1] = s_pl_fre[2] = 0.0F;
									s_pl_fim[0] = s_pl_fim[1] = s_pl_fim[2] = 0.0F;
									s_pl_famp[0] = s_pl_famp[1] = s_pl_famp[2] = 0.0F;
									s_pl_fmax = 0.0F; s_pl_fmin = 0.0F;
							}
					    /* 扣除噪声底 phase_lose_env_nf (断相包络被采样噪声抬到 ≈0.79σ) */
					    {
					        real32_T nf = DW.faule_protection.phase_lose_env_nf;
					        real32_T e0 = s_phase_env[0] - nf;
					        real32_T e1 = s_phase_env[1] - nf;
					        real32_T e2 = s_phase_env[2] - nf;
					        if (e0 < 0.0F) { e0 = 0.0F; }
					        if (e1 < 0.0F) { e1 = 0.0F; }
					        if (e2 < 0.0F) { e2 = 0.0F; }
					        {
					            /* 最小相持续性跟踪 (见宏区说明): 真断相 = 同一相恒为最小 */
					            uint8_T ph = ((e1 < e0) && (e1 <= e2)) ? 1U
					                       : (((e2 < e0) && (e2 < e1)) ? 2U : 0U);
					            if (ph == s_min_ph) { if (s_min_hold < PHASE_LOSE_HOLD_TICKS) { s_min_hold += FAULT_SLOW_DIV_TICKS; } }
					            else { s_min_ph = ph; s_min_hold = 0U; }
					        }
					        env_max = e0;
					        if (e1 > env_max) { env_max = e1; }
					        if (e2 > env_max) { env_max = e2; }
					        env_min = e0;
					        if (e1 < env_min) { env_min = e1; }
					        if (e2 < env_min) { env_min = e2; }
					    }

					    /* 编码器活跃度: CNT 连续零变化时长 (连接的编码器静止也有±1抖动) */
					    cnt_now = (uint16_T)(ENC_TIM->CNT);
					    if (cnt_now != s_fbk_cnt_last) {
					        s_fbk_cnt_last    = cnt_now;
					        s_cnt_still_ticks = 0U;
					        s_spd_invalid     = 0U;   /* 编码器又在出数, 转速参考恢复可信 */
					    } else if (s_cnt_still_ticks < 0xFFFFFFF0UL) {
					        s_cnt_still_ticks += FAULT_SLOW_DIV_TICKS;
					        if ((s_cnt_still_ticks >=
					             (uint32_T)DW.faule_protection.fbk_loss_fast_Per * FAULT_TICKS_PER_MS) &&
					            ((s_spd_mem_tick < SPD_MEM_TICKS) || (s_fbk_drop_mem > 0U))) {
					            s_spd_invalid = 1U;   /* 刚从 >=spd_th 转着(速度记忆 1s 内)或刚有损坏级跌落, 现在完全死寂 -> 参考失效(持续) */
					        }
					    }
					    /* ---- 转子运动证据: 50ms 窗口净变化(PWM 角 >=30.0° 或 CNT >=20) -> s_mot_hold ---- */
					    {
					        int16_t dm_ = (int16_t)(g_enc.angle_x10 - s_mot_pwm_last);
					        if (dm_ > 1800) { dm_ -= 3600; } else if (dm_ < -1800) { dm_ += 3600; }
					        s_mot_pwm_last = g_enc.angle_x10;
					        s_mot_pwm_acc += (int32_T)dm_;
					        s_mot_cnt_acc += (int32_T)(int16_t)(cnt_now - s_mot_cnt_last);
					        s_mot_cnt_last = cnt_now;
					        if ((s_mot_div += FAULT_SLOW_DIV_TICKS) >= FBK_MOTION_TICKS) {   /* 50ms 窗不变 */
					            int32_T ap_ = (s_mot_pwm_acc < 0) ? -s_mot_pwm_acc : s_mot_pwm_acc;
					            int32_T ae_ = (s_mot_cnt_acc < 0) ? -s_mot_cnt_acc : s_mot_cnt_acc;
					            if ((ap_ >= FBK_MOTION_DEG_MIN) || (ae_ >= PhaseLose_MotionCnt())) {
					                s_mot_hold = PHASE_LOSE_MOTION_HOLD;
					            } else if (s_mot_hold > 0U) {
					                s_mot_hold--;
					            }
					            s_mot_div = 0U; s_mot_pwm_acc = 0; s_mot_cnt_acc = 0;
					        }
					    }
					    
					    

					    


					    /* 速度记忆: 距上次 |转速|>=spd_th 的时长 */
					    if (aspd >= DW.faule_protection.fbk_loss_spd_th) {
					        s_spd_mem_tick = 0U;
					    } else if (s_spd_mem_tick < 0xFFFFFFF0UL) {
					        s_spd_mem_tick += FAULT_SLOW_DIV_TICKS;
					    }

					    /* **** 3.0a 主反馈丢失 (锁存仅 fault_clear) ****
					    s_cnt_still_ticks/s_spd_mem_tick/s_fbk_drop_mem 供"转速骤死(s_spd_invalid)"判定作前提 */

					    /* 零漂标定窗口(zero_flag==0)内相电流为冻结值, 计数与包络全清, 重启后重新建立 */
					    if (DW.adc_slmple.zero_flag == 0U) {
					        s_phase_lose_cnt  = 0U;
					        s_phase_multi_cnt = 0U;
					        s_phase_env[0]    = 0.0F;
					        s_phase_env[1]    = 0.0F;
					        s_phase_env[2]    = 0.0F;
					        s_min_hold       = 0U;   /* 包络清零期间 argmin 无意义 */
						        s_min_ph         = 0U;
						        s_pl_fund_cnt    = 0U;
						        s_pl_fre[0] = s_pl_fre[1] = s_pl_fre[2] = 0.0F;
						        s_pl_fim[0] = s_pl_fim[1] = s_pl_fim[2] = 0.0F;
						        s_pl_famp[0] = s_pl_famp[1] = s_pl_famp[2] = 0.0F;
						        s_pl_fmax = 0.0F; s_pl_fmin = 0.0F;
						        s_dc_u = 0.0F; s_dc_w = 0.0F;
						        s_dc_conv_ticks = 0U; s_dc_ready = 0U;
						        s_pl_inhibit = 0U; s_pl_above = 0U;
						        s_pl_imbal = 0U; s_pl_v_evid = 0U;
					    }
					    
					    /* **** 3.0b 缺相 (锁存; 断几相共用 B10) ****
					    多相判据使用双轴实测电流及 |V|-反电势; 单相判据使用包络与同步基波 */
					    s_pl_pi_active = ((DW.comm_contrl.control_mode == (uint16_T)Control_Mode_Speed) ||
						                      (DW.comm_contrl.control_mode == (uint16_T)Control_Mode_Torq) ||
					                      ((DW.comm_contrl.control_mode == (uint16_T)Control_Mode_Idq) &&
						                       (DW.comm_contrl.zero_identify_ctrl == 0U))) ? 1U : 0U;
					    s_pl_push_fail = 0U;
					    s_pl_imag = 0.0F; s_pl_v_evid = 0U;   /* 每拍清零, 开环/停机不得沿用旧电压证据 */
					    s_pl_err_d = 0.0F;  s_pl_err_q = 0.0F;  s_pl_vmag2 = 0.0F;
					    if (s_pl_pi_active != 0U) {   /* VF/高频注入为开环, pi_* 为残留值, 不可作证据 */
					        s_pl_err_d = DW.pi_ds.Ref - DW.pi_ds.Fbk;
					        s_pl_err_q = DW.pi_qs.Ref - DW.pi_qs.Fbk;
					        s_pl_vmag2 = DW.motor_anal.Vd_buf * DW.motor_anal.Vd_buf +
					                     DW.motor_anal.Vq_buf * DW.motor_anal.Vq_buf;
					                     /* 反电势(该转速下), 轻载高速时 Vq 几乎全是它, 不可当作"推不动" */
					                     real32_T emf_v = fabsf((real32_T)DW.motor_main_feedback.mech_Spd) *
					                                      ((real32_T)DW.motor_param.PP_num * 0.104719755F) *
					                                      DW.motor_param.flux_value;
					        {
					            /* 电压证据: 电流 < phase_lose_push_imax 且 |V|-反电势 >= phase_lose_vq_margin
					               (平方比较, 免开方) */
					            real32_T i_mag   = sqrtf(DW.pi_ds.Fbk * DW.pi_ds.Fbk + DW.pi_qs.Fbk * DW.pi_qs.Fbk);
					            real32_T v_lim   = emf_v + DW.faule_protection.phase_lose_vq_margin;
					            real32_T err_th  = DW.faule_protection.phase_lose_err_th;
					            uint8_T  i_small = (i_mag < DW.faule_protection.phase_lose_push_imax) ? 1U : 0U;
					            uint8_T  v_over  = ((v_lim <= 0.0F) || (s_pl_vmag2 >= (v_lim * v_lim))) ? 1U : 0U;
					            s_pl_imag   = i_mag;   /* 判据二"确实没有电流"用实测 dq 电流幅值 */
					            s_pl_v_evid = ((i_small != 0U) && (v_over != 0U)) ? 1U : 0U;
					            if ((fabsf(s_pl_err_q) >= err_th) ||
					                (fabsf(s_pl_err_d) >= err_th) ||
					                ((i_small != 0U) && (aspd <= PHASE_LOSE_SLOW_SPD) && (v_over != 0U))) {
					                s_pl_push_fail = 1U;
					            }
					        }
					        }

					    s_pl_imbal = ((env_max >= DW.faule_protection.phase_lose_act_curr) &&
					                  ((env_min * imbal_k) < env_max)) ? 1U : 0U;
					    s_fbk_drop_mem = (s_fbk_drop_mem > FAULT_SLOW_DIV_TICKS) ?
                     (uint16_t)(s_fbk_drop_mem - FAULT_SLOW_DIV_TICKS) : 0U;
					    s_pl_leak_div++;
					    
					    #if 0
/* 缺单相检测停用: 判定代码不参与编译, 基波/包络量仅保留诊断。 */
						    {
						        /* 缺单相检测停用 */
						        real32_T fund_cycles = elec_hz * (real32_T)PHASE_LOSE_FUND_TICKS /
						                                       (1000.0F * (real32_T)FAULT_TICKS_PER_MS);
						        uint8_T common = ((DW.adc_slmple.zero_flag == 1U) &&
						            (s_dc_ready != 0U) &&
						            ((fault_state_buf & (FAULT_BIT_PHASE_LOSE | FAULT_BIT_STALL |
						                                 FAULT_BIT_MOTOR_FEEDBACK_LOSS)) == 0U) &&
						            (fabsf(s_dc_u) < curr_th) && (fabsf(s_dc_w) < curr_th) &&
						            (s_run_ticks >= PHASE_LOSE_ARM_TICKS) &&
						            (s_fb_calib_run == 0U) && (s_fb_openloop == 0U) &&
						            (s_fb_quiet_ticks == 0U) && (s_pl_pi_active != 0U) &&
						            (s_pl_inhibit == 0U) &&
						            (aspd >= DW.faule_protection.phase_lose_min_spd) &&
						            (fund_cycles >= 1.0F)) ? 1U : 0U;   /* 300ms 确认窗至少一个完整电周期 */
						        uint8_T fund_hit = ((common != 0U) &&
						            (s_pl_fmax >= PHASE_LOSE_FUND_ALIVE) &&
						            ((s_pl_fmin * PHASE_LOSE_FUND_K) < s_pl_fmax)) ? 1U : 0U;
						        uint8_T env_hit = ((common != 0U) && (s_pl_imbal != 0U) &&
						            (env_max >= PHASE_LOSE_ALIVE_ENV) &&
						            (env_min < PHASE_LOSE_DEAD_ENV) &&
						            (s_min_hold >= PHASE_LOSE_HOLD_TICKS)) ? 1U : 0U;

						        if (fund_hit != 0U) {
						            if (s_pl_fund_cnt < PHASE_LOSE_FUND_TICKS) {
						                s_pl_fund_cnt += FAULT_SLOW_DIV_TICKS;
						            }
						        } else if ((s_pl_fund_cnt > 0U) &&
						                   ((s_pl_leak_div & PHASE_LOSE_LEAK_MASK) == 0U)) {
						            s_pl_fund_cnt = (s_pl_fund_cnt > FAULT_SLOW_DIV_TICKS) ?
						                            (s_pl_fund_cnt - FAULT_SLOW_DIV_TICKS) : 0U;
						        }
						        if (s_pl_fund_cnt >= PHASE_LOSE_FUND_TICKS) {
						            /* 缺单相不置位 */
						        }

						        if (env_hit != 0U) {
						            uint32_T set_ticks = (uint32_T)DW.faule_protection.phase_lose_set_Per *
						                                 FAULT_TICKS_PER_MS;
						            if (s_phase_lose_cnt < set_ticks) {
						                s_phase_lose_cnt += FAULT_SLOW_DIV_TICKS;
						            }
						            if (0U) {
						                fault_state_buf |= FAULT_BIT_PHASE_LOSE;
						                /* 缺单相不置位 */
						            }
						        } else if ((s_phase_lose_cnt > 0U) &&
						                   ((s_pl_leak_div & PHASE_LOSE_LEAK_MASK) == 0U)) {
						            s_phase_lose_cnt = (s_phase_lose_cnt > FAULT_SLOW_DIV_TICKS) ?
						                               (s_phase_lose_cnt - FAULT_SLOW_DIV_TICKS) : 0U;
						        }
						    }

						    #endif
							    /* 判据二: 无电流却推不动 (确认时间 phase_lose_multi_Per)
					    条件: 标定完成 ∧ 无缺相/主反馈已确诊 ∧ 推不动 ∧ 电流环活跃 ∧ 中低速 ∧
         最大相包络 < phase_lose_env_hi ∧ 实测 dq 电流 < phase_lose_nocurr */
					    if ((DW.adc_slmple.zero_flag == 1U) &&
					        (s_dc_ready != 0U) &&
						        /* 已确诊故障(堵转/主反馈丢失)时不判缺相: 断 A/B 会让角度冻结、三相电流变成
					           不平衡直流, 死相条件也会成立, 否则会双报且 DTC 标签歧义 */
					        ((fault_state_buf & (FAULT_BIT_PHASE_LOSE | FAULT_BIT_STALL |
						                                 FAULT_BIT_MOTOR_FEEDBACK_LOSS)) == 0U) &&
					        (fabsf(s_dc_u) < curr_th) &&
					        (fabsf(s_dc_w) < curr_th) &&   /* 直流大=在灌电流(角度冻结), 相是通的, 不是断相 */
					        (s_run_ticks >= PHASE_LOSE_ARM_TICKS) &&   /* 武装延时: 免包络建立期误判 */
					        /* 静止/低速档"在推却没电流": 只用实测 dq 电流幅值判断, 不用三路直流/环境包络 */
					        (s_fb_calib_run == 0U) && (s_fb_openloop == 0U) &&
						        (s_fb_quiet_ticks == 0U) &&   /* 辨识/校零及其静默窗不判 */
					        (s_pl_pi_active != 0U) &&   /* 只在闭环模式判缺相(VF/高频注入为开环) */
					        (aspd <= PHASE_LOSE_SLOW_SPD) &&   /* 只在中低速生效 */
					        (env_max < DW.faule_protection.phase_lose_env_hi) &&
					        (s_pl_imag < DW.faule_protection.phase_lose_nocurr) &&   /* 机器确实没有电流 */
					        (s_pl_v_evid != 0U) && (s_pl_inhibit == 0U)) {   /* 换向过渡期不判 */
					        /* 断2~3相/未接/采样断路: 无电流反馈但控制器在推 */
					        if (s_phase_multi_cnt <
					            (uint32_T)DW.faule_protection.phase_lose_multi_Per * FAULT_TICKS_PER_MS) {
					            s_phase_multi_cnt += FAULT_SLOW_DIV_TICKS;
					        }
					        if (s_phase_multi_cnt >=
					            (uint32_T)DW.faule_protection.phase_lose_multi_Per * FAULT_TICKS_PER_MS) {
					            fault_state_buf |= FAULT_BIT_PHASE_LOSE;
					            s_pl_src |= PL_SRC_NO_CURR;
					        }
					    } else if ((s_phase_multi_cnt > 0U) && ((s_pl_leak_div & PHASE_LOSE_LEAK_MASK) == 0U)) {
					        s_phase_multi_cnt = (s_phase_multi_cnt > FAULT_SLOW_DIV_TICKS) ?
						                            (s_phase_multi_cnt - FAULT_SLOW_DIV_TICKS) : 0U;
					    }
					    
					}
			if(DW.comm_contrl.control_mode==Control_Mode_Speed||(DW.comm_contrl.control_mode==Control_Mode_Idq&&DW.comm_contrl.zero_identify_ctrl==0)||DW.comm_contrl.control_mode==Control_Mode_Torq)
      {
						/*  3.0b 超速: 取绝对值; 合理性门滤尖峰 + 置位确认 + 最小保持 + 恢复确认 */
							int32_t spd      = DW.motor_main_feedback.mech_Spd;
							uint8_t spd_good = 1U;

						/* 信号层: 小幅超限=量化抖动/加速(仅冻结超速计时);
										大幅跳变=信号损坏(机械不可能), 累计确认后报反馈丢失 */
							if (g_spd_updated != 0U) //速度更新后再执行
							{
								g_spd_updated = 0U;
								if (s_spd_prev_ok != 0U) {
										int32_t d = spd - s_spd_prev;
										if (d <= -(int32_t)FBK_LOSS_DROP_TH) {
										    s_fbk_drop_mem = FBK_DROP_MEM_TICKS;   /* 指示转速异常跌落: 机械不可能 */
										}
										if (d < 0) { d = -d; }
										spd_good = 1U;
									}
								s_spd_prev = spd; 
								if (spd_good != 0U) {s_spd_prev_ok = 1U; }
								if (spd_good != 0U) {
										if (fault_state_buf & FAULT_BIT_OVERSPEED) {
												if (s_ovspd_hold > 0U) {
														s_ovspd_hold = (s_ovspd_hold > FAULT_SLOW_DIV_TICKS) ?
														                   (uint16_T)(s_ovspd_hold - FAULT_SLOW_DIV_TICKS) : 0U;
												} else if (fabsf((real32_T)spd) <=
																	 (real32_T)DW.faule_protection.over_speed_thresh) {
														DW.circuilt_cnt[2] += base_slow;
														if (DW.circuilt_cnt[2] >= OVSPD_CLR_TICKS) {
																DW.circuilt_cnt[2] = 0.0;
																fault_state_buf &= ~FAULT_BIT_OVERSPEED;   /* ← 补这一行 */
														}
												} else {
														DW.circuilt_cnt[2] = 0.0;
												}
										} else if (fabsf((real32_T)spd) >
															 (real32_T)DW.faule_protection.over_speed_thresh) {
												DW.circuilt_cnt[2] += base_slow;
												if (DW.circuilt_cnt[2] >= OVSPD_SET_TICKS) {
														DW.circuilt_cnt[2] = 0.0;
														fault_state_buf |= FAULT_BIT_OVERSPEED;
														s_ovspd_hold = OVSPD_HOLD_TICKS;  /* 置位即进入最小保持 */
												}
										} else {
												DW.circuilt_cnt[2] = 0.0;
										}
								}
						}
				}
    /* ---- 堵转 (LR_state==1 时检测, 运行态): 任一相电流 >= LRC_thresh 且 转速 < LR_speed_thresh
       持续 LR_Per; 缺相/主反馈已确诊时不判; 限流模式下条件消失 100ms 自恢复 ---- */
    if (DW.faule_protection.LR_state == 1U) {
        if ((s_fb_calib_run == 0U) &&   /* 辨识/标定中不判堵转 */
            ((fault_state_buf & (FAULT_BIT_PHASE_LOSE | FAULT_BIT_MOTOR_FEEDBACK_LOSS)) == 0U) &&
            ((u >= DW.faule_protection.LRC_thresh) ||
             (v >= DW.faule_protection.LRC_thresh) ||
             (w >= DW.faule_protection.LRC_thresh)) &&
            (fabsf((real_T)DW.motor_main_feedback.mech_Spd) <
             (real_T)DW.faule_protection.LR_speed_thresh) &&
            (s_mot_hold == 0U)) {   /* 无运动证据才判堵转(50ms 窗净变化置位) */
            DW.LR_Cnt += base_slow;
            if ((LR_state_o == 0U) && ((real_T)DW.LR_Cnt >= 1000.0F)) {
                LR_state_o = 1U;   /* 100ms: 先降频(仅从 0 态进入, 不覆盖 10) */
            }
            if ((real_T)DW.LR_Cnt >= (real_T)DW.faule_protection.LR_Per * FAULT_TICKS_PER_MS) {
                DW.LR_Cnt = 0.0F;
                s_fault_slow |= FAULT_BIT_STALL;
                if (DW.faule_protection.LR_mode == 1U) { LR_state_o = 10U; }
            }
        } else {
            /* 堵转条件消失: 限流模式下持续 100ms 自恢复, 否则复位计数 */
            if (((s_fault_slow & FAULT_BIT_STALL) != 0U) &&
                (DW.faule_protection.LR_mode == 1U)) {
                DW.LR_Cnt += base_slow;
                if (DW.LR_Cnt >= 1000.0F) {
                    DW.LR_Cnt = 0.0F;
                    s_fault_slow &= ~FAULT_BIT_STALL;
                    LR_state_o = 0U;
                }
            } else {
                DW.LR_Cnt = 0.0F;
                if ((s_fault_slow & FAULT_BIT_STALL) == 0U) {
                    LR_state_o = 0U;
                }
            }
        }
    }

    DW.faule_protection.LR_state_o = LR_state_o;
    return fault_state_buf;
}

/* 主反馈监vision(1kHz): PWM 帧跟踪 + 位置不一致 + 无活动 */
static uint32_T Fp_FbkWatch(uint32_T fault_state_buf)
{
//				/* ---- 主传感器丢失 = PWM 绝对位置数据失效 ----
//				frames 或 edges 任一增长即为线活; 两者均不变才累计, 校零静默与关窗时计数清零 */
//				{
//				    uint8_T en_ = ENC_Pwm_IsEnabled();
//				    if (s_fb_quiet_ticks > 0U) {
//					        s_pwm_lose_cnt = 0U;                     /* 校零静默: 本窗无活动证据作废并重新锚定 */
//					        s_pwm_frames_last = g_enc.frames;
//					        s_pwm_edges_last  = g_enc.edges;
//					    } else if (en_ != 0U) {
//				        if (s_pwm_en_last == 0U) {                  /* 窗口开启沿: 记下当前帧计数 */
//				            s_pwm_frames_last = g_enc.frames;
//				            s_pwm_edges_last  = g_enc.edges;
//				            s_pwm_lose_cnt = 0U;
//				        } else if ((g_enc.frames != s_pwm_frames_last) || (g_enc.edges != s_pwm_edges_last)) {   /* 有合法帧或任何边沿 -> 线还活着, 静默清零 */
//				            s_pwm_frames_last = g_enc.frames;
//				            s_pwm_edges_last  = g_enc.edges;
//				            s_pwm_lose_cnt = 0U;
//				        } else if (s_pwm_lose_cnt < FBK_LOSE_PWM_CNT_MAX) {
//				            s_pwm_lose_cnt += FAULT_SLOW_DIV_TICKS;   /* 15ms=15 慢拍, 不变 */
//				        }
//				    } else {
//				        s_pwm_lose_cnt = 0U;                        /* 窗口关闭: 本窗口结论作废 */
//				    }
//				    s_pwm_en_last = en_;
//				}
//				/* ---- 主反馈丢失: 位置不一致(自检窗关闭沿结算) ----
//				窗口内 PWM 角净变化 >=30.0deg 而 A/B 净变化 <=10 count -> 主反馈丢失; 仅使能态结算 */
//				{
//				    uint8_T en2_ = ENC_Pwm_IsEnabled();
//				    int16_t dp_ = (int16_t)(g_enc.angle_x10 - s_pos_pwm_last);
//				    if (dp_ > 1800) { dp_ -= 3600; } else if (dp_ < -1800) { dp_ += 3600; }
//				    s_pos_pwm_last = g_enc.angle_x10;
//				    if (en2_ != 0U) {
//				        if (s_pos_en_last == 0U) {   /* 窗口开启沿: 两侧同时重新锚定, 保证比较的是同一区间 */
//				            s_pos_pwm_acc = 0; s_pos_cnt_acc = 0; s_pos_div = 0U;
//				            s_pos_cnt_last = (uint16_T)(ENC_TIM->CNT);
//				        }
//				        s_pos_pwm_acc += (int32_T)dp_;
//				        if (dp_ != 0) { s_pos_div++; }   /* 角度被更新的拍数 = 持续运动(单帧毛刺只加 1) */
//				        s_pos_cnt_acc += (int32_T)(int16_t)((uint16_T)(ENC_TIM->CNT) - s_pos_cnt_last);
//				        s_pos_cnt_last = (uint16_T)(ENC_TIM->CNT);
//				    } else if (s_pos_en_last != 0U) {   /* 窗口关闭沿: 结算 */
//				        int32_T ap_ = (s_pos_pwm_acc < 0) ? -s_pos_pwm_acc : s_pos_pwm_acc;
//				        int32_T ae_ = (s_pos_cnt_acc < 0) ? -s_pos_cnt_acc : s_pos_cnt_acc;
//				        if ((DW.comm_contrl.control_mode != 0U) && (DW.comm_contrl.control_enable == 1U) &&   /* 停机态不判不锁存 */
//				            ((fault_state_buf & FAULT_PD_MASK) == 0U) &&   /* 下电类故障切断输出后的滑行期不判 */
//				            (s_fb_calib_run == 0U) && (s_fb_openloop == 0U) &&   /* 标定/辨识中、开环模式均不判(见 3.0a 前的说明) */
//				            (s_fb_quiet_ticks == 0U) && (ap_ >= FBK_POSCHK_PWM_MIN) && (ae_ <= FBK_POSCHK_CNT_MAX) &&
//				            (s_pos_div >= FBK_POSCHK_NZ_MIN)) {
//				            if (s_poschk_hit < FBK_POSCHK_CONF_WIN) { s_poschk_hit++; }
//				            if (s_poschk_hit >= FBK_POSCHK_CONF_WIN) {   /* 连续两个窗口都成立才判 */
//				                fault_state_buf |= FAULT_BIT_MOTOR_FEEDBACK_LOSS;
//				                s_fb_src |= FB_SRC_POS_MISMATCH;
//				            }
//				        } else if (s_poschk_hit > 0U) {
//				            s_poschk_hit--;   /* 证据断续: 漏泄不清零 */
//				        }
//				        s_pos_pwm_acc = 0; s_pos_cnt_acc = 0; s_pos_div = 0U;
//				    }
//				    s_pos_en_last = en2_;
//				}

//				/* 主反馈丢失: 窗口内无活动, 一次判定即锁存; 停机/未使能时不判也不锁存 */
//				if ((DW.comm_contrl.control_mode != 0U) &&
//				    (DW.comm_contrl.control_enable == 1U) &&   /* 仅运行态(闭环+使能)判定 */
//				    ((fault_state_buf & FAULT_PD_MASK) == 0U) &&   /* 同上: 输出已被下电类故障切断时不判 */
//				    (s_fb_calib_run == 0U) && (s_fb_openloop == 0U) &&   /* 标定/辨识中、开环模式均不判 */
//				    (s_fb_quiet_ticks == 0U) &&                 /* 校零及其后静默窗不判 */
//					    (DW.faule_protection.fbk_lose_pwm_Per != 0U) &&
//				    (s_pwm_en_last != 0U) &&
//				    (s_pwm_lose_cnt >=
//				     (uint32_T)DW.faule_protection.fbk_lose_pwm_Per * FAULT_TICKS_PER_MS)) {
//				    fault_state_buf |= FAULT_BIT_MOTOR_FEEDBACK_LOSS;
//				    s_fb_src |= FB_SRC_PWM_LOST;
//				}


    return fault_state_buf;
}

/* ---- 主反馈丢失: A/B 单线断(线边沿时间戳, 10kHz 快分支判定) ----
   单线断后该线不再产生边沿, 而 TIM2 输入级仍会为另一线的每个有效边沿置捕获标志(CC1IF/CC2IF
   在编码器模式下照常工作), "一线静默、对侧线照常翻转"就能直接判出断的是哪一线。
   健康正交的两线每走一个计数必然换线, 所以"对侧线连续累积 FBK_LINE_PEER_MIN 次边沿而本线一次
   也没有"对健康信号不可能成立 —— 此条与转速无关; FBK_LINE_STALE_TICKS 只是最小静默时长, 防断线
   瞬间的几次抖动被当成证据。时限 = max(2ms, 对侧线攒够 8 个边沿的时间): 500rpm≈2ms /
   100rpm≈5ms / 30rpm≈16ms / 5rpm≈95ms —— 换相角冻结到电流冲到过流通常要几 ms 以上, 断线先到。
   新鲜度门 FBK_LINE_RECENT_TICKS: 静默线的最后一次边沿须在近 200ms 内 —— 真断线时它是"刚断的",
   而静止转子的原始角度有 ±1 计数抖动, 停在某线阈值附近时只有该线反复越限, peer 无时间上限地累计,
   攒满 8 次就凑齐证据(实测不使能也误报) —— 超过 200ms 仍静默说明只是没在转, 不当断线。
   正常运行要求闭环使能且速度记忆有效; 过流先锁存时, 仅在故障前已有运动武装的
   200ms 补记窗内继续判线边沿。其他下电故障及未武装滑行不判, 不改变既有硬件保护动作。
   两线同时静默 = 停转或两线全断, 本判据不判(物理上不可分)。 */
#define FBK_LINE_STALE_TICKS  (2U * FAULT_TICKS_PER_MS)   /* 静默时长门限(快拍 = 100us): 2ms */
#define FBK_LINE_STALE_MAX    100000U                      /* 静默计时上限(防饱和) */
#define FBK_LINE_RECENT_TICKS (200U * FAULT_TICKS_PER_MS)  /* 断沿新鲜度上限: 静默线最后边沿须在近 200ms 内 */
#define FBK_LINE_PEER_MIN     8U                           /* 静默期内对侧线的边沿次数门限 */

static uint32_T Fp_LineWatch(uint32_T fault_state_buf)
{
    uint8_T lp_ = EncFoc_LinePulse();
    uint8_T i;
    uint8_T line_gate = 0U;

    for (i = 0U; i < 2U; i++) {
        if ((lp_ & (uint8_T)(1U << i)) != 0U) {
            s_ln_stale[i] = 0U;                              /* 本线有边沿: 静默计时清零 */
            if (s_ln_peer[i] < 0xFFU) { s_ln_peer[i]++; }     /* 本线边沿计入本方计数 */
            s_ln_peer[i ^ 1U] = 0U;                          /* 对侧计数从本线这次边沿起重算 */
        } else if (s_ln_stale[i] < FBK_LINE_STALE_MAX) {
            s_ln_stale[i]++;
        }
    }

    if ((s_fb_calib_run != 0U) || (s_fb_openloop != 0U) || (s_fb_quiet_ticks > 0U)) {
        s_ln_motion_armed = 0U;
        s_ln_oc_ticks = 0U;
        return fault_state_buf;
    }
    if (((fault_state_buf & (FAULT_BIT_PHASE_LOSE | FAULT_BIT_MOTOR_FEEDBACK_LOSS)) != 0U) ||
        ((fault_state_buf & (FAULT_PD_MASK & ~FAULT_BIT_OVERCURRENT)) != 0U)) {
        /* 非过流下电及其他根因的滑行期不补记 */
        s_ln_motion_armed = 0U;
        s_ln_oc_ticks = 0U;
        return fault_state_buf;
    }

    if ((fault_state_buf & FAULT_BIT_OVERCURRENT) != 0U) {
        if (s_ln_oc_ticks < FBK_LINE_RECENT_TICKS) { s_ln_oc_ticks++; }
        /* 只用故障前已武装的运动证据, 且补记不超过过流后 200ms */
        if ((s_ln_motion_armed != 0U) && (s_ln_oc_ticks <= FBK_LINE_RECENT_TICKS)) {
            line_gate = 1U;
        }
    } else {
        s_ln_oc_ticks = 0U;
        if ((DW.comm_contrl.control_mode != 0U) &&
            (DW.comm_contrl.control_enable == 1U) &&
            (s_spd_mem_tick < SPD_MEM_TICKS)) {
            s_ln_motion_armed = 1U;
            line_gate = 1U;
        } else {
            /* 未使能/停车后的自然滑行不得继承运动武装 */
            s_ln_motion_armed = 0U;
        }
    }

    if (line_gate != 0U) {
        if ((s_ln_stale[0] >= FBK_LINE_STALE_TICKS) &&
            (s_ln_stale[0] <= FBK_LINE_RECENT_TICKS)) {     /* TIM2_CH1 刚静默(2~200ms) */
            if (s_ln_peer[1] >= FBK_LINE_PEER_MIN) {
                fault_state_buf |= FAULT_BIT_MOTOR_FEEDBACK_LOSS;
                s_fb_src |= FB_SRC_LINE_A;
            }
        } else if ((s_ln_stale[1] >= FBK_LINE_STALE_TICKS) &&
                   (s_ln_stale[1] <= FBK_LINE_RECENT_TICKS)) {  /* TIM2_CH2 刚静默(2~200ms) */
            if (s_ln_peer[0] >= FBK_LINE_PEER_MIN) {
                fault_state_buf |= FAULT_BIT_MOTOR_FEEDBACK_LOSS;
                s_fb_src |= FB_SRC_LINE_B;
            }
        }
    }
    /* 诊断输出停用; 故障判据本身不变 */
    return fault_state_buf;
}

/* 运行态上下文(每拍): 进入运行沿复位证据 + 找零静默窗续期/递减 + 辨识/开环标志 */
static void Fp_RunGate(uint8_T slow_tick)
{
        /* 进入运行态: 复位反馈丢失/转速合理性上下文, 防停机期残留误判 */
        uint8_T run_now = (DW.comm_contrl.control_mode != 0U &&
                                        DW.comm_contrl.control_enable == 1U) ? 1U : 0U;
        if ((run_now != 0U) &&
            ((s_run_last == 0U) || (s_mode_last != DW.comm_contrl.control_mode))) {
            s_fbk_cnt_last  = (uint16_T)(ENC_TIM->CNT);   /* 重新锚定 */
            s_spd_prev_ok   = 0U;                          /* 下拍重新建基线 */
            s_phase_env[0]  = 0.0F;                        /* 缺相包络/基波清零: 免滞留旧值 */
            s_pl_fre[0] = s_pl_fre[1] = s_pl_fre[2] = 0.0F;
            s_pl_fim[0] = s_pl_fim[1] = s_pl_fim[2] = 0.0F;
            s_pl_famp[0] = s_pl_famp[1] = s_pl_famp[2] = 0.0F;
            s_pl_fmax = 0.0F; s_pl_fmin = 0.0F; s_pl_inhibit = 0U; s_pl_fund_cnt = 0U;
            s_phase_env[1]  = 0.0F;
            s_phase_env[2]  = 0.0F;
            s_phase_lose_cnt  = 0U;
            s_phase_multi_cnt = 0U;
            s_cnt_still_ticks = 0U;   /* CNT 重新锚定, 死寂计时重来 */
            s_spd_invalid     = 0U;   /* 重启不得残留上一轮的失效屏蔽(否则静止断相被漏判) */
            s_run_ticks       = 0U;
							s_dc_conv_ticks   = 0U;   /* 每次使能重新等直流收敛 */
            s_dc_ready        = 0U;
            s_fbk_drop_mem    = 0U;
            s_dc_u = 0.0F; s_dc_w = 0.0F;
            s_min_ph = 0U; s_min_hold = 0U; s_pl_above = 0U; s_pl_imbal = 0U;
            s_ln_motion_armed = 0U; s_ln_oc_ticks = 0U;
            s_mot_div = 0U; s_mot_pwm_acc = 0; s_mot_cnt_acc = 0; s_mot_hold = 0U;
            s_mot_pwm_last = g_enc.angle_x10; s_mot_cnt_last = (uint16_T)(ENC_TIM->CNT);
            s_pwm_en_last = 0U; s_pwm_lose_cnt = 0U; s_pwm_frames_last = 0U;
            s_ln_peer[0] = 0U; s_ln_peer[1] = 0U; s_ln_stale[0] = 0U; s_ln_stale[1] = 0U;
            s_pos_pwm_last = g_enc.angle_x10; s_pos_cnt_last = (uint16_T)(ENC_TIM->CNT);
            s_pos_pwm_acc = 0; s_pos_cnt_acc = 0; s_pos_div = 0U;
            s_pos_en_last = 0U;
            s_pwm_en_last = 0U;
            s_pwm_lose_cnt = 0U; s_pwm_frames_last = 0U; s_pwm_edges_last = 0U; s_poschk_hit = 0U;
            s_pl_v_evid = 0U;
            s_spd_mem_tick    = SPD_MEM_TICKS;  /* 启动初期无速度记忆 */
        }
        s_run_last = run_now;
        s_mode_last = DW.comm_contrl.control_mode;

        /* 校零(找零)期间及其后 FB_QUIET_TICKS 内不判主反馈, 并清掉相关证据 */
        if (EncPwmSamp_IsActive() != 0U) {
            s_fb_quiet_ticks = FB_QUIET_TICKS;      /* 校准进行中: 每拍续期 */
        }
        else if ((s_fb_quiet_ticks > 0U) && (slow_tick != 0U)) {   /* 仅慢拍递减(300ms 不变) */
            s_fb_quiet_ticks = (s_fb_quiet_ticks > FAULT_SLOW_DIV_TICKS) ? (uint16_T)(s_fb_quiet_ticks - FAULT_SLOW_DIV_TICKS) : 0U;   /* 1kHz 慢节拍, 300ms 不变 */
        }
        if (s_fb_quiet_ticks > 0U) {
            s_spd_invalid  = 0U;                    /* 不保留"骤死"判决 */
            s_spd_mem_tick = SPD_MEM_TICKS;         /* 不保留"刚在转"的记忆 */
            s_pos_pwm_acc  = 0; s_pos_cnt_acc = 0; s_pos_div = 0U; s_poschk_hit = 0U;
            s_pwm_lose_cnt = 0U;
            s_phase_lose_cnt = 0U; s_phase_multi_cnt = 0U; s_pl_fund_cnt = 0U;
            s_min_hold = 0U; s_pl_v_evid = 0U;
            s_pl_fre[0] = s_pl_fre[1] = s_pl_fre[2] = 0.0F;
            s_pl_fim[0] = s_pl_fim[1] = s_pl_fim[2] = 0.0F;
            s_pl_famp[0] = s_pl_famp[1] = s_pl_famp[2] = 0.0F;
            s_pl_fmax = 0.0F; s_pl_fmin = 0.0F;
            s_ln_motion_armed = 0U; s_ln_oc_ticks = 0U;
        }
        /* 主反馈判据公共前提: 辨识进行中(1..99 或 identify_state!=0)不判; OK/FIRL 不算进行中 */
        s_fb_calib_run =
            (((DW.comm_contrl.zero_identify_state != 0U) &&
              (DW.comm_contrl.zero_identify_state <  FB_IDF_STATE_RESULT_MIN)) ||
             (DW.comm_contrl.identify_state != 0U)) ? 1U : 0U;

             /* 开环模式(VF/FD/高频注入)不判主反馈 */
             s_fb_openloop = ((DW.comm_contrl.control_mode == (uint16_T)Control_Mode_VF) ||
                               (DW.comm_contrl.control_mode == (uint16_T)Control_Mode_FD) ||
                               (DW.comm_contrl.control_mode == (uint16_T)Control_Mode_HighFre)) ? 1U : 0U;
}

/* CPU 堆栈哨兵校验(1kHz, 100ms 节拍) */
static uint32_T Fp_SlowStackGuard(uint32_T fault_state_buf)
{
		/* 2.7 CPU 堆栈溢出: 栈底哨兵被覆盖即置位 (降频校验, 栈生长缓慢) */
		if (s_stack_guard_cnt >= STACK_GUARD_PERIOD) {
				s_stack_guard_cnt = 0U;
				if (Stack_Guard_Check() != 0U) {
						fault_state_buf |= FAULT_BIT_CPU_STACK_OVERFLOW;
				}
		} else {
				s_stack_guard_cnt += FAULT_SLOW_DIV_TICKS;
		}
    return fault_state_buf;
}

/* 转把故障位(1kHz): 随 handlebar_state 置位/清除 */
static uint32_T Fp_SlowThrottleBit(uint32_T fault_state_buf, uint8_T handlebar_state)
{
		/* 2.6 转把故障: 随转把状态置位, 电压恢复正常后自动清除 */
		if (handlebar_state == 1U) {
            s_fault_slow |= FAULT_BIT_THROTTLE_FAULT;
        } else {
            s_fault_slow &= ~FAULT_BIT_THROTTLE_FAULT;
        }
    return fault_state_buf;
}

/* 驱动器(MOS)温度(1kHz): 过温/高温告警/回滞自恢复 */
static uint32_T Fp_SlowTempDriver(uint32_T fault_state_buf, uint8_T det_run, real32_T base_slow)
{
			  /*2.5 MOS温度: 过温保护 + 回滞恢复 (逻辑同电机温度, 用 MOS 自己的确认时间) */
        if (s_fault_slow & FAULT_BIT_DRIVER_OVERTEMP) {
            if (DW.motor_anal.mos_temp <
                MosOvRec_Thresh()) {
                s_fault_slow &= ~FAULT_BIT_DRIVER_OVERTEMP;
                DW.mos_temp_cnt[0] = 0.0;
                DW.mos_temp_cnt[1] = 0.0;
            }
        } else if (det_run != 0U) {
            /* 运行态才做置位/清位判决; 停机/未使能时保持现状 —— 否则被别的下电类故障
               (缺相/主反馈等)停掉后, 下面"温度正常"分支会把已经报出的高温告警清掉 */
            if (DW.motor_anal.mos_temp >
                DW.faule_protection.over_Mos_temp_thresh) {
                /* **** 超过过温阈值: 累加计时 → 触发下电 **** */
                DW.mos_temp_cnt[0] += base_slow;
                DW.mos_temp_cnt[1]  = 0.0;
                if (DW.mos_temp_cnt[0] >=
                    DW.faule_protection.over_Mos_temp_thresh_Per * FAULT_TICKS_PER_MS) {
                    DW.mos_temp_cnt[0] = 0.0;
                    s_fault_slow |= FAULT_BIT_DRIVER_OVERTEMP;
                }
            } else if (DW.motor_anal.mos_temp >
                       DW.faule_protection.over_Mos_temp_high_thresh) {
                /* **** 高温告警 (不下电) **** */
                DW.mos_temp_cnt[1] += base_slow;
                DW.mos_temp_cnt[0]  = 0.0;
                if (DW.mos_temp_cnt[1] >=
                    DW.faule_protection.over_Mos_temp_thresh_Per * FAULT_TICKS_PER_MS) {
                    DW.mos_temp_cnt[1] = 0.0;
                    s_fault_slow |= FAULT_BIT_DRIVER_HIGH_TEMP;
                }
            } else {
                /* **** 温度正常: 清计数 + 清高温告警位 **** */
                DW.mos_temp_cnt[0] = 0.0;
                DW.mos_temp_cnt[1] = 0.0;
                s_fault_slow &= ~FAULT_BIT_DRIVER_HIGH_TEMP;
            }
        } else {
            /* 停机/未使能: 不置位; 只有温度确实回到告警阈值以下才清位(恢复照常评估) */
            if (DW.motor_anal.mos_temp <= DW.faule_protection.over_Mos_temp_high_thresh) {
                DW.mos_temp_cnt[0] = 0.0;
                DW.mos_temp_cnt[1] = 0.0;
                s_fault_slow &= ~FAULT_BIT_DRIVER_HIGH_TEMP;
            }
        }
    return fault_state_buf;
}

/* 电机温度(1kHz): 过温/高温告警/回滞自恢复 */
static uint32_T Fp_SlowTempMotor(uint32_T fault_state_buf, uint8_T det_run, real32_T base_slow)
{
				/* 2.4 电机温度: 过温保护 + 回滞恢复 (停机态也评估, 保证可自恢复)
           过温触发后, 须降到独立恢复阈值 recover_thresh 以下才解除 (回滞),
           与高温告警阈值解耦, 防止反复启停 */
				if (s_fault_slow & FAULT_BIT_MOTOR_OVERTEMP) {
            /* **** 已过温: 等待回滞恢复 **** */
            if (DW.motor_anal.motor_temp <
                MotorOvRec_Thresh()) {
                /* 降到恢复阈值以下: 解除过温 */
                s_fault_slow &= ~FAULT_BIT_MOTOR_OVERTEMP;
                DW.motor_temp_cnt[0] = 0.0;
                DW.motor_temp_cnt[1] = 0.0;
            }
        } else if (det_run != 0U) {
            /* 运行态才做置位/清位判决; 停机/未使能时保持现状 —— 否则被别的下电类故障
               (缺相/主反馈等)停掉后, 下面"温度正常"分支会把已经报出的高温告警清掉 */
            if (DW.motor_anal.motor_temp >
                DW.faule_protection.over_motor_temp_thresh) {
                /* **** 超过过温阈值: 累加计时 → 触发下电 **** */
                DW.motor_temp_cnt[0] += base_slow;
                DW.motor_temp_cnt[1]  = 0.0;
                if (DW.motor_temp_cnt[0] >=
                    DW.faule_protection.over_motor_temp_thresh_Per * FAULT_TICKS_PER_MS) {
                    DW.motor_temp_cnt[0] = 0.0;
                    s_fault_slow |= FAULT_BIT_MOTOR_OVERTEMP;
                }
            } else if (DW.motor_anal.motor_temp >
                       DW.faule_protection.over_motor_temp_high_thresh) {
                /* **** 高温告警 (不下电) **** */
                DW.motor_temp_cnt[1] += base_slow;
                DW.motor_temp_cnt[0]  = 0.0;
                if (DW.motor_temp_cnt[1] >=
                    DW.faule_protection.over_motor_temp_thresh_Per * FAULT_TICKS_PER_MS) {
                    DW.motor_temp_cnt[1] = 0.0;
                    s_fault_slow |= FAULT_BIT_MOTOR_HIGH_TEMP;
                }
            } else {
                /* **** 温度正常: 清计数 + 清高温告警位 **** */
                DW.motor_temp_cnt[0] = 0.0;
                DW.motor_temp_cnt[1] = 0.0;
                s_fault_slow &= ~FAULT_BIT_MOTOR_HIGH_TEMP;
            }
        } else {
            /* 停机/未使能: 不置位; 只有温度确实回到告警阈值以下才清位(恢复照常评估) */
            if (DW.motor_anal.motor_temp <= DW.faule_protection.over_motor_temp_high_thresh) {
                DW.motor_temp_cnt[0] = 0.0;
                DW.motor_temp_cnt[1] = 0.0;
                s_fault_slow &= ~FAULT_BIT_MOTOR_HIGH_TEMP;
            }
        }
				
    return fault_state_buf;
}

/* 母线过压/欠压(1kHz): 置位(仅运行态)与自恢复; 计数 1ms 步进, 确认时间 = 参数值(ms) */
/* 启动联锁(使能沿, 1kHz): 指令启动那一刻母线就已越限 -> 同拍置位。
   中断内顺序是"故障检测(6) -> 模式仲裁(7) -> 启停逻辑(8)", 本拍置位后第 8 步会因
   FAULT_PD_MASK 命中而不开输出 —— 母线不足时电机根本不启动。
   (旧实现只有运行态检测: 必须先把输出开起来才判, 于是"母线 57V 也能启动、等确认时间才报欠压")
   读数有效门: 母线低于 0.2x欠压阈值 视为"没接电源/未上电"不锁死, 台架不带电测逻辑仍可试。 */
static uint32_T Fp_BusArm(uint32_T fault_state_buf, uint8_T det_run)
{
    static uint8_t s_en_last = 0U;
    real32_T vb      = DW.motor_anal.bus_Volt;
    real32_T v_floor = DW.faule_protection.under_Volt_thresh * 0.2F;

    if ((det_run != 0U) && (s_en_last == 0U)) {          /* 使能沿 */
        if ((vb > v_floor) && (vb < DW.faule_protection.under_Volt_thresh)) {
            fault_state_buf |= FAULT_BIT_BUS_UNDERVOLTAGE;
        } else if (vb > DW.faule_protection.over_Volt_thresh) {
            fault_state_buf |= FAULT_BIT_BUS_OVERVOLTAGE;
        }
    }
    s_en_last = det_run;
    return fault_state_buf;
}

static uint32_T Fp_SlowBus(uint32_T fault_state_buf, uint8_T det_run, real32_T base_slow)
{
				/* 2.1 母线过压 (动作: > 过压阈值; 恢复: <= 自恢复点参数, 默认 阈值-2V) */
				if (fault_state_buf & FAULT_BIT_BUS_OVERVOLTAGE) { //过压恢复检测
						if (DW.motor_anal.bus_Volt <= BusOvRec_Thresh()) {
								DW.faule_protection.over_Volt_Cnt = (uint16_T)(DW.faule_protection.over_Volt_Cnt + 1U);
								if (DW.faule_protection.over_Volt_Cnt >= DW.faule_protection.over_Volt_Per) {
										DW.faule_protection.over_Volt_Cnt = 0U;
										fault_state_buf &= ~FAULT_BIT_BUS_OVERVOLTAGE;
								}
						} else {
								DW.faule_protection.over_Volt_Cnt = 0U;
						}
				} else if ((det_run != 0U) && (DW.motor_anal.bus_Volt > DW.faule_protection.over_Volt_thresh)) { //过压检测(仅运行态)
						DW.faule_protection.over_Volt_Cnt = (uint16_T)(DW.faule_protection.over_Volt_Cnt + 1U);
						if (DW.faule_protection.over_Volt_Cnt >= DW.faule_protection.over_Volt_Per) {
								DW.faule_protection.over_Volt_Cnt = 0U;
								fault_state_buf |= FAULT_BIT_BUS_OVERVOLTAGE;
						}
				} else {
						DW.faule_protection.over_Volt_Cnt = 0U;
				}
				/* 2.2 母线欠压 (动作: < 欠压阈值; 恢复: >= 自恢复点参数, 默认 阈值+2V)
				   计数 1ms 步进, 确认时间 = under_Volt_Per(ms) */
				if (fault_state_buf & FAULT_BIT_BUS_UNDERVOLTAGE) {//欠压恢复检测
						if (DW.motor_anal.bus_Volt >= BusUvRec_Thresh()) {
								DW.faule_protection.under_Volt_Cnt = (uint16_T)(DW.faule_protection.under_Volt_Cnt + 1U);
								if (DW.faule_protection.under_Volt_Cnt >= DW.faule_protection.under_Volt_Per) {
										DW.faule_protection.under_Volt_Cnt = 0U;
										fault_state_buf &= ~FAULT_BIT_BUS_UNDERVOLTAGE;
								}
						} else {
								DW.faule_protection.under_Volt_Cnt = 0U;
						}
				} else if ((det_run != 0U) && (DW.motor_anal.bus_Volt < DW.faule_protection.under_Volt_thresh)) {   //欠压检测(仅运行态)
						DW.faule_protection.under_Volt_Cnt = (uint16_T)(DW.faule_protection.under_Volt_Cnt + 1U);
						if (DW.faule_protection.under_Volt_Cnt >= DW.faule_protection.under_Volt_Per) {//欠压检测
								DW.faule_protection.under_Volt_Cnt = 0U;
								fault_state_buf |= FAULT_BIT_BUS_UNDERVOLTAGE;
						}
				} else {
						DW.faule_protection.under_Volt_Cnt = 0U;
				}
    return fault_state_buf;
}

static uint32_T s_selfcheck_start_ms = 0U;

void Fault_SelfCheck_Start(void)
{
    s_selfcheck_start_ms = HAL_GetTick();
    DW.check_Cnt = 0U;
    DW.faule_protection.self_check = 1U;
    DW.fun_state_machine.self_check_Sta = 0U;
    /* 自检期间禁止控制启动: 即使 CAN 提前下发使能/模式, 也不允许绕过自检 */
    DW.comm_contrl.control_enable = 0U;
    DW.comm_contrl.control_mode = 0U;
    DW.comm_contrl.contorl_mode_can = 0U;
    DW.comm_contrl.contorl_mode_fun = 0U;
}

/* 故障清除(脉冲): 故障位/计数/证据量/来源码/上报状态一并清零, 并发起一次自检 */
static uint32_T Fp_ClearAll(void)
{
    uint32_T fault_state_buf = 0U;   /* 全量清零后返回 */
         /* 清除故障(脉冲式): 故障位/计数/证据量/来源码/上报状态一并清零, 命令自清防永久屏蔽 */
        fault_state_buf = 0U;
        s_fault_slow = 0U;   /* 慢分支缓存随故障清除一并归零 */
        /* 确认计数与回滞状态 */
        DW.circuilt_cnt[0] = 0.0;
        DW.circuilt_cnt[1] = 0.0;
        DW.circuilt_cnt[2] = 0.0;
        DW.mos_temp_cnt[0] = 0.0;
        DW.mos_temp_cnt[1] = 0.0;
        DW.motor_temp_cnt[0] = 0.0;
        DW.motor_temp_cnt[1] = 0.0;
        DW.faule_protection.over_Volt_Cnt  = 0U;
        DW.faule_protection.under_Volt_Cnt = 0U;
        DW.handlebar_cnt = 0.0F;
        DW.LR_Cnt        = 0.0F;
        DW.check_Cnt     = 0U;
        DW.faule_protection.LR_state_o = 0U;
        /* 各故障的子状态与下发零 */
        DW.faule_protection.short_circuit   = 0U;
        DW.faule_protection.MCU_flash_state = 0U;
        DW.faule_protection.LR_state_o      = 0U;
        DW.faule_protection.handlebar_state = 0U;
        DW.comm_contrl.fault_power_down_state = 0U;   /* 下电状态随故障位一起清, 清完即可再使能 */
        /* 快速位(过流/短路/主反馈丢失)存活在本寄存器里: 快速任务只置位不清位,
           尾部合并又会把本寄存器的快速位并回清零后的缓冲 —— 不在此处直接清零,
           这三位将永远无法通过 fault_clear 清除 */
        DW.comm_contrl.fault_state = 0U;
        /* 上报状态: lsat 归零, 下次新故障仍按"变化"上报一次 */
        DW.comm_contrl.fault_state_lsat = 0U;
        /* 来源码 + 判据证据量 */
        s_fb_src = 0U;  s_pl_src = 0U;
        s_fb_quiet_ticks = 0U;  s_ln_peer[0] = 0U;  s_ln_peer[1] = 0U;
        s_ln_stale[0] = 0U; s_ln_stale[1] = 0U;
        s_ln_motion_armed = 0U; s_ln_oc_ticks = 0U;
        s_spd_invalid = 0U; s_spd_mem_tick = SPD_MEM_TICKS; s_fbk_drop_mem = 0U;
        s_cnt_still_ticks = 0U; s_fbk_cnt_last = (uint16_T)(ENC_TIM->CNT);
        s_pwm_lose_cnt = 0U; s_pwm_frames_last = 0U; s_pwm_edges_last = 0U;
        s_pwm_en_last = 0U; s_pos_en_last = 0U; s_poschk_hit = 0U;
        s_run_last = 0U; s_mode_last = 0U;
        s_pos_pwm_acc = 0; s_pos_cnt_acc = 0; s_pos_div = 0U;
        s_pos_pwm_last = g_enc.angle_x10; s_pos_cnt_last = (uint16_T)(ENC_TIM->CNT);
        s_mot_div = 0U; s_mot_pwm_acc = 0; s_mot_cnt_acc = 0; s_mot_hold = 0U;
        s_mot_pwm_last = g_enc.angle_x10; s_mot_cnt_last = (uint16_T)(ENC_TIM->CNT);
        s_phase_lose_cnt = 0U; s_phase_multi_cnt = 0U; s_min_hold = 0U;
        s_phase_env[0] = 0.0F; s_phase_env[1] = 0.0F; s_phase_env[2] = 0.0F;
        s_pl_fre[0] = s_pl_fre[1] = s_pl_fre[2] = 0.0F;
        s_pl_fim[0] = s_pl_fim[1] = s_pl_fim[2] = 0.0F;
        s_pl_famp[0] = s_pl_famp[1] = s_pl_famp[2] = 0.0F;
        s_pl_fmax = 0.0F; s_pl_fmin = 0.0F; s_pl_inhibit = 0U; s_pl_fund_cnt = 0U;
        s_dc_conv_ticks = 0U; s_dc_ready = 0U; s_run_ticks = 0U;
        s_dc_u = 0.0F; s_dc_w = 0.0F; s_min_ph = 0U; s_pl_above = 0U;
        s_pl_err_d = 0.0F; s_pl_err_q = 0.0F; s_pl_vmag2 = 0.0F;
        s_pl_pi_active = 0U; s_pl_push_fail = 0U; s_pl_imbal = 0U;
        s_pl_imag = 0.0F; s_pl_v_evid = 0U; s_pl_dead = 0U;
        s_ovspd_hold = 0U; s_spd_prev = 0; s_spd_prev_ok = 0U;
        /* 命令自清 + 进入一次自检 */
        DW.faule_protection.fault_clear = 0U;
        s_fwchk_req = 1U;   /* 重新做一次固件完整性校验(主循环执行, 自检分支判定) */
        DW.faule_protection.self_check = 1U;
        DW.fun_state_machine.self_check_Sta = 0U;
        s_selfcheck_start_ms = HAL_GetTick();
        /* 清故障后同样进入自检, 禁止带使能/模式跳过自检 */
        DW.comm_contrl.control_enable = 0U;
        DW.comm_contrl.control_mode = 0U;
        DW.comm_contrl.contorl_mode_can = 0U;
        DW.comm_contrl.contorl_mode_fun = 0U;
        FOC_Ctrl_Clear();
    return fault_state_buf;
}


/* 转把电压越限计数(1kHz, 200ms 双向确认) */
static uint8_T Fp_ThrottleCount(uint8_T handlebar_state, real32_T base_slow)
{
    {
        real32_T hb_v = (real32_T)DW.motor_anal.handlebar_Volt;
        if ((hb_v < DW.faule_protection.handlebar_voltage[1]) ||
            (hb_v > DW.faule_protection.handlebar_voltage[0])) {
            /* 越限: 累计到确认时间 -> 置位; 计数饱和, 不在置位时清零 */
            if (DW.handlebar_cnt < (real32_T)HANDLEBAR_FAULT_TICKS) {
                DW.handlebar_cnt += base_slow;
            }
            if (DW.handlebar_cnt >= (real32_T)HANDLEBAR_FAULT_TICKS) {
                handlebar_state = 1U;
            }
        } else if ((hb_v > (DW.faule_protection.handlebar_voltage[1] + HANDLEBAR_HYST_V)) &&
                   (hb_v < (DW.faule_protection.handlebar_voltage[0] - HANDLEBAR_HYST_V))) {
            /* 明确回到范围内: 累计回到 0 才清位 */
            if (DW.handlebar_cnt > 0.0F) {
                DW.handlebar_cnt -= base_slow;
            } else {
                handlebar_state = 0U;
            }
        }
        /* 回滞带内(限值附近): 状态与计数都保持, 防止在阈值处来回翻转 */
    }
    return handlebar_state;
}

void Fault_FastTask(void)
{
    volatile uint32_T fault_state_buf;
    if (DW.faule_protection.fault_clear == 1U) { return; }

    fault_state_buf = DW.comm_contrl.fault_state;
    if (DW.motor_param.sensor_type != 1U) {
        fault_state_buf = Fp_LineWatch(fault_state_buf);
    }
    if ((DW.comm_contrl.control_mode != 0U) &&
        (DW.comm_contrl.control_enable == 1U)) {
        if ((fabsf(DW.motor_anal.phaseU_Curr) >= DW.faule_protection.Curr_thresh) ||
            (fabsf(DW.motor_anal.phaseV_Curr) >= DW.faule_protection.Curr_thresh) ||
            (fabsf(DW.motor_anal.phaseW_Curr) >= DW.faule_protection.Curr_thresh)) {
            fault_state_buf |= FAULT_BIT_OVERCURRENT;
        }
        if (DW.faule_protection.short_circuit == 1U) {
            fault_state_buf |= FAULT_BIT_PHASE_SHORT;
        }
    }
    if ((fault_state_buf & FAULT_PD_MASK) != 0U) {
        DW.comm_contrl.fault_power_down_state = 1U;
        DW.comm_contrl.control_enable = 0U;
        DW.comm_contrl.control_mode = 0U;
    }
    DW.comm_contrl.fault_state = fault_state_buf;
}

void Fault_Protect_Task(void)
{
	
    volatile real32_T base_cnt;
    volatile uint32_T fault_state_buf;
    volatile uint8_T  handlebar_state;
    volatile uint8_T  power_down = 0U;
		volatile real32_T u, v, w;
		
		/* 1、读取以前的故障情况 */
		handlebar_state = DW.faule_protection.handlebar_state;
		fault_state_buf = DW.comm_contrl.fault_state;
    /* 时基 10000/PWM_freq, 防 0 */
   // base_cnt = DW.motor_anal.time_base;
		base_cnt=1;
    if (base_cnt == 0.0F) {
        base_cnt = 1.0F;
    }

    /* 主循环1kHz执行: 每次均为1ms慢拍, 计数步长与原ISR慢分支一致 */
    real32_T base_slow = base_cnt * (real32_T)FAULT_SLOW_DIV_TICKS;
    uint8_T  slow_tick = 1U;

    /* 转把电压异常: 越限(低于下限或高于上限)持续 200ms 置位; 清位同样要求"明确回到范围内"
       持续 200ms(带 0.05V 回滞)。原实现是"置位计 200ms、清位瞬时清零": 转把静止电压
       常压在限值附近(静止约 0.8V, 而下限就是 0.8V), 一次抖动就瞬时清位、再花 200ms 重新
       置位 -> 状态反复翻转, 每翻转一次就上报一次故障。
       计数 handlebar_cnt 双向: 越限 +1, 明确恢复 −1, 落在回滞带内保持不动(不翻转)。 */
    if (slow_tick != 0U) { handlebar_state = Fp_ThrottleCount(handlebar_state, base_slow); }
/************************ 一、故障清除 *******************************************************/				
    if (DW.faule_protection.fault_clear == 1U) {
        fault_state_buf = Fp_ClearAll();
    } else {
/************************ 二、常规检测/自恢复 **************************************/
/*   以下过压/欠压/电机过温/驱动器过温/两路高温告警只在"运行态(闭环 + 已使能)"置位:
   停机/未使能/电机不转时不检测这几条; 但恢复(清位)分支照常评估, 停机也能自动恢复。 */
uint8_T det_run = ((DW.faule_protection.self_check == 0U) &&
                   (DW.comm_contrl.control_mode != 0U) &&
                   (DW.comm_contrl.control_enable == 1U)) ? 1U : 0U;
				/* 主反馈异常的门控: 3.0/3.0a/3.0b 只在"闭环模式 + 已使能"的运行态判定(开环(VF/防盗/
				   高频注入)、停机/未使能、找零与校准期间一律不判); 3.0c(A/B 单线断)例外 —— 它自带转子
				   在动的旁证, 输出被下游故障切掉后仍要判, 否则断线根因再也报不出来。 */
				u = fabsf(DW.motor_anal.phaseU_Curr);
				v = fabsf(DW.motor_anal.phaseV_Curr);
				w = fabsf(DW.motor_anal.phaseW_Curr);
				/* ================= 过压 / 欠压: 自恢复逻辑说明 =================
				   两者同一套写法: "触发档 / 恢复档"二选一, 共用同一个确认计数。
				     触发档: 越过动作阈值(过压 > 过压阈值 / 欠压 < 欠压阈值) 持续确认时间 -> 置位;
				     恢复档: 故障位已在时, 回到"动作阈值 ∓ 2V"以内 持续确认时间 -> 清位(回滞 2V);
				   计数清零: 任一档内条件不成立即清零(不是漏泄), 即要求"连续成立满时间";
				   评估时机: 本段在"与运行状态无关"分支里, 停机态也评估 —— 这是停机后仍能自动恢复的原因;
				   作用范围: 只改本地故障位, 下电状态由故障位派生 -> 清位即恢复输出。
				   回滞量已参数化: over_Volt_recover / under_Volt_recover (曾为写死 2V); 置位与恢复共用同一个确认时间。
			   注意: 过/欠压计数 1ms 步进(参数即 ms); 温度/堵转/电路异常计数按 100us 拍步进,
			   比较时参数(ms)x10 (FAULT_TICKS_PER_MS) —— 全部 *_Per 参数统一 ms 口径
				   ============================================================ */
				if (slow_tick != 0U) { fault_state_buf = Fp_BusArm(fault_state_buf, det_run); }   /* 使能沿联锁: 先于本拍启停逻辑 */
				if (slow_tick != 0U) { fault_state_buf = Fp_SlowBus(fault_state_buf, det_run, base_slow); }
				if (slow_tick != 0U) { fault_state_buf = Fp_SlowTempMotor(fault_state_buf, det_run, base_slow); }
			  if (slow_tick != 0U) { fault_state_buf = Fp_SlowTempDriver(fault_state_buf, det_run, base_slow); }
		if (slow_tick != 0U) { fault_state_buf = Fp_SlowThrottleBit(fault_state_buf, handlebar_state); }
		if (slow_tick != 0U) { fault_state_buf = Fp_SlowStackGuard(fault_state_buf); }
        Fp_RunGate(slow_tick);

				/* 3.0c A/B 单线断: TIM2 线边沿 + 闭环运动门; 过流后按 Fp_LineWatch 限时补记 */
				/* A/B线断由Fault_FastTask在10kHz ISR中处理; Fp_FbkWatch当前无有效判定 */

/************************三、运行态故障检测 **************************************/				
				if (DW.comm_contrl.control_mode != 0U&&DW.comm_contrl.control_enable==1) {			

				if (slow_tick != 0U) { fault_state_buf = Fp_RunCriteria(fault_state_buf, base_slow); }
			/* 3.1 过流: 3 拍确认, 锁存(仅 fault_clear 清除) */
				if ((u >= DW.faule_protection.Curr_thresh) ||
						(v >= DW.faule_protection.Curr_thresh) ||
						(w >= DW.faule_protection.Curr_thresh)) {
						DW.circuilt_cnt[1] += base_cnt;
						if (DW.circuilt_cnt[1] >= 3.0) {
								DW.circuilt_cnt[1] = 0.0;
								fault_state_buf |= FAULT_BIT_OVERCURRENT;
						}
				} 
						else {
						DW.circuilt_cnt[1] = 0.0;
				}
				/* 3.2 短路 (TIM1 BKIN 硬件触发后置位 short_circuit): 锁存 */
				if (DW.faule_protection.short_circuit == 1U) {
						fault_state_buf |= FAULT_BIT_PHASE_SHORT;
				}
			} else if (DW.faule_protection.self_check == 1U) {
				s_fwchk_req = 1U;   /* 进入自检: 请求重新校验固件完整性 */
/***********************四、 自检 (停机态) **************************************/				
            if (slow_tick != 0U) {   /* 1kHz 慢分支: 零漂自检(1ms 确认不变) */
            /* 4.1 电流采样漂移自检: 零偏已扣除, 静止时 |电流| 超阈值即为零漂 */

            if ((fabsf(DW.motor_anal.phaseU_Curr) > DW.faule_protection.hardware_phase_curr) ||
                (fabsf(DW.motor_anal.phaseV_Curr) > DW.faule_protection.hardware_phase_curr)) {
                DW.circuilt_cnt[0] += base_slow;
                if (DW.circuilt_cnt[0] > 10.0) {
                    DW.circuilt_cnt[0] = 0.0;
                    fault_state_buf |= FAULT_BIT_PHASE_OFFSET;
                }
            } else {
                DW.circuilt_cnt[0] = 0.0;
            }

            }
            if (slow_tick != 0U) {   /* 1kHz 慢分支: 温度采样电路自检 */
            /* 4.2  温度传感器电路自检 (ADC 越界持续 -> MOS:16 / 电机:32) */
            if ((DW.motor_anal.mos_temp_Rs < DW.faule_protection.hardware_temp_lower_limit[0]) || (DW.motor_anal.mos_temp_Rs  >DW.faule_protection.hardware_temp_upper_limit[0])) {
                DW.faule_protection.hardware_temp_thresh_Cnt[0] += base_slow;
                if (DW.faule_protection.hardware_temp_thresh_Cnt[0] > DW.faule_protection.hardware_temp_thresh_Per[0] * FAULT_TICKS_PER_MS) {
                    DW.faule_protection.hardware_temp_thresh_Cnt[0] = 0.0;
                    s_fault_slow |= FAULT_BIT_DRIVER_TEMP_SENSOR;
                }
            } else {
                DW.faule_protection.hardware_temp_thresh_Cnt[0] = 0.0;
            }
            if ((DW.motor_anal.motor_temp_Rs  < DW.faule_protection.hardware_temp_lower_limit[1]) || (DW.motor_anal.motor_temp_Rs  > DW.faule_protection.hardware_temp_upper_limit[1])) {
                DW.faule_protection.hardware_temp_thresh_Cnt[1] += base_slow;
                if (DW.faule_protection.hardware_temp_thresh_Cnt[1] > DW.faule_protection.hardware_temp_thresh_Per[1] * FAULT_TICKS_PER_MS) {
                    DW.faule_protection.hardware_temp_thresh_Cnt[1] = 0.0;
                    s_fault_slow |= FAULT_BIT_MOTOR_TEMP_SENSOR;
                }
            } else {
                DW.faule_protection.hardware_temp_thresh_Cnt[1]=0;
            }
            }
            /* 4.3 MCU 存储器(Flash)自检: 结果由启动阶段的固件完整性校验写入 MCU_flash_state。
                   校验要读全代码区并可能写 EEPROM, 不能在中断里做, 所以"算"在 main 里、"判定"在这里 */
            if (DW.faule_protection.MCU_flash_state != 0U) {
                fault_state_buf |= FAULT_BIT_MCU_MEMORY;
            }

            /* 保存上一拍电流, 自检节拍计数 (0~1199 循环, 1kHz 慢拍 -> 窗口 1200ms) */
            DW.U_Curr_last = DW.adc_slmple.phaseU_Curr;
            DW.V_Curr_last = DW.adc_slmple.phaseV_Curr;
            /* 自检窗口用真实时间, 不依赖任务调用次数:
               主循环被抢占/任务延迟时, 仍按 1.2s 上限退出 */
            if (slow_tick != 0U) {
                if ((HAL_GetTick() - s_selfcheck_start_ms) >= 1200U) {
                    DW.check_Cnt = 0U;
                    DW.faule_protection.self_check = 0U;
                    DW.fun_state_machine.self_check_Sta = 1U;
                    DW.comm_contrl.control_Run_state = 1U;
                } else if (DW.check_Cnt < 0xFFFFU) {
                    DW.check_Cnt++;   /* 仅诊断观察, 不参与退出判定 */
                }
            }
        }
    }
    /* 合并ISR快速保护最新结果, 防止主循环慢任务被抢占后覆盖故障位 */
    fault_state_buf |= DW.comm_contrl.fault_state &
                       (FAULT_BIT_OVERCURRENT | FAULT_BIT_PHASE_SHORT |
                        FAULT_BIT_MOTOR_FEEDBACK_LOSS);
    fault_state_buf = (fault_state_buf & ~FAULT_SLOW_MASK) | s_fault_slow;

    /* ***** 五、下电状态由故障位派生: 故障位锁存期间持续下电, 消除反复启停 ***** */
    if ((fault_state_buf & FAULT_PD_MASK) != 0U) {
        power_down = 1U;
    }
    if (((fault_state_buf & FAULT_BIT_STALL) != 0U) &&
        (DW.faule_protection.LR_mode != 1U)) {
        power_down = 1U;
    }
    /* ***** 六、状态输出 ***** */
		/****** 6.0 整车故障等级域 (B2.5/B2.6 = bit21/22): 取激活故障最高分级,
					 派生显示量, 不参与保护逻辑 ***** */
		{
				uint32_T lv;
				if ((fault_state_buf & FAULT_LV3_MASK) != 0U)      { lv = 3U; }
				else if ((fault_state_buf & FAULT_LV2_MASK) != 0U) { lv = 2U; }
				else if ((fault_state_buf & FAULT_LV1_MASK) != 0U) { lv = 1U; }
				else                                               { lv = 0U; }
				fault_state_buf = (fault_state_buf & ~(3UL << 21)) | (lv << 21);
		}
		//状态输出
    DW.comm_contrl.fault_power_down_state = (uint16_T)power_down;
    DW.comm_contrl.fault_state            = fault_state_buf;
		DW.faule_protection.handlebar_state		= handlebar_state;
}

/******************************************************************************
 * 文件: encoder_foc_sdk.c
 * 功能: TIM2 正交编码器 FOC 驱动 (角度/转速/Z相检测/零位辨识/PWM绝对位置校零)
 * 创建日期: 2026-06-22
 * 修改日期: 2026-10-26
 * 修改内容: 机械每转计数统一为线数x4, 电角度使用电机PP_num;
 *           1kHz同步运行期线数/极对数配置
 * 修改人: rui.zhang
 ******************************************************************************/

#include "encoder_foc_sdk.h"
#include <string.h>
#include "encoder_pwm_u.h"                      /* EncPwmSamp_* PWM raw 去卷绕采样器 */
#include "foc_data.h"           /* DW 数据块 */
#include "MotorCtr_parameter.h"  /* Control_Mode_Idq */
#include "Parameter_Idf.h"        /* BestA_u / ST_IDF_* */

EncoderHandle_t g_enc_foc = {0};

/* 校零学习完成后的写入请求 (主循环上下文处理, 见 SignalProc_PwmZeroCalib) */
volatile uint8_t g_zero_ref_save_pending = 0U;

static uint16_t s_zero_cfg_pp = 0U;
static uint16_t s_zero_cfg_lines = 0U;

#if (ZERO_IDF_ALGORITHM == ZERO_IDF_ALG_IF_SWEEP)
static uint16_t s_idf_i_cnt       = 0U;
static uint16_t s_idf_i_target    = 5U;
static uint16_t s_idf_wait_steady = 0U;
#else
static uint32_t s_vq_last_cnt;
static uint16_t s_vq_last_pwm;
static uint32_t s_vq_best_cnt;
static uint16_t s_vq_best_pwm;
static int32_t  s_vq_dir_vote;
static uint8_t  s_vq_have_last;
#endif

static void ENC_ZeroIdf_StopFailed(void)
{
    BestA_u.qual_fail = 1U;
    DW.comm_contrl.zero_identify_state = ST_IDF_ZERO_FIRL;
    g_enc_foc.z_clear = 0U;
    BestA_u.Z_flag = 0U;
    BestA_u.PP_cnt = 0U;
    BestA_u.sweep_dir = 0U;
    EncPwmSamp_Abort();
}

void ENC_ZeroIdf_Reset(void)
{
    EncFoc_ApplyConfig();
    s_zero_cfg_pp = DW.motor_param.PP_num;
    s_zero_cfg_lines = DW.motor_param.enc_line_count;
#if (ZERO_IDF_ALGORITHM == ZERO_IDF_ALG_IF_SWEEP)
    s_idf_i_cnt       = 0U;
    s_idf_i_target    = 5U;
    s_idf_wait_steady = 0U;
    BestA_u.PP_cnt = 0U;
    BestA_u.Z_flag = 0U;
    BestA_u.sweep_dir = 0U;
    BestA_u.qual_fail = 0U;
    BestA_u.Angle_buff = 0U;
    g_enc_foc.z_phase_state = Z_IDLE;
    memset(BestA_u.TIM_cnt_buf, 0, sizeof(BestA_u.TIM_cnt_buf));
    memset(BestA_u.PWM_cnt_buf, 0, sizeof(BestA_u.PWM_cnt_buf));
#else
    s_vq_last_cnt  = 0U;
    s_vq_last_pwm  = 0U;
    s_vq_best_cnt  = 0U;
    s_vq_best_pwm  = 0U;
    s_vq_dir_vote  = 0;
    s_vq_have_last = 0U;
    BestA_u.Best_Angle_CNT = 0U;
    BestA_u.Average_CNT    = 0U;
    BestA_u.JILU_Vd        = 0.0F;
    BestA_u.JILU_Vq        = 0.0F;
    BestA_u.Vq_max         = 0.0F;
    BestA_u.Vq_max_index   = 0U;
    BestA_u.qual_fail      = 1U;
    DW.motor_main_feedback.elec_angle = 0U;
#endif
    EncPwmSamp_Abort();
}

/* 编码器初始化: 启动 TIM2 编码器模式并清零软件句柄 */
void ENC_Init(void)
{
    /* TIM2 已由 tim.c::MX_TIM2_Init() 完整初始化为编码器模式
       (TI12, Period=0xFFFFFFFF, IC Filter=10), 此处不再重复
       HAL_TIM_Encoder_Init, 以避免与底层 CubeMX 配置冲突/重复初始化。
       本函数仅负责: 启动编码器计数 + 建立软件句柄。            */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    ENC_TIM->CNT = 0;

    memset(&g_enc_foc, 0, sizeof(g_enc_foc));
    g_enc_foc.last_cnt_u16  = 0;
    g_enc_foc.PrevCapture   = 0;
    EncFoc_ApplyConfig();   /* 线数/极对数走运行期参数(CAN 可改), 默认 1024 线 / 4 极对 */
}

/* 应用运行期配置: 机械每转计数 = 线数x4 (正交4倍频), 与电机极对数无关。
   电机极对数只用于机械角->电角度、每电周期计数和PWM角度域。
   兜底: 线数超出64~16383或极对数不在1~64时按默认值处理, 避免溢出和除零。 */
void EncFoc_ApplyConfig(void)
{
    uint16_t ln = DW.motor_param.enc_line_count;
    uint16_t pp = DW.motor_param.PP_num;
    uint16_t pn;

    g_enc_foc.RevertSignal = (DW.motor_param.encoder_reverse != 0U) ? 1U : 0U;
    if ((ln < 64U) || (ln > 16383U)) { ln = 1024U; }
    if ((pp < 1U) || (pp > 64U)) { pp = 4U; }

    pn = (uint16_t)(ln * 4U);
    if ((g_enc_foc.PulseNumber == pn) &&
        (g_enc_foc.bElToMecRatio == (uint8_t)pp)) { return; }

    g_enc_foc.PulseNumber   = pn;
    /* 只有配置变化才重算派生系数; ENC_Init 后 PulseNumber=0, 必会重新应用 */
    g_enc_foc.SpdCoeffF     = 60000.0f / (16.0f * (float)pn);
    g_enc_foc.MecDegPerCntF = 360.0f / (float)pn;
    g_enc_foc.bElToMecRatio = (uint8_t)pp;

    /* PulseNumber 为 2 的幂时用掩码+移位, 免去每拍的取模与 64 位除法 */
    if ((pn != 0U) && ((pn & (uint16_t)(pn - 1U)) == 0U)) {
        uint8_t  sh = 0U;
        uint16_t v  = pn;
        while (v > 1U) { v >>= 1; sh++; }
        g_enc_foc.PulseMask  = (uint16_t)(pn - 1U);
        g_enc_foc.PulseShift = (uint8_t)(16U - sh);
    } else {
        g_enc_foc.PulseMask  = 0U;
        g_enc_foc.PulseShift = 0xFFU;
    }
}

/* 每拍最大合法计数增量: 按 22000rpm 折算 (原固定 1500 相当于 1024 线时的 22000rpm) */
/* A/B 线边沿活动查询(主反馈判据用): 输入级每个有效边沿置位 CC1IF/CC2IF
   (编码器启动时捕获通道已使能), 读即清; 返回 bit0=A线 bit1=B线
   =本查询周期内该线出现过边沿。健康正交两线活动度恒相等 */
uint8_t EncFoc_LinePulse(void)
{
    uint32_t sr = ENC_TIM->SR;
    uint8_t  r  = 0U;
    if ((sr & TIM_SR_CC1IF) != 0U) { r |= 1U; }
    if ((sr & TIM_SR_CC2IF) != 0U) { r |= 2U; }
    ENC_TIM->SR = sr & ~(TIM_SR_CC1IF | TIM_SR_CC2IF);   /* 写0清除, 其余位保持 */
    return r;
}

uint16_t EncFoc_DeltaMax(void)
{
    uint32_t pn = (uint32_t)g_enc_foc.PulseNumber;
    uint32_t dm = (pn * 22000U) / 60000U;   /* 每 ms 计数 = pn * rpm / 60000 */
    if (dm < 16U)    { dm = 16U; }
    if (dm > 60000U) { dm = 60000U; }
    return (uint16_t)dm;
}

/* 编码器角度更新(10kHz): 读取 TIM2 CNT 差分累积, 计算机械/电角度
   校零完成后应用 zero_offset 校正电角度。 */
void ENC_CalcAngle(void)
{
    uint16_t cnt_u16 = (uint16_t)(ENC_TIM->CNT);
    int16_t  diff    = (int16_t)(cnt_u16 - g_enc_foc.last_cnt_u16);
    g_enc_foc.last_cnt_u16 = cnt_u16;
    g_enc_foc.raw_count += (int32_t)diff;

    int32_t mech_mod;
    int16_t mecAngle;
    if (g_enc_foc.PulseShift != 0xFFU) {          /* 2 的幂: 掩码 + 移位 */
        mech_mod = g_enc_foc.raw_count & (int32_t)g_enc_foc.PulseMask;
        mecAngle = (int16_t)((uint16_t)((uint32_t)mech_mod << g_enc_foc.PulseShift));
    } else {
        mech_mod = g_enc_foc.raw_count % (int32_t)g_enc_foc.PulseNumber;
        if (mech_mod < 0) { mech_mod += g_enc_foc.PulseNumber; }
        mecAngle = (int16_t)(((uint64_t)mech_mod * 65536ULL) / (uint64_t)g_enc_foc.PulseNumber);
    }

    if (g_enc_foc.RevertSignal) { mecAngle = -mecAngle; }

    int16_t elAngle = mecAngle * (int16_t)g_enc_foc.bElToMecRatio;
    g_enc_foc.hMecAngle = mecAngle;

    /* 校零完成后应用零位偏移校正电角度 */
    if ((g_enc_foc.zero_state == 1U) && (g_enc_foc.z_first_state == 1U)) {
        g_enc_foc.hElAngle = elAngle - (int16_t)g_enc_foc.zero_offset;
    } else {
        g_enc_foc.hElAngle = elAngle;
    }
}

/* 编码器速度计算(1kHz): 同步运行期配置, 16 拍滑动窗口平均 + RPM 换算 */
void ENC_CalcSpeed(void)
{
    EncFoc_ApplyConfig();
    uint16_t curr = (uint16_t)(ENC_TIM->CNT);
    int16_t  diff16 = (int16_t)(curr - g_enc_foc.PrevCapture);
    int32_t  delta  = (int32_t)diff16;

    /* 物理不可能的拍(多为 Z 相清零/计数污染): 先用上一拍有效值顶替再入窗,
       原实现先入窗后判断, 窗口里留下的仍是坏值, 等于没有过滤 */
    if ((delta > (int32_t)EncFoc_DeltaMax()) || (delta < -(int32_t)EncFoc_DeltaMax())) {
        delta = g_enc_foc.DeltaBuffer[(g_enc_foc.DeltaIndex - 1) & 0x0F];
    }
    g_enc_foc.DeltaBuffer[g_enc_foc.DeltaIndex] = delta;
    g_enc_foc.DeltaIndex = (g_enc_foc.DeltaIndex + 1) & 0x0F;
    int32_t sum = 0;
    for (int i = 0; i < 16; i++) sum += g_enc_foc.DeltaBuffer[i];
    /* 16 拍平均保留小数: 整数右移丢 4 位(4096 线时读数只剩 14.65rpm 一档),
       且 floor 让均值恒低 0.5 计数(闭环把真机转速抬高约 7rpm) */
    g_enc_foc.hAvrSpeedRPM  = (float)sum * g_enc_foc.SpdCoeffF;   /* 系数已预算, 免每 ms 一次 VDIV */
    g_enc_foc.hAvrMecSpeed  = (int16_t)((sum + ((sum >= 0) ? 8 : -8)) / 16);   /* 调试观察: 整数计数/ms */
    if (g_enc_foc.RevertSignal != 0U) {
        g_enc_foc.hAvrSpeedRPM = -g_enc_foc.hAvrSpeedRPM;
        g_enc_foc.hAvrMecSpeed = (int16_t)(-g_enc_foc.hAvrMecSpeed);
    }
    g_enc_foc.PrevCapture   = curr;
}

/* 获取机械角度 (度): raw_count -> 0~360° */
float ENC_GetMecAngleDeg(void)
{
    int32_t mech_mod;
    if (g_enc_foc.PulseShift != 0xFFU) {
        mech_mod = g_enc_foc.raw_count & (int32_t)g_enc_foc.PulseMask;
    } else {
        mech_mod = g_enc_foc.raw_count % g_enc_foc.PulseNumber;
        if (mech_mod < 0) { mech_mod += g_enc_foc.PulseNumber; }
    }
    if ((DW.motor_param.sensor_type == 0U) && (g_enc_foc.RevertSignal != 0U)) {
        mech_mod = (g_enc_foc.PulseNumber - mech_mod) % g_enc_foc.PulseNumber;
    }
    return (float)mech_mod * g_enc_foc.MecDegPerCntF;   /* 系数已预算, 免每拍一次 VDIV */
}

/* ********************* Z相上升沿检测 (10kHz调用) ********************* */
/* 原理: 磁编码器Z相输出宽脉冲 (可能占空比50%), 不能用电平检测。
   必须检测上升沿, 并在触发后锁定一段时间防止同一脉冲重复捕获。
   状态机: Z_IDLE(检测高电平) -> Z_WAIT_LOW(消抖5拍) -> Z_WAIT_RISE(等下降沿)
          -> Z_LOCKOUT(锁定500 counts) -> Z_IDLE
   检测到上升沿时清零 ENC_TIM->CNT 建立 Z 相零位参考。                      */
void ZPhase_Process(void)
{
    uint8_t  z_pin       = ((Encoder_Z_GPIO_Port->IDR & Encoder_Z_Pin) != 0U) ? 1U : 0U;
    int32_t  current_raw = (int32_t)(ENC_TIM->CNT);
    static uint8_t delay_cnt = 0;

    switch (g_enc_foc.z_phase_state) {

    case Z_IDLE:
        /* 检测到高电平 -> 进入消抖 */
        if (z_pin == GPIO_PIN_SET) {
            g_enc_foc.z_phase_state = Z_WAIT_LOW;
            delay_cnt               = 0;
            /* 若 z_clear 使能, 清零硬件计数器作为 Z 相零位 */
            if (g_enc_foc.z_clear == 1U) {
                ENC_TIM->CNT = 0;
                /* Z 相清零: 同步归零硬件 CNT、raw_count 与速度/角度基准
                   (只在辨识期间 z_clear==1 生效) */
                g_enc_foc.raw_count    = 0;
                g_enc_foc.PrevCapture  = 0U;   /* 速度基准同步 */
                g_enc_foc.last_cnt_u16 = 0U;   /* 角度基准同步 */
            }
        }
        break;

    case Z_WAIT_LOW:
        /* 持续高电平消抖 (5拍 = 0.5ms @10kHz) */
        if (++delay_cnt >= Z_DEBOUNCE_TICKS) {
            delay_cnt = 0;
            if (z_pin == GPIO_PIN_SET) {
                g_enc_foc.z_phase_state = Z_WAIT_RISE;
            } else {
                g_enc_foc.z_phase_state = Z_IDLE;  /* 假高电平, 复位 */
            }
        }
        break;

    case Z_WAIT_RISE:
        /* 下降沿 -> 确认上升沿, 进入锁定 */
        if (z_pin == GPIO_PIN_RESET) {
            g_enc_foc.z_lockout_cnt   = (uint32_t)current_raw;
            g_enc_foc.z_phase_state   = Z_LOCKOUT;
        }
        break;

    case Z_LOCKOUT:
        /* 锁定期间忽略所有Z相电平, 远离锁定位置后重新使能 */
        {
            int32_t dist = current_raw - (int32_t)g_enc_foc.z_lockout_cnt;
            if (dist < 0) dist = -dist;
            if (dist >= (int32_t)Z_LOCKOUT_COUNTS) {
                g_enc_foc.z_phase_state = Z_IDLE;
            }
            /* 首次找到Z相 */
            if (g_enc_foc.z_first_state == 0U) {
                g_enc_foc.z_first_state = 1U;
            }
        }
        break;

    default:
        g_enc_foc.z_phase_state = Z_IDLE;
        break;
    }
}
/* ---- 按极对数双向扫点校零 (正/反扫消摩擦残差, 每转谐波平均) ---- */
#define ZERO_IDF_COUNTS_PER_PP   ((int32_t)g_enc_foc.PulseNumber / (int32_t)g_enc_foc.bElToMecRatio)  /* counts/极对, 随运行期参数 */
#define ZERO_IDF_PWM_PER_PP      (4096 / (int32_t)g_enc_foc.bElToMecRatio)                          /* clocks/极对, 随运行期参数 */
/* ABZ 计数类限值随主反馈分辨率换算 (基准 @4096 计数/圈): 40/4096 ≈ 3.6°机械 */
#define ZERO_IDF_HYST_MAX_CNT    ((int32_t)((g_enc_foc.PulseNumber * 40U) / 4096U))
#define ZERO_IDF_HYST_MAX_PWM    48     /* PWM clocks: PWM 绝对角每圈固定 4096, 不随 ABZ 线数变 */
#define ZERO_IDF_SPREAD_MAX_CNT  ((int32_t)((g_enc_foc.PulseNumber * 60U) / 4096U))  /* 60/4096 ≈ 5.4°机械 */
#define ZERO_IDF_PWM_SPREAD_MAX  60     /* 同上: PWM 侧固定刻度 */
#define ZERO_IDF_PWM_SAMP_DELAY  10000U /* 稳态等待满1.0s后开始PWM采样 (后0.5s窗口) */


/* 折到 [0, m) */
static int32_t mod_norm(int32_t x, int32_t m)
{
    x %= m;
    return (x < 0) ? (x + m) : x;
}

#if (ZERO_IDF_ALGORITHM == ZERO_IDF_ALG_IF_SWEEP)
/* 模 m 圆周平均 (去卷绕法: 以首值为参考展开, 要求样本簇展宽 < m/2);
   spread 输出展开后 max-min, 可为 NULL */
static int32_t circ_mean_mod(const int32_t *v, uint8_t n, int32_t m, int32_t *spread)
{
    int32_t ref = v[0];
    int32_t sum = 0, mn = 0, mx = 0;
    uint8_t i;

    for (i = 0U; i < n; i++) {
        int32_t d = v[i] - ref;
        if (d >  m / 2) d -= m;
        if (d < -m / 2) d += m;
        sum += d;
        if (d < mn) mn = d;
        if (d > mx) mx = d;
    }
    if (spread != (int32_t *)0) { *spread = mx - mn; }
    return mod_norm(ref + sum / (int32_t)n, m);
}

/* 圆周距离 (|a-b| 折到 [0, m/2]) */
static int32_t circ_dist(int32_t a, int32_t b, int32_t m)
{
    int32_t d = mod_norm(a - b, m);
    return (d > m / 2) ? (m - d) : d;
}

/* 零位合成: 正/反扫各PP个记录折到极对内坐标圆周平均 -> zero_offset;
   正/反扫均值距离 = 迟滞, 超上限判失败; PWM记录折到mod(4096/PP)平均 -> pwm_zero_ref;
   由正扫相邻点变化方向学习 pwm_dir */
static void ENC_ZeroIdf_Synth(void)
{
    int32_t abz[2U * ZERO_IDF_MAX_PP], pwmv[2U * ZERO_IDF_MAX_PP];
    int32_t c0, cf, cb, ref0, pf, pb;
    int32_t spread_abz, hyst_abz, hyst_pwm, dsum;
    uint8_t i, pp;
    int64_t off16;

    pp = g_enc_foc.bElToMecRatio;
    for (i = 0U; i < pp; i++) {
        abz[i]       = mod_norm(BestA_u.TIM_cnt_buf[i],        ZERO_IDF_COUNTS_PER_PP);
        abz[pp + i]  = mod_norm(BestA_u.TIM_cnt_buf[pp + i],   ZERO_IDF_COUNTS_PER_PP);
        pwmv[i]      = mod_norm(BestA_u.PWM_cnt_buf[i],        ZERO_IDF_PWM_PER_PP);
        pwmv[pp + i] = mod_norm(BestA_u.PWM_cnt_buf[pp + i],   ZERO_IDF_PWM_PER_PP);
    }

    c0 = circ_mean_mod(abz, (uint8_t)(2U * pp), ZERO_IDF_COUNTS_PER_PP, &spread_abz);
    cf = circ_mean_mod(abz, pp, ZERO_IDF_COUNTS_PER_PP, (int32_t *)0);
    cb = circ_mean_mod(&abz[pp], pp, ZERO_IDF_COUNTS_PER_PP, (int32_t *)0);
    pf = circ_mean_mod(pwmv, pp, ZERO_IDF_PWM_PER_PP, (int32_t *)0);
    pb = circ_mean_mod(&pwmv[pp], pp, ZERO_IDF_PWM_PER_PP, (int32_t *)0);
    ref0 = circ_mean_mod(pwmv, (uint8_t)(2U * pp), ZERO_IDF_PWM_PER_PP, (int32_t *)0);


    hyst_abz = circ_dist(cf, cb, ZERO_IDF_COUNTS_PER_PP);
    hyst_pwm = circ_dist(pf, pb, ZERO_IDF_PWM_PER_PP);
    g_enc_foc.zero_hyst_cnt = (uint16_t)hyst_abz;

    /* PWM 方向自学习: 正扫PP点raw(全量域) 相邻差累加, 符号即方向 */
    dsum = 0;
    for (i = 0U; (i + 1U) < pp; i++) {
        int32_t d = BestA_u.PWM_cnt_buf[i + 1U] - BestA_u.PWM_cnt_buf[i];
        if (d >  2048) d -= 4096;
        if (d < -2048) d += 4096;
        dsum += d;
    }
    g_enc_foc.pwm_dir = (dsum < 0) ? 1U : 0U;

    if ((BestA_u.qual_fail == 0U)
        && (hyst_abz   <= ZERO_IDF_HYST_MAX_CNT)
        && (hyst_pwm   <= ZERO_IDF_HYST_MAX_PWM)
        && (spread_abz <= ZERO_IDF_SPREAD_MAX_CNT)) {
        /* 成功: 写零位偏移 + 学习 PWM 绝对基准, 请求主循环写入 Flash */
        off16 = ((int64_t)c0 * 65536LL) / (int64_t)ZERO_IDF_COUNTS_PER_PP+ ZERO_IDF_FIXED_OFFSET_16;
        g_enc_foc.zero_offset   = (int32_t)(off16 & 0xFFFFLL);
        g_enc_foc.zero_state    = 1U;
        g_enc_foc.pwm_zero_ref  = (uint16_t)ref0;
        g_enc_foc.pwm_ref_valid = 1U;
        g_zero_ref_save_pending = 1U;
        DW.comm_contrl.zero_identify_state = ST_IDF_ZERO_WAIT;
    } else {
        ENC_ZeroIdf_StopFailed();
    }

    /* 清理状态机 */
    g_enc_foc.z_clear = 0U;
    BestA_u.Z_flag    = 0U;
    BestA_u.PP_cnt    = 0U;
    BestA_u.sweep_dir = 0U;
    EncPwmSamp_Abort();
}

/* IF控制 + Z相检测, 10kHz调用 */
static void ENC_ZeroIdf_IfTask(void)
{
    EncPwmSampResult_t res;
    /* 运行状态改 file-scope (s_idf_*), 由 ENC_ZeroIdf_Reset 在启动辨识前复位 */
    /* ---- 步骤1: Z相检测 (找到2次Z相脉冲后才开始记录) ---- */
		g_enc_foc.z_clear=1;
		if (BestA_u.Z_flag < 3U) {
        ZPhase_Process();
        if (BestA_u.Z_flag == 0U || BestA_u.Z_flag == 2U) {
            /* 还没找到Z相 或 第1次已通过等待第2次 */
            if (g_enc_foc.z_phase_state == Z_LOCKOUT) {
                if (BestA_u.Z_flag == 0U) {
                    BestA_u.Z_flag = 1U;  /* 找到第1次Z相 */
                } else {
                    BestA_u.Z_flag = 3U;  /* 找到第2次Z相 --- 防止误触发 */
                }
                BestA_u.PP_cnt = 0U;      /* 极对数计数清零 */
            }
        } else if (BestA_u.Z_flag == 1U && g_enc_foc.z_phase_state == Z_IDLE) {
            BestA_u.Z_flag = 2U;          /* 第1次脉冲结束, 等待第2次 */
        }
    }

    /* ---- 步骤2: IF 慢旋 + 电角度步进 (10kHz载波节拍), 正/反双向各扫一圈 ---- */
    if (s_idf_i_cnt == s_idf_i_target) {
        s_idf_i_cnt = 0U;

        /* 电角度步进 (12bit 虚拟角度 0~4095): 正扫 +delta, 反扫 -delta */
        if (BestA_u.sweep_dir == 0U) {
            BestA_u.Angle_buff = (BestA_u.Angle_buff + BestA_u.Angle_delta) & 0x0FFFU;
        } else {
            BestA_u.Angle_buff = (BestA_u.Angle_buff + 4096U - BestA_u.Angle_delta) & 0x0FFFU;
        }

        /* 将虚拟角度写入 FOC 控制变量, 覆盖编码器读数 */
        DW.motor_main_feedback.elec_angle = BestA_u.Angle_buff;

        /* 稳态等待结束 -> 记录本零位点 ABZ/PWM, 恢复正常步进 */
        if (s_idf_wait_steady == 1U) {
            s_idf_wait_steady = 0U;
            s_idf_i_target    = 10U;  /* 恢复较快步进 */

            if ((BestA_u.Z_flag == 3U) && (BestA_u.PP_cnt > 0U)
                && (BestA_u.PP_cnt <= (uint16_t)g_enc_foc.bElToMecRatio)) {
                /* 前PP项记录正扫, 后PP项记录反扫 */
                uint8_t idx = (uint8_t)(BestA_u.PP_cnt - 1U);
                if (BestA_u.sweep_dir != 0U) {
                    idx = (uint8_t)(idx + (uint8_t)g_enc_foc.bElToMecRatio);
                }
                BestA_u.TIM_cnt_buf[idx] = (int32_t)(ENC_TIM->CNT);
                if (EncPwmSamp_Stop(&res, ZERO_IDF_PWM_SPREAD_MAX) != 0U) {
                    BestA_u.PWM_cnt_buf[idx] = (int32_t)res.raw_avg;
                } else {
                    ENC_ZeroIdf_StopFailed();
                    return;
                }

                if (BestA_u.PP_cnt == (uint16_t)g_enc_foc.bElToMecRatio) {
                    if (BestA_u.sweep_dir == 0U) {
                        /* 正扫完成 -> 换反扫再采一圈 (正反平均消摩擦残差) */
                        BestA_u.sweep_dir = 1U;
                        BestA_u.PP_cnt    = 0U;
                    } else {
                        /* 双扫完成 -> 合成零位/PWM基准, 判质量, 交接状态 */
                        ENC_ZeroIdf_Synth();
                    }
                }
            }
        }

        /* 电角度过零 -> 触发稳态等待 + 极对计数 */
        if ((BestA_u.Angle_buff % 4096U) == 0U) {
            if (s_idf_wait_steady == 0U) {
                BestA_u.PP_cnt++;
                s_idf_wait_steady = 1U;
                s_idf_i_target    = 15000U;  /* 等待 1500ms @10kHz, 让电流稳定 */
            }
        }
    } else {
        s_idf_i_cnt++;
        /* 稳态等待满 1.0s 后开始 PWM raw 采样 (取后 0.5s 窗口, 避开收敛初期振荡) */
        if ((s_idf_wait_steady == 1U) && (s_idf_i_cnt == ZERO_IDF_PWM_SAMP_DELAY)) {
            EncPwmSamp_Start();
        }
        if (EncPwmSamp_IsActive() != 0U) {
            EncPwmSamp_Poll();
        }
    }
}
#else
static void ENC_ZeroIdf_VqScanSynth(void)
{
    uint16_t index;
    uint16_t angle12;
    int32_t pwm_domain;
    int32_t pwm_delta;
    int32_t pwm_ref;
    int64_t off16;

    if ((BestA_u.qual_fail == 0U) && (s_vq_dir_vote != 0)) {
        index = (uint16_t)((BestA_u.Vq_max_index + ZERO_VQ_SCAN_POINTS
                            - ZERO_VQ_SCAN_BACKOFF) % ZERO_VQ_SCAN_POINTS);
        angle12 = (uint16_t)(((uint32_t)index * 4096U) / ZERO_VQ_SCAN_POINTS);
        g_enc_foc.pwm_dir = (s_vq_dir_vote < 0) ? 1U : 0U;
        pwm_domain = (int32_t)EncFoc_PwmDomain();
        pwm_delta = (int32_t)(((uint32_t)angle12 * (uint32_t)pwm_domain) / 4096U);
        if (g_enc_foc.pwm_dir == 0U) {
            pwm_ref = (int32_t)s_vq_best_pwm - pwm_delta;
        } else {
            pwm_ref = (int32_t)s_vq_best_pwm + pwm_delta;
        }

        off16 = ((int64_t)(int32_t)s_vq_best_cnt * 65536LL)
                / (int64_t)ZERO_IDF_COUNTS_PER_PP
                - ((int32_t)angle12 << 4) + ZERO_IDF_FIXED_OFFSET_16;
        g_enc_foc.zero_offset   = (int32_t)(off16 & 0xFFFFLL);
        g_enc_foc.zero_state    = 1U;
        g_enc_foc.z_first_state = 1U;
        g_enc_foc.pwm_zero_ref  = (uint16_t)mod_norm(pwm_ref, pwm_domain);
        g_enc_foc.pwm_ref_valid = 1U;
        g_enc_foc.zero_hyst_cnt = 0U;
        g_zero_ref_save_pending = 1U;
        DW.comm_contrl.zero_identify_state = ST_IDF_ZERO_WAIT;
    } else {
        DW.comm_contrl.zero_identify_state = ST_IDF_ZERO_FIRL;
    }
    g_enc_foc.z_clear = 0U;
    EncPwmSamp_Abort();
}

/* Idq电流闭环扫描: 每点60拍, 后40拍平均, 取Vd近零且正转时的最大Vq */
static void ENC_ZeroIdf_VqScanTask(void)
{
    BestA_u.Average_CNT++;
    if (BestA_u.Average_CNT == ZERO_VQ_SCAN_PERIOD) {
        uint32_t cnt = ENC_TIM->CNT;
        uint16_t pwm = g_enc.raw;
        int32_t d_cnt = 0;
        int32_t d_pwm;
        float vd_limit = (DW.motor_anal.bus_Volt < ZERO_VQ_SCAN_BUS_THRESHOLD)
                         ? ZERO_VQ_SCAN_VD_MAX_LOW : ZERO_VQ_SCAN_VD_MAX_HIGH;

        BestA_u.JILU_Vd *= 0.025F;
        BestA_u.JILU_Vq *= 0.025F;
        BestA_u.Average_CNT = 0U;
        BestA_u.Best_Angle_CNT++;

        if (s_vq_have_last != 0U) {
            d_cnt = (int32_t)(cnt - s_vq_last_cnt);
            d_pwm = (int32_t)pwm - (int32_t)s_vq_last_pwm;
            if (d_pwm > 2048) { d_pwm -= 4096; }
            if (d_pwm < -2048) { d_pwm += 4096; }
            if ((d_cnt > 0) && (d_pwm != 0)) {
                s_vq_dir_vote += ((d_cnt > 0) == (d_pwm > 0)) ? 1 : -1;
            }
        }
        s_vq_last_cnt = cnt;
        s_vq_last_pwm = pwm;
        s_vq_have_last = 1U;

        if ((d_cnt > 0) && (BestA_u.JILU_Vq > BestA_u.Vq_max)
            && (fabsf(BestA_u.JILU_Vd) < vd_limit)) {
            BestA_u.Vq_max = BestA_u.JILU_Vq;
            BestA_u.Vq_max_index = BestA_u.Best_Angle_CNT;
            BestA_u.qual_fail = 0U;
            s_vq_best_cnt = cnt;
            s_vq_best_pwm = pwm;
        }
        BestA_u.JILU_Vd = 0.0F;
        BestA_u.JILU_Vq = 0.0F;

        if (BestA_u.Best_Angle_CNT >= ZERO_VQ_SCAN_POINTS) {
            ENC_ZeroIdf_VqScanSynth();
            return;
        }
        DW.motor_main_feedback.elec_angle = BestA_u.Best_Angle_CNT & 0x0FFFU;
    } else if (BestA_u.Average_CNT > ZERO_VQ_SCAN_SKIP) {
        BestA_u.JILU_Vd += DW.motor_anal.Vd_buf;
        BestA_u.JILU_Vq += DW.motor_anal.Vq_buf;
    }
}
#endif

void ENC_ZeroIdf_Task(void)
{
    if (DW.comm_contrl.zero_identify_state != ST_IDF_ZERO_RUN) { return; }
    EncFoc_ApplyConfig();
    if ((DW.motor_param.sensor_type == 1U) ||
        (DW.motor_param.PP_num < 1U) || (DW.motor_param.PP_num > ZERO_IDF_MAX_PP) ||
        (s_zero_cfg_pp != DW.motor_param.PP_num) ||
        (s_zero_cfg_lines != DW.motor_param.enc_line_count)) {
        /* 霍尔模式、超过16极对或扫描中改动配置: 禁止混用该ABZ找零流程 */
        ENC_ZeroIdf_StopFailed();
        return;
    }
#if (ZERO_IDF_ALGORITHM == ZERO_IDF_ALG_IF_SWEEP)
    if (BestA_u.qual_fail != 0U) {
        ENC_ZeroIdf_StopFailed();
        return;
    }
    ENC_ZeroIdf_IfTask();
#else
    ENC_ZeroIdf_VqScanTask();
#endif
}

/* ********************* 上电快速恢复 (PWM 绝对位置 + 已学习基准) *********************/
/* 上电快速恢复: 转子静止时采一次 PWM raw, 由已学习的 pwm_zero_ref/pwm_dir 反算 zero_offset
   1) r = raw mod (4096/PP)  2) d = (pwm_dir==0 ? r-ref : ref-r) mod (4096/PP)
   3) elec16 = d × 65536/(4096/PP)  4) zero_offset = elAngle_now - elec16 + 固定90°
   仅在ABZ模式、PP不超过16且转子静止时调用 (静止守卫由调用方负责) */
uint8_t ENC_ZeroOffset_FromPwmRaw(uint16_t raw_avg)
{
    int32_t d;
    int32_t r;
    uint16_t domain;
    int16_t elec_true, el_now;

    EncFoc_ApplyConfig();
    if ((DW.motor_param.sensor_type == 1U) ||
        (DW.motor_param.PP_num < 1U) || (DW.motor_param.PP_num > 64U) ||
        (raw_avg > 4095U)) {
        return 0U;
    }
    domain = EncFoc_PwmDomain();
    if ((domain == 0U) || (g_enc_foc.pwm_zero_ref >= domain)) { return 0U; }

    r = (int32_t)(raw_avg % domain);
    if (g_enc_foc.pwm_dir == 0U) {
        d = r - (int32_t)g_enc_foc.pwm_zero_ref;
    } else {
        d = (int32_t)g_enc_foc.pwm_zero_ref - r;
    }
    if (d < 0) { d += domain; }

    elec_true = (int16_t)((d * 65536) / (int32_t)domain);
    el_now    = (int16_t)(g_enc_foc.hMecAngle * (int16_t)g_enc_foc.bElToMecRatio);

    /* 16bit 算术自然模 65536, 自动处理过零 */
    g_enc_foc.zero_offset   = (int32_t)(uint16_t)(el_now - elec_true) + ZERO_IDF_FIXED_OFFSET_16;
    g_enc_foc.pwm_ref_valid = 1U;
    g_enc_foc.zero_state    = 1U;
    g_enc_foc.z_first_state = 1U;
    return 1U;
}
/* 手动零位基准: 无需先自动找零, 由 CAN 下发新的 pwm_zero_ref 后,
   在线程里用当前静止 PWM raw 反算 zero_offset。
   pwm_dir沿用已学习值或默认0; 成功后置pwm_ref_valid=1并允许落盘。 */
uint16_t EncFoc_PwmDomain(void)   /* PWM 原始值每极对刻度 = 4096/极对数 */
{
    uint16_t pp = (uint16_t)g_enc_foc.bElToMecRatio;
    if ((pp < 1U) || (pp > 64U)) { pp = 4U; }
    return (uint16_t)(4096U / pp);
}

uint8_t ENC_ManualZeroRef_Apply(uint16_t raw_ref, uint16_t raw_now)
{
    uint16_t old_ref = g_enc_foc.pwm_zero_ref;

    EncFoc_ApplyConfig();
    if ((DW.motor_param.sensor_type == 1U) ||
        (DW.motor_param.PP_num < 1U) || (DW.motor_param.PP_num > 64U)) { return 0U; }
    if (raw_ref >= EncFoc_PwmDomain()) { return 0U; }   /* 域随传感器极对数 */

    g_enc_foc.pwm_zero_ref = raw_ref;                   /* 新基准 */
    if (ENC_ZeroOffset_FromPwmRaw(raw_now) == 0U) {
        g_enc_foc.pwm_zero_ref = old_ref;               /* 反算失败: 回滚 */
        return 0U;
    }
    g_enc_foc.zero_hyst_cnt = 0U;                       /* 基准已变: 迟滞监控清零重计 */
    DW.motor_main_feedback.elec_angle_offset = (uint16_T)(raw_ref & 0xFFFFU);
    return (raw_ref != old_ref) ? 1U : 0U;
}

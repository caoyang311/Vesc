/******************************************************************************
 * 文件: foc_ctrl.c
 * 功能: FOC 核心控制 (Clarke/Park/电流环/弱磁/MTPA/查表/iPark/SVPWM)
 * 创建日期: 2026-06-22
 * 修改日期: 2026-10-29
 * 修改内容: 母线反算按上一拍调制矢量和反馈坐标对齐; 弱磁按实际电压利用率闭环;
 *           单层功率与电压限制连续收放, 删除自动零矢量和零速/负转速退力;
 *           调制比例与回构同源可选, 当前为UNITY验证配置; 普通电流/母线限幅不重复降速;
 *           d轴过冲时d/q同步限流; 统一电流圆/参数有效性/模式复位
 * 修改人: rui.zhang
 ******************************************************************************/

/* PI 就地操作 PI_Params; sin/cos 用 SinCos_Table 查表; Clarke/Park/iPark/SVPWM 内联展开。
 * d/q: pi_ds=(Id_ref,Id_fbk)->Vd, pi_qs=(Iq_ref,Iq_fbk)->Vq, 各轴积分独立。 */

#include "foc_ctrl.h"
#include "foc_data.h"
#include "foc_pi.h"
#include "Common_Math.h"
#include "Axis_transform.h"
#include "SVPWM_tiaozhi.h"
#include "MTPV_MAP.h"   /* MAP 查表法 (contorl_mode_sub==2) */
#include "encoder_foc_sdk.h"    /* ENCODER_REVERSE_ENABLE: ABZ默认方向 */
#include "tim.h"                /* htim1: FOC_STOP/motor_start 控制 PWM 输出 */
#include <math.h>
#include <string.h>
/********************* 全局数据实例 ******************** */
FOC_Ctrl_Data DW;
volatile uint8_t g_foc_param_invalid = 0U;

/* 变换中间量结构体 (全局, 便于调试器观察; FOC_Ctrl_Init 中清零) */
CLARKE clarke_u;   /* Clarke: Us/Vs/Ws -> Alpha/Beta */
PARK   park_u;     /* Park : Alpha/Beta + Sine/Cosine -> Ds/Qs */
IPARK  ipark_u;    /* iPark: Ds/Qs + Sine/Cosine -> Alpha/Beta */
SVPWM  svpwm_u;    /* SVPWM: Ualpha/Ubeta -> Ta/Tb/Tc -> SVPTa/b/c */

/* SVPWM 扇区常量 (MATLAB ConstB.Add_aw) — 改用 SVPWM_Cale 等价写法后不再需要 */

/* 速度斜坡梯度: 需跨控制周期保持 (替代 MATLAB DW.Merge_o) */
static real32_T s_grad_buf = 0.0F;
/* 转矩/保持类模式的指令保持值(限幅后): 不再回读 PI.Ref, 避免限幅被逐拍累乘 */
static real32_T s_hold_id_cmd = 0.0F;
static real32_T s_hold_iq_cmd = 0.0F;
/* 下游限制状态: 汇总标志供速度环条件积分, 分项标志供可达性调速器 */
static uint8_t s_feas_clamp_act = 0U;
static uint8_t s_volt_clamp_act = 0U;
static uint8_t s_circ_clamp_act = 0U;
static uint8_t s_phase_clamp_act = 0U;
static uint8_t s_pwr_d_excess = 0U;
static uint8_t s_pwr_fb_bad = 0U;
static uint8_t s_volt_exhausted = 0U;
static uint8_t s_bus_guard_act = 0U;
static real32_T s_util_lpf = 0.0F;
static real32_T s_fw_prev = 0.0F;
static real32_T s_pwr_q_scale = 1.0F;
static real32_T s_volt_q_scale = 1.0F;
static real32_T s_valpha_applied = 0.0F;
static real32_T s_vbeta_applied = 0.0F;
static real32_T s_we_fb_f = 0.0F;
static real32_T s_iscale = 1.0F;
static uint16_T s_cmode_prev = 0xFFFFU;
static uint16_T s_sub_prev = 0xFFFFU;
#if (CURR_FB_FILT_EN != 0)
static real32_T s_id_f = 0.0F;
static real32_T s_iq_f = 0.0F;
#endif
/* ******************** 静态辅助 ******************** */

/* 输出归一化与CCR回构共用同一比例 */
static inline real32_T foc_mod_vscale(real32_T vbus)
{
#if (FOC_MOD_SCALE_MODE == FOC_MOD_SCALE_UNITY)
    return 0.5F * vbus;
#else
    return MATH_1_SQRT3 * vbus;
#endif
}

/* 电压裕量(占"有效电压天花板"的比例): 天花板 = min(线性调制极限, 占空比钳位换算)。
   占空比钳位是逐相的, 指令越过它先被截成谐波(矢量畸变, 不是干净限幅) —— 椭圆与弱磁
   目标都必须落在它之下才真实有效; 占空比钳位宏一改, 此处整套自动重整。
   成对关系: 椭圆门槛 = 天花板*(1-margin); 弱磁目标 = 天花板*(1-2*margin)。
   改裕量只动 FW_MARGIN_PCT */
#define PWM_DUTY_VFRAC  (((real32_T)PWM_DUTY_MAX_LIMIT - (real32_T)(PWM_TIMER_PERIOD / 2U))                          / ((real32_T)(PWM_TIMER_PERIOD / 2U)))
#define FW_VEFF_VFRAC    ((PWM_DUTY_VFRAC < 1.0F) ? PWM_DUTY_VFRAC : 1.0F)
#define FW_MARGIN_PCT   0.05F
#define FW_ELL_VFRAC    (FW_VEFF_VFRAC * (1.0F - FW_MARGIN_PCT))
#define FW_UTIL_REF     (0.577350259F * FW_VEFF_VFRAC * (1.0F - 2.0F * FW_MARGIN_PCT))
/* 角度分数(0~1 圈) -> sin/cos, 复用 SinCos_Table(12bit)。
 * (int32)截断 + &0x0FFF 对正/负分数都按模 4096 折叠。 */
static void foc_sincos_frac(real32_T frac, real32_T *s, real32_T *c)
{
    Ang_SinCos sc;
    sc.table_Angle = (uint16_T)((int32_T)(frac * 4096.0F) & 0x0FFF);
    SinCos_Table(&sc);
    *s = sc.table_Sin;
    *c = sc.table_Cos;
}

/* ******************** 可达性调速器 ********************
   电压不可达、母线预算被 d 轴耗尽或反馈异常时降低速度参照。
   普通电流/母线闭环直接限制 q，不在这里重复退让；调速器不直接缩放 Id/Iq。
   50ms 漏泄确认接管, 100ms 漏泄确认释放(预算耗尽时立即接管)。 */
#if (PWR_GOV_EN != 0)
static uint8_t  s_gov_bind = 0U;
static uint8_t  s_gov_hold = 0U;
static real32_T s_gov_ref = 0.0F;
static real32_T s_gov_red = 0.0F;
static real32_T s_gov_on_t = 0.0F;
static real32_T s_gov_off_t = 0.0F;
static uint8_T  s_gov_init = 0U;
static real32_T PwrGov_Ref(real32_T ref_cmd, real32_T dt)
{
    real32_T sgn, mag_cmd, mag_ref, mag_target, ebus, delta, max_dn, max_up;
    uint8_t motoring;

    if (dt < 1.0e-5F) { dt = 2.0e-4F; }
    sgn = (ref_cmd >= 0.0F) ? 1.0F : -1.0F;
    mag_cmd = ref_cmd * sgn;
    if (s_gov_init == 0U) { s_gov_init = 1U; s_gov_ref = ref_cmd; }
    mag_ref = s_gov_ref * sgn;

    if ((mag_cmd < mag_ref) || ((ref_cmd * s_gov_ref) < 0.0F)) {
        s_gov_ref = ref_cmd;
        mag_ref = mag_cmd;
        s_gov_red = 0.0F;
        s_gov_hold = 0U;
        s_gov_on_t = 0.0F;
        s_gov_off_t = 0.0F;
    }

    /* 电流圆、相电流和母线闭环已直接约束 q，不再重复压低速度参照。
       只有电压不可达、母线预算被 d 轴耗尽或反馈异常时才退让速度。 */
    s_gov_bind = (s_volt_clamp_act || s_pwr_d_excess || s_pwr_fb_bad) ? 1U : 0U;
    /* 物理预算耗尽、反馈异常或电压域耗尽时立即接管, 不等普通钳位确认窗 */
    if ((s_pwr_d_excess != 0U) || (s_pwr_fb_bad != 0U) ||
        (s_volt_exhausted != 0U)) { s_gov_on_t = 0.05F; }
    motoring = ((ref_cmd * (real32_T)DW.motor_main_feedback.mech_Spd) > 0.0F) ? 1U : 0U;
    if (motoring != 0U) {
        if (s_gov_bind != 0U) {
            s_gov_on_t += dt;
            s_gov_off_t -= 4.0F * dt;
        } else {
            s_gov_on_t -= 4.0F * dt;
            s_gov_off_t += dt;
        }
    } else {
        s_gov_on_t = 0.0F;
        s_gov_off_t = 0.0F;
        s_gov_hold = 0U;
    }
    if (s_gov_on_t < 0.0F) { s_gov_on_t = 0.0F; }
    else if (s_gov_on_t > 0.05F) { s_gov_on_t = 0.05F; }
    if (s_gov_off_t < 0.0F) { s_gov_off_t = 0.0F; }
    else if (s_gov_off_t > 0.10F) { s_gov_off_t = 0.10F; }
    if ((motoring != 0U) && (s_gov_on_t >= 0.05F)) { s_gov_hold = 1U; }
    else if (s_gov_off_t >= 0.10F) { s_gov_hold = 0U; }

    if ((motoring != 0U) && (s_gov_hold != 0U) && (s_gov_bind != 0U)) {
        if ((DW.sys_param.bus_curr_max > 0U) &&
            (g_bus_curr_status == BUS_FB_VALID)) {
            ebus = g_bus_curr_fast - (real32_T)DW.sys_param.bus_curr_max;
        } else {
            ebus = 0.0F;
        }
        s_gov_red += ((ebus > 5.0F) ? 250.0F : 50.0F) * dt;
    } else if (s_gov_hold == 0U) {
        s_gov_red -= 120.0F * dt;
    }
    if (s_gov_red < 0.0F) { s_gov_red = 0.0F; }
    if (s_gov_red > mag_cmd) { s_gov_red = mag_cmd; }
    mag_target = mag_cmd - s_gov_red;

    delta = mag_target - mag_ref;
    max_dn = 300.0F * dt;
    /* 正常目标上升使用配置的加速斜率; s_gov_red 的恢复速率仍独立为120rpm/s */
    max_up = DW.comm_contrl.speed_Ke_Acc * dt;
    if (max_up < 0.0F) { max_up = 0.0F; }
    if (delta < -max_dn) { delta = -max_dn; }
    else if (delta > max_up) { delta = max_up; }
    s_gov_ref = (mag_ref + delta) * sgn;
    return s_gov_ref;
}
#endif

/* 电流圆半径(A): out_curr_max(上位机配置, 0=板极限600A) 与 comm_contrl.I_max 合并取小;
   I_max>0 时参与, 0 时忽略. 该圆同时约束 MTPA/弱磁 id 上限和电压椭圆可用 iq. */
float FOC_CurrentCircle(void)
{
    real32_T lim = 600.0F;
    if (DW.sys_param.out_curr_max > 0U) {
        lim = (real32_T)DW.sys_param.out_curr_max;
    }
    if ((DW.comm_contrl.I_max > 0.0F) && (DW.comm_contrl.I_max < lim)) {
        lim = DW.comm_contrl.I_max;
    }
    if (lim > 600.0F) { lim = 600.0F; }
    if (lim < 1.0F) { lim = 1.0F; }
    return lim;
}

static real32_T foc_flux_ld_limit(void)
{
    static real32_T flux_prev = -1.0F;
    static real32_T ld_prev = -1.0F;
    static real32_T limit = 0.0F;
    real32_T flux = DW.motor_param.flux_value;
    real32_T ld = DW.motor_param.Ld_value;

    if ((flux != flux_prev) || (ld != ld_prev)) {
        limit = ((flux > 0.0F) && (ld > 1.0E-7F)) ? (flux / ld) : 0.0F;
        flux_prev = flux;
        ld_prev = ld;
    }
    return limit;
}

void FOC_MotorParams_Poll(void)
{
    static uint16_t pp_prev = 0U;
    static real32_T emf_prev = -1.0F;
    uint16_t pp = DW.motor_param.PP_num;
    real32_T emf = DW.motor_param.EMF_constant;

    if (pp_prev == 0U) {
        pp_prev = pp;
        emf_prev = emf;
        return;
    }
    if ((pp != pp_prev) || (emf != emf_prev)) {
        if ((pp > 0U) && (emf > 0.0F) && (emf < 1000000.0F)) {
            DW.motor_param.flux_value = (emf * 0.0008165F) /
                                        ((real32_T)pp * 0.10472F);
        }
        pp_prev = pp;
        emf_prev = emf;
    }
}

static uint8_t foc_motor_params_valid(void)
{
    if ((DW.motor_param.PP_num < 1U) || (DW.motor_param.PP_num > 64U)) { return 0U; }
    if (!((DW.motor_param.Ld_value > 1.0E-7F) &&
          (DW.motor_param.Ld_value < 1000000.0F))) { return 0U; }
    if (!((DW.motor_param.Lq_value > 1.0E-7F) &&
          (DW.motor_param.Lq_value < 1000000.0F))) { return 0U; }
    if (!((DW.motor_param.flux_value > 0.0F) &&
          (DW.motor_param.flux_value < 1000000.0F))) { return 0U; }
    if (!((DW.motor_param.Rs_value >= 0.0F) &&
          (DW.motor_param.Rs_value < 1000000.0F))) { return 0U; }
    return 1U;
}

/* 实时 Ld/Lq 估算(仅诊断): 用上一拍实际调制电压、同坐标电流反馈和电机参数估算;
   结果经两级 LPF 写 DW.motor_anal.Ld_est/Lq_est, 不改控制、不改表。
   电压含死区/管压降误差, 只作趋势/调试, 不直接填饱和表。 */
void FOC_LdLq_Estimate(void)
{
    static real32_T s_ld = 0.0412F;
    static real32_T s_lq = 0.0981F;
    static real32_T s_ld_fast = 0.0412F;
    static real32_T s_lq_fast = 0.0981F;
    real32_T we = fabsf((real32_T)DW.motor_main_feedback.mech_Spd)
                  * (real32_T)DW.motor_param.PP_num * 0.104719755F;
    real32_T id = DW.pi_ds.Fbk;
    real32_T iq = DW.pi_qs.Fbk;
    real32_T vd = DW.motor_anal.Vd_buf;
    real32_T vq = DW.motor_anal.Vq_buf;
    real32_T rs = DW.motor_param.Rs_value;
    real32_T psi = DW.motor_param.flux_value;
    real32_T den_d, den_q, ld_raw, lq_raw;

    DW.motor_anal.LdLq_est_valid = 0U;
    if ((DW.motor_anal.Ld_est <= 0.0F) || (DW.motor_anal.Lq_est <= 0.0F)) {
        DW.motor_anal.Ld_est = s_ld;
        DW.motor_anal.Lq_est = s_lq;
        s_ld_fast = s_ld;
        s_lq_fast = s_lq;
    }

    /* 低速/小电流时分母病态, 不更新 */
    if ((we < 200.0F) || (fabsf(id) < 30.0F) || (fabsf(iq) < 30.0F)) { return; }
    den_d = we * id;
    den_q = we * iq;
    if ((fabsf(den_d) < 1.0F) || (fabsf(den_q) < 1.0F)) { return; }

    lq_raw = (rs * id - vd) / den_q;
    ld_raw = (vq - rs * iq - (we * psi)) / den_d;

    /* 合理性窗口 + 凸极方向校验; 数据18 的指令电压反算会出现 Ld>Lq, 这类点丢弃 */
    if ((ld_raw < 1.0e-6F) || (ld_raw > 500.0e-6F) ||
        (lq_raw < 1.0e-6F) || (lq_raw > 500.0e-6F) ||
        (lq_raw <= ld_raw)) {
        return;
    }

    /* 单点尖峰限幅: 相对快级值超过 ±20% 的原始值先夹住, 再进 LPF */
    if (ld_raw > (s_ld_fast * (1.0F + LDLQ_EST_OUTLIER_FRAC))) {
        ld_raw = s_ld_fast * (1.0F + LDLQ_EST_OUTLIER_FRAC);
    } else if (ld_raw < (s_ld_fast * (1.0F - LDLQ_EST_OUTLIER_FRAC))) {
        ld_raw = s_ld_fast * (1.0F - LDLQ_EST_OUTLIER_FRAC);
    }
    if (lq_raw > (s_lq_fast * (1.0F + LDLQ_EST_OUTLIER_FRAC))) {
        lq_raw = s_lq_fast * (1.0F + LDLQ_EST_OUTLIER_FRAC);
    } else if (lq_raw < (s_lq_fast * (1.0F - LDLQ_EST_OUTLIER_FRAC))) {
        lq_raw = s_lq_fast * (1.0F - LDLQ_EST_OUTLIER_FRAC);
    }

    /* 两级 LPF: 快级抑制逐拍噪声, 慢级给观测/上位机平滑值 */
    s_ld_fast += LDLQ_EST_FAST_ALPHA * (ld_raw - s_ld_fast);
    s_lq_fast += LDLQ_EST_FAST_ALPHA * (lq_raw - s_lq_fast);
    s_ld += LDLQ_EST_SLOW_ALPHA * (s_ld_fast - s_ld);
    s_lq += LDLQ_EST_SLOW_ALPHA * (s_lq_fast - s_lq);

    DW.motor_anal.Ld_est = s_ld;
    DW.motor_anal.Lq_est = s_lq;
    DW.motor_anal.LdLq_est_valid = 1U;
}




static real32_T pwr_temp_scale(real32_T temp, real32_T t_high, real32_T t_ov)
{
    if (t_ov <= t_high)      { return 1.0F; }
    if (temp <= t_high)      { return 1.0F; }
    if (temp >= t_ov)        { return 0.01F; }
    return 1.0F - 0.99F * ((temp - t_high) / (t_ov - t_high));
}

/* 快速母线反算闭环: 输出 Is_cap, 不使用电机参数或饱和表。 */
static real32_T s_is_cap = -1.0F;

static real32_T foc_pwr_cap(void)
{
    real32_T circ = FOC_CurrentCircle();
    real32_T cap_min = (circ < PWR_CL_MIN_CAP) ? circ : PWR_CL_MIN_CAP;
    if ((!((s_is_cap < 1000000.0F) && (s_is_cap > -1000000.0F))) ||
        (s_is_cap < 0.0F)) { s_is_cap = circ; }
    if (s_is_cap > circ) { s_is_cap = circ; }
    if (s_is_cap < cap_min) { s_is_cap = cap_min; }
    return s_is_cap;
}

static real32_T foc_q_scale_step(real32_T state, real32_T target)
{
    real32_T rate;
    if (!((state >= 0.0F) && (state <= 1.0F))) { state = 1.0F; }
    if (!((target >= 0.0F) && (target <= 1.0F))) {
        target = (target > 1.0F) ? 1.0F : 0.0F;
    }
    rate = (target < state) ? FOC_Q_LIMIT_ATTACK : FOC_Q_LIMIT_RELEASE;
    state += rate * (target - state);
    if (state < 0.0F) { state = 0.0F; }
    else if (state > 1.0F) { state = 1.0F; }
    return state;
}

void foc_pwr_limit_single(real32_T *id_ref, real32_T *iq_ref,
                          real32_T spd, real32_T vbus)
{
    real32_T ilim, err, id_abs, iq_lim, target, iq_abs;
    uint8_t driving;
    s_pwr_fb_bad = 0U;
    if ((id_ref == 0) || (iq_ref == 0)) {
        s_bus_guard_act = 0U;
        return;
    }
    driving = ((spd >= 0.0F) && (*iq_ref > 0.0F)) ||
              ((spd < 0.0F) && (*iq_ref < 0.0F));
    ilim = (DW.sys_param.bus_curr_max > 0U)
           ? (PWR_LIMIT_BUS_SCALE * (real32_T)DW.sys_param.bus_curr_max) : 0.0F;
    if ((ilim < 1.0F) || (vbus < 1.0F)) {
        s_is_cap = FOC_CurrentCircle();
        s_pwr_q_scale = 1.0F;
        s_bus_guard_act = 0U;
        return;
    }
    if (driving == 0U) {
        s_is_cap += PWR_CL_RELEASE * (FOC_CurrentCircle() - s_is_cap);
        s_pwr_q_scale = foc_q_scale_step(s_pwr_q_scale, 1.0F);
        s_bus_guard_act = 0U;
        return;
    }

    if (g_bus_curr_status == BUS_FB_STARTUP) {
        s_is_cap = FOC_CurrentCircle();
    } else {
        (void)foc_pwr_cap();
        err = g_bus_curr_fast - ilim;
        if ((g_bus_curr_status != BUS_FB_VALID) ||
            !((err < 1000000.0F) && (err > -1000000.0F))) {
            s_is_cap = (FOC_CurrentCircle() < PWR_CL_MIN_CAP)
                       ? FOC_CurrentCircle() : PWR_CL_MIN_CAP;
            s_pwr_fb_bad = 1U;
        } else if (err > PWR_CL_DEADBAND) {
            real32_T cut = (PWR_CL_KP + PWR_CL_INTEGRAL) * err;
            if (cut > PWR_CL_MAX_SLEW) { cut = PWR_CL_MAX_SLEW; }
            s_is_cap -= cut;
        } else if (err < 0.0F) {
            real32_T rel = (err < -PWR_CL_FAST_REL_TH) ? PWR_CL_FAST_REL : PWR_CL_RELEASE;
            s_is_cap += rel * (FOC_CurrentCircle() - s_is_cap);
        }
    }

    (void)foc_pwr_cap();
    id_abs = fabsf(*id_ref);
    iq_abs = fabsf(*iq_ref);
    target = 1.0F;
    if (((id_abs * id_abs) + (iq_abs * iq_abs)) > (s_is_cap * s_is_cap)) {
        iq_lim = ((s_is_cap > id_abs) && (s_is_cap > 0.0F))
                 ? sqrtf((s_is_cap * s_is_cap) - (id_abs * id_abs)) : 0.0F;
        target = (iq_abs > 1.0E-6F) ? (iq_lim / iq_abs) : 1.0F;
    }
    if ((g_bus_curr_status == BUS_FB_VALID) &&
        (s_is_cap < (0.995F * FOC_CurrentCircle())) && (s_is_cap <= id_abs)) {
        s_pwr_d_excess = 1U;
    }
    s_pwr_q_scale = foc_q_scale_step(s_pwr_q_scale, target);
    *iq_ref *= s_pwr_q_scale;
    s_bus_guard_act = ((target < 0.995F) || (s_pwr_q_scale < 0.995F) ||
                       (s_pwr_fb_bad != 0U)) ? 1U : 0U;
}

/* dq 电流反馈可选低通: 仅反馈, 不滤指令; 默认关闭, 开启会牺牲相位裕量. */
static void foc_curr_fb_filter(real32_T *id_fb, real32_T *iq_fb)
{
#if (CURR_FB_FILT_EN != 0)
    s_id_f += CURR_FB_FILT_ALPHA * ((*id_fb) - s_id_f);
    s_iq_f += CURR_FB_FILT_ALPHA * ((*iq_fb) - s_iq_f);
    *id_fb = s_id_f;
    *iq_fb = s_iq_f;
#endif
}

/* 实测相电流最终限制: 快降慢升, 不改故障/停机逻辑。
   高速电动且Id未过冲时保留Id、只缩Iq; |Id反馈|超过|Id指令|时d/q同步缩放,
   防止弱磁d轴过冲时只裁q而失去电流约束。 */
static void foc_phase_current_limit(real32_T *id_ref, real32_T *iq_ref)
{
    real32_T imeas = sqrtf((DW.pi_ds.Fbk * DW.pi_ds.Fbk) +
                           (DW.pi_qs.Fbk * DW.pi_qs.Fbk));
    real32_T imax  = FOC_CurrentCircle();
    if (imeas > imax) {
        real32_T k_meas = imax / imeas;
        if (k_meas < s_iscale) { s_iscale = k_meas; }
        else                   { s_iscale += 0.01F * (k_meas - s_iscale); }
    } else {
        s_iscale += 0.005F * (1.0F - s_iscale);
        if (s_iscale > 1.0F) { s_iscale = 1.0F; }
    }
    if (s_iscale < 0.999F) {
        if ((fabsf(DW.pi_ds.Fbk) > fabsf(*id_ref)) ||
            ((real32_T)DW.motor_main_feedback.mech_Spd <= 50.0F) || (*iq_ref <= 0.0F)) {
            *id_ref *= s_iscale;
            *iq_ref *= s_iscale;
        } else {
            *iq_ref *= s_iscale;
        }
        s_feas_clamp_act = 1U;
        s_phase_clamp_act = 1U;
    }
}

#if (VOLT_ELLIPSE_CLAMP_EN != 0)
/* 最终电流圆与电压可达性限制: 先修正不可行的 d, 再连续收放 q。 */
static void foc_volt_ellipse_clamp(real32_T *id_ref, real32_T *iq_ref)
{
    real32_T id = *id_ref;
    real32_T iq = *iq_ref;
    real32_T we, vlim, ld_w, lq_w, fl, vq_t, x;
    real32_T circ = FOC_CurrentCircle();
    real32_T id_pos_cap = 0.95F * circ;
    real32_T id_neg_cap = id_pos_cap;
    real32_T iqlim, iq_abs, target;

    s_volt_exhausted = 0U;
    {
        real32_T id_fw = foc_flux_ld_limit();
        if ((id_fw > 0.0F) && (id_fw < id_neg_cap)) { id_neg_cap = id_fw; }
    }
    if (id > id_pos_cap) { id = id_pos_cap; }
    else if (id < -id_neg_cap) { id = -id_neg_cap; }

    we = fabsf((real32_T)DW.motor_main_feedback.mech_Spd)
         * ((real32_T)DW.motor_param.PP_num * 0.104719755F);
    vlim = FW_ELL_VFRAC * DW.motor_param.Vdc_svpwm_buf;
    iq_abs = fabsf(iq);
    iqlim = iq_abs;
    if ((we >= 1.0F) && (vlim >= 1.0F)) {
        lq_w = Lookup_Lq(id, fabsf(iq)) * 1.0e-6F;
        ld_w = Lookup_Ld(id, fabsf(iq)) * 1.0e-6F;
        if (!(lq_w > 1.0e-7F)) { lq_w = DW.motor_param.Lq_value; }
        if (!(ld_w > 1.0e-7F)) { ld_w = DW.motor_param.Ld_value; }
        fl = DW.motor_param.flux_value + (ld_w * id);
        if ((ld_w > 1.0E-7F) && ((fl * we) > vlim)) {
            id = ((vlim / we) - DW.motor_param.flux_value) / ld_w;
            if (id < -id_neg_cap) { id = -id_neg_cap; }
            else if (id > id_pos_cap) { id = id_pos_cap; }
            fl = DW.motor_param.flux_value + (ld_w * id);
        }
        {
            real32_T fl_min = 0.25F * DW.motor_param.flux_value;
            if (fl < fl_min) { fl = fl_min; }
        }
        vq_t = we * fl;
        x = (vlim * vlim) - (vq_t * vq_t);
        if ((x > 0.0F) && (lq_w > 1.0E-9F)) {
            real32_T iq_cross = we * lq_w * iq_abs;
            if ((iq_cross * iq_cross) > x) {
                iqlim = sqrtf(x) / (we * lq_w);
            }
        } else {
            iqlim = 0.0F;
            s_volt_exhausted = 1U;
        }
    }

    /* 硬电流圆不经过平滑; 平滑只用于母线功率和电压可达性退让 */
    if (((id * id) + (iq * iq)) > (circ * circ)) {
        real32_T iq_circ2 = (circ * circ) - (id * id);
        real32_T iq_circ = (iq_circ2 > 0.0F) ? sqrtf(iq_circ2) : 0.0F;
        if (iq > iq_circ) {
            iq = iq_circ;
            s_feas_clamp_act = 1U;
            s_circ_clamp_act = 1U;
        } else if (iq < -iq_circ) {
            iq = -iq_circ;
            s_feas_clamp_act = 1U;
            s_circ_clamp_act = 1U;
        }
    }
    *id_ref = id;
    *iq_ref = iq;
    iq_abs = fabsf(iq);
    target = (iq_abs > 1.0E-6F) ? (iqlim / iq_abs) : 1.0F;
    if (target > 1.0F) { target = 1.0F; }
    s_volt_q_scale = foc_q_scale_step(s_volt_q_scale, target);
    if ((target < 0.995F) || (s_volt_q_scale < 0.995F)) {
        s_feas_clamp_act = 1U;
        s_volt_clamp_act = 1U;
    }
#if (VOLT_ELLIPSE_Q_CUT_EN != 0)
    *iq_ref *= s_volt_q_scale;
#else
    s_volt_q_scale = 1.0F;
#endif
}
#endif

/* 输出限幅: 仅执行电流圆和温度降额, 回馈功率由VCU统一管理。 */
float Pwr_Limit_Curr(float iq_dem)
{
    real32_T spd = (real32_T)DW.motor_main_feedback.mech_Spd;
    real32_T hi, lo;
    real32_T circ = FOC_CurrentCircle();

    hi = circ;
    {
        real32_T ks = pwr_temp_scale(DW.motor_anal.mos_temp,
                                     DW.faule_protection.over_Mos_temp_high_thresh,
                                     DW.faule_protection.over_Mos_temp_thresh);
        real32_T km = pwr_temp_scale(DW.motor_anal.motor_temp,
                                     DW.faule_protection.over_motor_temp_high_thresh,
                                     DW.faule_protection.over_motor_temp_thresh);
        hi *= (ks < km) ? ks : km;
    }
    lo = -circ;

    if (fabsf(spd) < 50.0F) {
        if (iq_dem >= 0.0F) {
            if (iq_dem >  hi) { iq_dem =  hi; }
            if (iq_dem <  lo) { iq_dem =  lo; }
        } else {
            if (iq_dem > -lo) { iq_dem = -lo; }
            if (iq_dem < -hi) { iq_dem = -hi; }
        }
    } else if (spd >= 0.0F) {
        if (iq_dem >  hi) { iq_dem =  hi; }
        if (iq_dem <  lo) { iq_dem =  lo; }
    } else {
        if (iq_dem > -lo) { iq_dem = -lo; }
        if (iq_dem < -hi) { iq_dem = -hi; }
    }
    return iq_dem;
}

/* 查表法 (contorl_mode_sub==2) 标准函数: (扭矩 N·m, |转速| rpm) -> (Id, Iq) 参考。
   负扭矩只取反 Iq, Id 不变(弱磁恒为负); 表数据与实际电流圆半径不一致时等比
   收缩防过流指令; 出口按母线预算再限一次 q */
static void foc_map_lookup(real32_T trq, real32_T spd, real32_T *id_out, real32_T *iq_out)
{
    real32_T t_  = fabsf(trq);
    real32_T s_  = fabsf(spd);
    real32_T id_ = Lookup_Id(t_, s_);
    real32_T iq_ = Lookup_Iq(t_, s_);

    if (trq < 0.0F) { iq_ = -iq_; }
    /* 电流圆与电压可达性在最终参考端统一处理, 不在此裁减弱磁 */
    iq_ = Pwr_Limit_Curr(iq_);
    *id_out = id_;
    *iq_out = iq_;
}


/* MTPA + 弱磁: 输入带符号矢量需求 Is_dem, 输出 Id/Iq 参考。
   弱磁幅值由电压利用率闭环决定; MTPA 角按需求与上一拍 q 指令的较小幅值计算。 */
static void foc_mtpa_flux(real32_T Is_dem, real32_T *id_out, real32_T *iq_out)
{
    real32_T util, fw, ldlq, cb, sinb, cosb, iq_s, sinfw, cosfw, idfw;
    real32_T iq_abs, iq_mtpa, id_cmd, iqlim, id2, imax2, id_max;
    real32_T lq_w = 0.0F, ld_w = 0.0F;

    /* 饱和电感按上一拍指令工作点查表, 无效表值回退到电机参数。 */
    {
        real32_T iq_abs_w = fabsf(DW.pi_qs.Ref);
        lq_w = Lookup_Lq(DW.pi_ds.Ref, iq_abs_w) * 1.0e-6F;
        ld_w = Lookup_Ld(DW.pi_ds.Ref, iq_abs_w) * 1.0e-6F;
        if (!(lq_w > 1.0e-7F)) { lq_w = DW.motor_param.Lq_value; }
        if (!(ld_w > 1.0e-7F)) { ld_w = DW.motor_param.Ld_value; }
    }

    if (DW.motor_anal.bus_Volt <= 1.0F) {
        *id_out = DW.pi_ds.Ref;
        *iq_out = DW.pi_qs.Ref * FOC_Q_PASSIVE_DECAY;
        s_util_lpf = 0.0F;
        s_fw_prev = 0.0F;
        DW.pi_voltage.Up = 0.0F;  DW.pi_voltage.Ui = 0.0F;
        DW.pi_voltage.Vl = 0.0F;  DW.pi_voltage.PI_out = 0.0F;
        return;
    }

    /* 电压利用率只取实际输出电压, 低通后驱动弱磁角。 */
    util = sqrtf((DW.motor_anal.Vd_buf * DW.motor_anal.Vd_buf) +
                 (DW.motor_anal.Vq_buf * DW.motor_anal.Vq_buf)) / DW.motor_anal.bus_Volt;
    if (util > 3.0F) { util = 3.0F; }
    s_util_lpf += 0.15F * (util - s_util_lpf);
    util = s_util_lpf;

    /* 电压宽裕且无转矩需求时退出弱磁; 电压紧张时保留弱磁以维持可控性。 */
    if ((!(fabsf(Is_dem) >= 1.0E-3F)) && (util < 0.45F)) {
        *id_out = 0.0F;
        *iq_out = DW.pi_qs.Ref * FOC_Q_PASSIVE_DECAY;
        s_fw_prev = 0.0F;
        DW.pi_voltage.Up = 0.0F;  DW.pi_voltage.Ui = 0.0F;
        DW.pi_voltage.Vl = 0.0F;  DW.pi_voltage.PI_out = 0.0F;
        return;
    }
    DW.pi_voltage.Ref = FW_UTIL_REF;
    DW.pi_voltage.Fbk = util;
    PI_Contrl_Modern(&DW.pi_voltage, 0U, 0.0F);
    fw = DW.pi_voltage.PI_out;
    if (fw > 0.0F)             { fw = 0.0F; }
    else if (fw < -1.5707963F) { fw = -1.5707963F; }

    /* 电压裕量充足时禁止弱磁角继续加深, 积分状态同步到钳后角度。 */
    if ((util < (FW_UTIL_REF - 0.02F)) && (fw < s_fw_prev)) {
        fw = s_fw_prev;
        DW.pi_voltage.Ui     = fw - DW.pi_voltage.Up;
        DW.pi_voltage.Vl     = fw;
        DW.pi_voltage.PI_out = fw;
    }
    if ((fw <= -1.5F) && (util < 0.45F)) {
        DW.pi_voltage.Ui     = 0.0F;
        DW.pi_voltage.Up     = 0.0F;
        DW.pi_voltage.Vl     = 0.0F;
        DW.pi_voltage.PI_out = 0.0F;
        fw = 0.0F;
    }
    s_fw_prev = fw;

    /* 速度环饱和时不能用其需求放大 MTPA 去磁分量。 */
    iq_abs  = fabsf(Is_dem);
    iq_mtpa = fabsf(DW.pi_qs.Ref);
    if (iq_abs < iq_mtpa)  { iq_mtpa = iq_abs; }
    if (iq_mtpa < 1.0E-3F) { iq_mtpa = 1.0E-3F; }
    ldlq = lq_w - ld_w;
    if (ldlq > 10.0F)        { ldlq = 10.0F; }
    else if (ldlq < 1.0E-6F) { ldlq = 1.0E-6F; }
    cb = ((DW.motor_param.flux_value / ldlq) * 0.25F) * (1.0F / iq_mtpa);
    cb -= sqrtf((cb * cb) + 0.5F);
    if (cb > 1.0F)       { cb = 1.0F; }
    else if (cb < -1.0F) { cb = -1.0F; }
    foc_sincos_frac(acosf(cb) / 6.28318548F, &sinb, &cosb);
    iq_s = sinb * Is_dem;

    /* 弱磁深度只由电压闭环调节, 不再随转矩需求或转速额外缩放。 */
    foc_sincos_frac(fw / 6.28318548F, &sinfw, &cosfw);
    id_max = -foc_flux_ld_limit();
    {
        real32_T id_circ = 0.95F * FOC_CurrentCircle();
        if (id_max < -id_circ) { id_max = -id_circ; }
    }
    idfw = sinfw * (-id_max);
    if (idfw > 0.0F) { idfw = 0.0F; }
    id_cmd = idfw + (iq_mtpa * cosb);
    if (id_cmd < id_max) { id_cmd = id_max; }

    /* 电流圆保 d 裁 q; 电压可达性在最终参考端统一限制。 */
    id2   = id_cmd * id_cmd;
    imax2 = FOC_CurrentCircle() * FOC_CurrentCircle();
    iqlim = (id2 > imax2) ? 0.0F : sqrtf(imax2 - id2);
    if (iq_s > iqlim)       { *iq_out = iqlim; }
    else if (iq_s < -iqlim) { *iq_out = -iqlim; }
    else                    { *iq_out = iq_s; }
    *id_out = id_cmd;
}

/* 调制系数随额定电压(sys_param.rated_volt)派生; 额定电压变更(CAN/EEPROM 装载)后调用,
   未配置(0/过小)时回编译默认值。比例由FOC_MOD_SCALE_MODE统一选择 */
void FOC_UpdateSvpwmFactor(void)
{
    real32_T v = (real32_T)DW.sys_param.rated_volt;
    if (v < 10.0F) { v = (real32_T)RATED_VOLTAGE_DEFAULT; }
    DW.motor_anal.svpwm_factor = foc_mod_vscale(v);
}

/* ******************** FOC 初始化 ******************** */
/* 移植自 MCU_MotorContrl_2024b_initialize() 的 FOC 部分 (PI 增益在 PI_Pare_init) */
void FOC_Ctrl_Init(void)
{
    /* 变换中间量结构体清零 */
    memset(&clarke_u, 0, sizeof(clarke_u));
    memset(&park_u,   0, sizeof(park_u));
    memset(&ipark_u,  0, sizeof(ipark_u));
    memset(&svpwm_u,  0, sizeof(svpwm_u));
    s_grad_buf = 0.0F;

    
    DW.motor_anal.PWM_HalfPerMax = (uint16_T)(PWM_TIMER_PERIOD >> 1U);
    DW.motor_anal.PWM_freq       = 1U;
    DW.motor_anal.time_base      = 1.0F;
		
		DW.sys_param.rated_volt=72;
		DW.sys_param.bus_curr_max=200;
		DW.sys_param.out_curr_max=600;
		DW.vehicle_info.BMS_charge_curr[0]=80;	/* BMS 缺省: 允许充电电流(A) */
		DW.vehicle_info.BMS_charge_curr[1]=80;
		DW.vehicle_info.BMS_charge_curr_valid=0U;	/* 0=整车未接入, 上述缺省生效; CAN写入后置位 */
		DW.vehicle_info.BMS_charge_volt_max=80.0F;	/* BMS 缺省: 最大充电电压(V) */
		DW.sys_param.temp_sensor=0;
		DW.sys_param.sensor_type=1;
	
    DW.faule_protection.over_Volt_thresh       = 86.0F;
    DW.faule_protection.under_Volt_thresh      = 60.0F;
    /* 自恢复点: 默认保持原有 2V 回滞(86->84 / 27->29), 可由 CAN 改写 */
    DW.faule_protection.over_Volt_recover      = 84U;
    DW.faule_protection.under_Volt_recover     = 29U;
    DW.faule_protection.Curr_thresh            = 550.0F;
    DW.faule_protection.over_Mos_temp_thresh   = 75.0F;
		DW.faule_protection.over_Mos_temp_high_thresh = 70.0F;
		DW.faule_protection.over_Mos_temp_recover_thresh = 65.0F;
		DW.faule_protection.over_Mos_temp_thresh_Per=200;
    DW.faule_protection.over_motor_temp_thresh = 130.0F;
		DW.faule_protection.over_motor_temp_high_thresh = 125.0F;
		DW.faule_protection.over_motor_temp_recover_thresh = 120.0F;
		DW.faule_protection.over_motor_temp_thresh_Per = 200;
    /* 硬件自检阈值: 原先无代码默认值(上电为 0), 会让全新板的自检必然误报
       相电流零漂/温度采样电路异常, 此处补齐。单位: 电流为 A, 温度电路阈值为
       ADC 电压 V (采样电阻值变量在当前编译模式下存的就是电压) */
    DW.faule_protection.hardware_phase_curr          = 10.0F;
    DW.faule_protection.hardware_temp_upper_limit[0] = 3.2F;
    DW.faule_protection.hardware_temp_lower_limit[0] = 0.1F;
    DW.faule_protection.hardware_temp_thresh_Per[0]  = 100U;
    DW.faule_protection.hardware_temp_upper_limit[1] = 3.2F;
    DW.faule_protection.hardware_temp_lower_limit[1] = 0.1F;
    DW.faule_protection.hardware_temp_thresh_Per[1]  = 100U;
    /* 堵转电流阈值: 单位与 LR_limit_Curr 同为 A, 必须低于限流值(100)才可能触发 ——
       相电流峰值约等于限流值, 阈值高于限流则该判据永不成立(工程默认值由外部下发,
       改此处只对空白板生效; 已有板子须用 CAN 写一次, 参数存 EEPROM 不清不重载) */
    DW.faule_protection.LRC_thresh             = 300.0F;
    /* 确认时间已统一 ms 口径(1kHz 慢分支 1ms/拍): 默认值折回旧实现的物理时间,
       否则默认行为会从 5ms/50ms 变成 50ms/500ms(这两个参数既不持久化也不走 CAN,
       改默认值即改行为) —— 母线跌落保护应快于 50ms 量级 */
    DW.faule_protection.over_Volt_Per          = 5U;    /* 5ms */
    DW.faule_protection.under_Volt_Per         = 50U;   /* 50ms */
    DW.faule_protection.over_speed_thresh      = 7200U;
    DW.faule_protection.speed_MAX_Per          = 1000U;
    DW.faule_protection.hearbeat_timeout       = 500U;
    DW.faule_protection.LR_speed_thresh        = 100U;
    DW.faule_protection.LR_Per                 = 5000U;
    DW.faule_protection.LR_state               = 1U;   /* Q3: 启用堵转(时长 5000ms 见 LR_Per) */
    DW.faule_protection.LR_mode                = 0U;
    DW.faule_protection.LR_limit_Curr          = 100.0F;
		DW.faule_protection.handlebar_voltage[0]	 = 4.4f;
		DW.faule_protection.handlebar_voltage[1]	 = 0.8f;
    /* 主反馈丢失: 速度记忆阈值 + 编码器死寂确认 + 大电流门 */
    DW.faule_protection.fbk_loss_spd_th    = 100.0F;   /* 速度记忆阈值 RPM */
    DW.faule_protection.fbk_loss_fast_Per  = 3U;       /* 编码器死寂确认 ms */
    DW.faule_protection.fbk_loss_curr_th   = 50.0F;    /* 慢速档大电流阈值 A */
    DW.faule_protection.fbk_lose_curr_hi   = 40.0F;    /* 电流-转速矛盾: 大电流门 A */
    /* 缺相 (包络不平衡 + 断多相), 缺几相共用同一故障码 */
    /* 幅值门: 直流阻断器(旋转时跟踪扣除通道零偏)已接管抗零偏职责, 门限仅需
       盖过阻断残余(收敛后 <0.03A)与包络噪声, 取 0.10A —— 可判相电流幅值约 >=0.16A */
    DW.faule_protection.phase_lose_act_curr  = 0.10F;  /* 幅值门 A(直流阻断后, 仅抗残余噪声) */
    DW.faule_protection.phase_lose_env_nf    = 0.10F;  /* 包络噪声底扣除 A(实测采样噪声σ≈0.1A) */
    /* 不平衡比: 健康基线实测 1.20。U/W 是物理传感器, 断后悬空拾取约 1.0~1.2A 交流
       残差, 使该相包络不归零, 比值天生只有约 2:1(V 是软件合成相, 断时结构性为 0,
       可达 4.65)。故门限取 2.0。【余量薄, 待 1ms 数据复核】 */
    DW.faule_protection.phase_lose_imbal_k   = 3.0F;
    DW.faule_protection.phase_lose_min_spd   = 100.0F; /* 速度门 RPM: 低速包络随转子位置脉动, 需抑制(台架 100rpm 单相断线实测可判, 200 会漏判) */
    DW.faule_protection.phase_lose_set_Per   = 100U;   /* 断单相确认 ms(第21轮: 200->100 提速) */
    DW.faule_protection.phase_lose_err_th    = 2.0F;
    /* 电压证据余量: 不要求顶到调制上限(那既慢又常到不了)。实测断两相时相电流很小而 Vq 已很大, 只要 Vq 比该转速反电势高 8V 即判"推不动"。 */
    DW.faule_protection.phase_lose_vq_margin = 8.0F;
    DW.faule_protection.phase_lose_multi_Per = 300U;   /* 断多相确认 ms(第21轮: 500->300 提速): 负载阶跃类瞬态只有 10~100ms, 300ms 仍能滤掉 */
    DW.faule_protection.phase_lose_nocurr   = 10.0F;   /* 残余零偏容差: 2×实测±0.5A */
    DW.faule_protection.phase_lose_push_imax = 10.0F;   /* 与 nocurr 同口径(限流10%) */
    DW.faule_protection.phase_lose_env_hi   = 10.0F;   /* Q1: 限流 10% */
    DW.faule_protection.fbk_lose_pwm_Per    = 15U;  /* 窗口内无新帧判定时间 ms(帧率~971Hz, 15ms=约15帧缺席); 0=关 */
		
    DW.motor_main_feedback.poles = 4U;

    DW.motor_param.Vdc_square    = 1727.99988F;
    DW.motor_param.motor_type    = 0U;
    DW.motor_param.sensor_type   = 0U;
    DW.motor_param.temp_sensor   = 3U;
    DW.motor_param.PP_num        = 4U;
    DW.motor_param.enc_line_count    = 1024U;   /* 主反馈分辨率: UIME 1024 线(10bit) */
		DW.motor_param.sensor_pole_pairs = 4U;      /* 保留字段, 不参与编码器计算 */
		DW.motor_param.encoder_reverse = ENCODER_REVERSE_ENABLE; /* 默认ABZ方向, CAN/EEPROM可覆盖 */
		DW.comm_contrl.zero_ref_manual     = 0U;   /* 手动零位基准(未下发) */
    DW.motor_param.rated_volt    = 72U;
    DW.motor_param.peak_curr     = 300U;
    DW.motor_param.rated_torque  = 8U;
    DW.motor_param.rated_speed   = 6000U;
    DW.motor_param.peak_speed    = 9000U;
    DW.motor_param.Ld_value      = 0.0000412F;
    DW.motor_param.Lq_value      = 0.0000981F;
    DW.motor_param.flux_value    = 0.016569F;
    DW.motor_param.EMF_constant  = 8.45F;
    DW.motor_param.Rs_value      = 0.04F;
    /* CAN A0/04/2 弱磁电流默认值: ~=psi/Lq(59.2A)自然弱磁深度, EEPROM 有存值则覆盖 */
    DW.comm_contrl.I_max         = 300.0F;
		DW.motor_param.reduction_ratio=6.08F;
    DW.motor_param.hall_phase_shift = 0;   /* A7/03 idx0: 霍尔相位偏移(12bit 电角度) */

    DW.adc_slmple.bus_Curr           = 0U;
    DW.adc_slmple.phaseU_Curr        = 0U;
    DW.adc_slmple.phaseV_Curr        = 0U;
    DW.adc_slmple.phaseW_Curr        = 0U;
    DW.adc_slmple.bus_Volt           = 0U;
    DW.adc_slmple.MOS_Temp           = 0U;
    DW.adc_slmple.mototr_Temp        = 0U;
    DW.adc_slmple.handlebar_Volt     = 0U;
    DW.adc_slmple.bus_Curr_offset    = 0U;
    DW.adc_slmple.phaseU_Curr_offset = 0U;
    DW.adc_slmple.phaseV_Curr_offset = 0U;
    DW.adc_slmple.phaseW_Curr_offset = 0U;
    DW.adc_slmple.zero_flag          = 0U;

    DW.slope_calc_spd.slope_grad = 0.1F;
    DW.slope_calc_spd.grad_timer = 9U;   /* 1=200us 节拍(速度环动态不变); 斜率=设定值RPM/s */

    DW.comm_contrl.sys_run_mode    = 0U;
    DW.comm_contrl.mos_temp_sensor = 0U;
    DW.comm_contrl.speed_Ke_Acc    = 1000.0F;
    DW.comm_contrl.speed_Ke_Dece   = 1000.0F;

		DW.comm_contrl.zero_identify_curr=2;
		DW.comm_contrl.zero_identify_ctrl=0;
		DW.comm_contrl.identify_speed=1000;
		DW.comm_contrl.identify_curr=3;
		DW.comm_contrl.identify_ctrl=0;
    FOC_Ctrl_Clear();
		FOC_UpdateSvpwmFactor();       /* 调制系数随额定电压派生, 见下 */
}

/* ******************** FOC 清零 ******************** */
void FOC_Ctrl_Clear(void)
{
    DW.pi_position.Up = 0.0F;  DW.pi_position.Ui = 0.0F;
    DW.pi_position.Vl = 0.0F;  DW.pi_position.PI_out = 0.0F;
    DW.pi_ds.Up = 0.0F;  DW.pi_ds.Ui = 0.0F;
    DW.pi_ds.Vl = 0.0F;  DW.pi_ds.PI_out = 0.0F;
    DW.pi_qs.Up = 0.0F;  DW.pi_qs.Ui = 0.0F;
    DW.pi_qs.Vl = 0.0F;  DW.pi_qs.PI_out = 0.0F;
    DW.pi_speed.Up = 0.0F;  DW.pi_speed.Vl = 0.0F;
    DW.pi_speed.Ui = 0.0F;  DW.pi_speed.PI_out = 0.0F;
    DW.pi_voltage.Up = 0.0F;  DW.pi_voltage.Ui = 0.0F;
    DW.pi_voltage.Vl = 0.0F;  DW.pi_voltage.PI_out = 0.0F;
    /* 指令、弱磁、调速器、限幅和反馈状态统一复位 */
    s_hold_id_cmd = 0.0F;
    s_hold_iq_cmd = 0.0F;
    s_is_cap = -1.0F;
    s_util_lpf = 0.0F;
    s_fw_prev = 0.0F;
    s_pwr_q_scale = 1.0F;
    s_volt_q_scale = 1.0F;
    s_valpha_applied = 0.0F;
    s_vbeta_applied = 0.0F;
    s_we_fb_f = 0.0F;
    DW.motor_anal.Vd_buf = 0.0F;
    DW.motor_anal.Vq_buf = 0.0F;
    DW.pi_ds.Ref = 0.0F;
    DW.pi_qs.Ref = 0.0F;
    s_iscale = 1.0F;
    s_cmode_prev = 0xFFFFU;
    s_sub_prev = 0xFFFFU;
    s_grad_buf = 0.0F;
    s_feas_clamp_act = 0U;
    s_volt_clamp_act = 0U;
    s_circ_clamp_act = 0U;
    s_phase_clamp_act = 0U;
    s_pwr_d_excess = 0U;
    s_pwr_fb_bad = 0U;
    s_volt_exhausted = 0U;
    s_bus_guard_act = 0U;
#if (CURR_FB_FILT_EN != 0)
    s_id_f = 0.0F;
    s_iq_f = 0.0F;
#endif
#if (PWR_GOV_EN != 0)
    s_gov_bind = 0U;
    s_gov_hold = 0U;
    s_gov_ref = 0.0F;
    s_gov_red = 0.0F;
    s_gov_on_t = 0.0F;
    s_gov_off_t = 0.0F;
    s_gov_init = 0U;
#endif
    BusCurrFast_Reset();
}

/* ******************** 停止 FOC ******************** */
/* 关闭 PWM 输出, 占空比清零, 清 PI, 速度环斜坡计数清零 */
void motor_stop(void)
{
    /* 关闭 PWM 输出 (主通道 + 互补通道) */
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

    /* 占空比清零 */
    htim1.Instance->CCR1 = 0U;
    htim1.Instance->CCR2 = 0U;
    htim1.Instance->CCR3 = 0U;
    DW.ctrl_hw_io.SVPTa = 0U;
    DW.ctrl_hw_io.SVPTb = 0U;
    DW.ctrl_hw_io.SVPTc = 0U;

    /* 清 PI */
    FOC_Ctrl_Clear();

    /* 速度环斜坡计数器清零 */
    DW.slope_calc_spd.timer_count = 0U;
    DW.slope_calc_spd.Y_out       = 0.0F;
    s_grad_buf                    = 0.0F;
		DW.motor_anal.Vq_buf					=0;
		DW.motor_anal.Vd_buf					=0;
		DW.up_data_param.id_ref_current = 0.0F;
		DW.up_data_param.iq_ref_current = 0.0F;
}
real32_T id_ref = 0.0F, iq_ref = 0.0F;
/* ******************** 启动电机 ******************** */
/* 三相占空比预置 40%, 再开启 PWM 输出 */
void motor_start(void)
{
    uint16_T duty40 = (uint16_T)(((uint32_T)(htim1.Instance->ARR + 1U) * 2U) / 5U);  /* 40% */

    htim1.Instance->CCR1 = duty40;
    htim1.Instance->CCR2 = duty40;
    htim1.Instance->CCR3 = duty40;
    DW.ctrl_hw_io.SVPTa = duty40;
    DW.ctrl_hw_io.SVPTb = duty40;
    DW.ctrl_hw_io.SVPTc = duty40;

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

/* ******************** FOC 核心控制任务 ******************** */
/* 在 10kHz PWM/ADC 中断中调用 */
void FOC_Ctrl_Task(void)
{
     real32_T volatile v_q, v_d;        /* Vq/Vd 命令 */
     real32_T volatile sin_th, cos_th;  /* 角度 sin/cos */
     Ang_SinCos sc;
    uint16_T cmode = DW.comm_contrl.control_mode;
    uint16_T submode = DW.comm_contrl.contorl_mode_sub;

    if ((s_cmode_prev != cmode) || (s_sub_prev != submode)) {
        FOC_Ctrl_Clear();
        DW.slope_calc_spd.Y_out = (real32_T)DW.motor_main_feedback.mech_Spd;
        DW.slope_calc_spd.timer_count = 0U;
        s_cmode_prev = cmode;
        s_sub_prev = submode;
    }

    if ((cmode != (uint16_T)Control_Mode_VF) &&
        (cmode != (uint16_T)Control_Mode_HighFre) &&
        (foc_motor_params_valid() == 0U)) {
        DW.comm_contrl.control_enable = 0U;
        motor_stop();
        return;
    }

    if (cmode == (uint16_T)Control_Mode_VF) {
        /* VF 开环 */
        v_q = DW.comm_contrl.VF_Vq_Vol;
        v_d = DW.comm_contrl.VF_Vd_Vol;
        DW.Angle_buf += DW.comm_contrl.VF_freq;
        sc.table_Angle = (uint16_T)((int32_T)DW.Angle_buf & 0x0FFF);
			  SinCos_Table(&sc);
        sin_th = sc.table_Sin;
        cos_th = sc.table_Cos;

    } else if (cmode == (uint16_T)Control_Mode_HighFre) {
        /* 高频注入 */
        v_q = 2.0F;
        v_d = 0.0F;
        sc.table_Angle = DW.motor_main_feedback.elec_angle & 0x0FFFU;
    } else {
        /* ***** 闭环 ***** */
//        real32_T id_ref = 0.0F, iq_ref = 0.0F;
#if (FOC_FEED_FORWARD_EN == 0)
        real32_T vq_lim;
        uint8_T  lstate;
#endif

        if (cmode == (uint16_T)Control_Mode_Idq) {
            id_ref = DW.comm_contrl.I_d_target;
            {
                real32_T id_cap = FOC_CurrentCircle();
                if (id_ref > id_cap) { id_ref = id_cap; }
                else if (id_ref < -id_cap) { id_ref = -id_cap; }
            }
            iq_ref = DW.comm_contrl.I_q_target;
            /* 需求级输出限幅(保 d 裁 q): Idq 直通与速度/转矩模式同一条限流链 */
            iq_ref = Pwr_Limit_Curr(iq_ref);
        }
				else if (cmode == (uint16_T)Control_Mode_Idq_Tset) {
            id_ref = DW.comm_contrl.I_d_target;
            {
                real32_T id_cap = FOC_CurrentCircle();
                if (id_ref > id_cap) { id_ref = id_cap; }
                else if (id_ref < -id_cap) { id_ref = -id_cap; }
            }
            iq_ref = DW.comm_contrl.I_q_target;
            iq_ref = Pwr_Limit_Curr(iq_ref);
						DW.motor_main_feedback.elec_angle=0;
        }				
				else if ((cmode == (uint16_T)Control_Mode_Speed) ||
                   (cmode == (uint16_T)Control_Mode_FD)) {
            real32_T spd_out;
            uint8_T  spd_pi_ran;   /* 本拍速度 PI 是否运行(反算抗饱和按同节拍注入) */
            /* --- 速度斜坡 + 速度环 (按 grad_timer 节拍更新) --- */
            if (DW.slope_calc_spd.timer_count >= DW.slope_calc_spd.grad_timer) {
                real32_T tgt, ka, kd, step;
                GXieLv gx;
                /* 节拍周期(s)=(grad_timer+1)拍x100us; 斜率随节拍折算 -> 设定值=真 RPM/s */
                step = (real32_T)(DW.slope_calc_spd.grad_timer + 1U) * 1.0e-4F;
                if (cmode == (uint16_T)Control_Mode_Speed) {
#if (PWR_GOV_EN != 0)
                    tgt  = PwrGov_Ref(DW.comm_contrl.speed_target, step);   /* 功率调速器: 连续恒功率滑落 */
#else
                    tgt  = DW.comm_contrl.speed_target;
#endif
                    ka   = DW.comm_contrl.speed_Ke_Acc*step;
                    kd   = DW.comm_contrl.speed_Ke_Dece*step;
                } else {
                    DW.pi_position.Ref = DW.comm_contrl.position_target;
                    DW.pi_position.Fbk = DW.motor_main_feedback.mech_angle;
                    PI_Contrl(&DW.pi_position);
                    tgt  = DW.pi_position.PI_out;
                    ka   = DW.comm_contrl.speed_Ke_Acc*step;
                    kd   = DW.comm_contrl.speed_Ke_Dece*step;
                }
                /* 斜率按幅值增减选: 目标与斜坡输出异号时先按减速档收到零(整机反向), 同号时幅值
                   在增走加速档、在减走减速档。跟斜坡输出比而不是跟目标历史比: 指令不变而斜坡
                   仍在落时也要走减速档 */
#if (SPD_DEC_SOFT_EN != 0)
                /* 低速减速值收敛(过零保护, 干预减速度而非输出): 减速档梯度随当前转速
                   线性收缩(300rpm 起收敛, 保底 5% 防停死) —— 速度环跟踪平缓的斜坡,
                   制动需求自然有界, 惯性平滑落到目标不穿零; 刻意反向时过零段同样收敛(温柔换向) */
                {
                    /* 收敛参考取"当前转速"与"斜坡离目标的剩余距离"的较小者:
                       只按转速收敛时, 减到非零目标(如 3000->580)在 580rpm 以上全程不收缩,
                       斜坡全速撞进目标 -> 速度环以 2400rpm/s 量级的制动权限过冲穿零
                       (实测冲过 580 一路到 -265 再拉回); 距离项让任意目标都能软着陆,
                       转速项继续覆盖刻意反向的过零段 */
                    real32_T spd_w  = fabsf((real32_T)DW.motor_main_feedback.mech_Spd);
                    real32_T dist_w = fabsf(tgt - DW.slope_calc_spd.Y_out);
                    real32_T ref_w  = (spd_w < dist_w) ? spd_w : dist_w;
                    real32_T kdec = ref_w / 300.0F;
                    if (kdec > 1.0F) { kdec = 1.0F; }
                    if (kdec < 0.05F) { kdec = 0.05F; }
                    kd *= kdec;
                }
#endif
                if ((tgt * DW.slope_calc_spd.Y_out) < 0.0F)              { s_grad_buf = kd; }
                else if (fabsf(tgt) >= fabsf(DW.slope_calc_spd.Y_out))   { s_grad_buf = ka; }
                else                                                     { s_grad_buf = kd; }
                gx.XieLv_X    = tgt;
                gx.XieLv_Y    = DW.slope_calc_spd.Y_out;
                gx.XieLv_Grad = s_grad_buf;
                Grad_XieLv(&gx);
                DW.slope_calc_spd.Y_out = gx.XieLv_Y;
                DW.pi_speed.Ref = gx.XieLv_Y;
                DW.pi_speed.Fbk = (real32_T)DW.motor_main_feedback.mech_Spd;
                {
                    /* 可达性条件积分: 下游钳位(限制/电压椭圆/相电流)激活期间, 只挡加深
                       方向、放行回卷方向(与 PI_Contrl_Modern 同哲学) —— 需求贴可达边界
                       滑行时积分器不上绕, 卸载/松绑瞬间不甩突变目标(数据17: 旧顶死保持
                       机制解除瞬间甩出上绕需求, 硬件过流 BKIN) */
                    real32_T ui_prev = DW.pi_speed.Ui;
                    PI_Contrl(&DW.pi_speed);
                    if (((s_feas_clamp_act != 0U) || (s_bus_guard_act != 0U) ||
                         (s_pwr_d_excess != 0U)) &&
                        (((DW.pi_speed.Ui - ui_prev) * DW.pi_speed.PI_out) > 0.0F)) {
                        DW.pi_speed.Ui = ui_prev;
                    }
                }
                spd_out = DW.pi_speed.PI_out;
                DW.slope_calc_spd.timer_count = 0U;
                spd_pi_ran = 1U;
            } else {
                DW.slope_calc_spd.timer_count =
                    (uint16_T)(DW.slope_calc_spd.timer_count + 1U);   /* 累加分频: 200us 节拍确定化(原置满写法凑巧也是 2 拍一跳, 行为不变) */
                spd_out = DW.pi_speed.PI_out;
                spd_pi_ran = 0U;
            }

            /* 输出限幅(速度环输出级), 每拍执行; 钳位时对速度环积分器反算抗饱和 */
            {
                real32_T iq_lim = Pwr_Limit_Curr(spd_out);
                if ((iq_lim != spd_out) && (spd_pi_ran != 0U)
#if (PWR_GOV_EN != 0)
                    && (s_gov_hold == 0U)   /* 调速器接管后停反算: 抽积分是与钳位对拉的环路 */
#endif
                                                                   ) {
                    /* 反算与速度 PI 同节拍注入, 保持 Kaw 原语义 */
                    DW.pi_speed.Ui += DW.pi_speed.Kaw * (iq_lim - spd_out);
                }
                spd_out = iq_lim;
            }

            /* 弱磁角域每拍钉 [0, -pi/2]: 不钉时低速积分器驻留正向, 高速弱磁介入延迟秒级 */
            DW.pi_voltage.upper_limit = 0.0F;
            DW.pi_voltage.lower_limit = -1.5707963F;
            if (DW.comm_contrl.contorl_mode_sub == 1U) {
                foc_mtpa_flux(spd_out, &id_ref, &iq_ref);   /* MTPA + 弱磁; 母线预算只在 MTPA 后单层执行 */
            } else if (DW.comm_contrl.contorl_mode_sub == 2U) {
                /* 查表法 (MTPV MAP): 表的请求轴是扭矩(N·m), 速度环输出是电流域
                   (A), 入口按 Kt 折算后再查表 */
                real32_T kt = 1.5F * (real32_T)DW.motor_param.PP_num * DW.motor_param.flux_value;
                if (kt < 0.001F) { kt = 0.001F; }
                foc_map_lookup(spd_out * kt,
                               (real32_T)DW.motor_main_feedback.mech_Spd,
                               &id_ref, &iq_ref);
            } else {
                id_ref = 0.0F;  iq_ref = spd_out;           /* Id=0 */
            }
        } else if (cmode == (uint16_T)Control_Mode_Torq) {
            /* 转矩模式: VCU 转矩需求 Te_target(N·m, A0/0D) -> 固定转矩常数折算 iq 需求,
               下游与速度模式同链(输出限幅 -> MTPA+弱磁/查表 -> 电流环), 速度环不受影响。
               Kt = 1.5*p*psi 由电机参数派生, 上位机改参数即自动更新;
               MTPA 区间实际转矩比指令富余(单调同向, 油门手感由 VCU 曲线吸收)。
               平滑与限速由 VCU 负责, 超速由故障层兜底; 转矩斜坡(slope_calc_torque)
               预留后续按需接入 */
            real32_T kt = 1.5F * (real32_T)DW.motor_param.PP_num * DW.motor_param.flux_value;
            real32_T iq_dem;
            if (kt < 0.001F) { kt = 0.001F; }
            iq_dem = DW.comm_contrl.Te_target / kt;
            iq_dem = Pwr_Limit_Curr(iq_dem);
            /* 弱磁角域钉位与速度模式一致(foc_mtpa_flux 依赖) */
            DW.pi_voltage.upper_limit = 0.0F;
            DW.pi_voltage.lower_limit = -1.5707963F;
            if (DW.comm_contrl.contorl_mode_sub == 1U) {
                foc_mtpa_flux(iq_dem, &id_ref, &iq_ref);     /* MTPA + 弱磁; 母线预算只在 MTPA 后单层执行 */
            } else if (DW.comm_contrl.contorl_mode_sub == 2U) {
                /* 查表(MTPV MAP): 输入即转矩本体(N·m), 不经 Kt 折算 */
                foc_map_lookup(DW.comm_contrl.Te_target,
                               (real32_T)DW.motor_main_feedback.mech_Spd,
                               &id_ref, &iq_ref);
            } else {
                id_ref = 0.0F;  iq_ref = iq_dem;             /* Id=0 */
            }
        }
				else {
            id_ref = s_hold_id_cmd;
            iq_ref = s_hold_iq_cmd;
        }
				//限制器清除
        s_feas_clamp_act = 0U;
        s_volt_clamp_act = 0U;
        s_circ_clamp_act = 0U;
        s_phase_clamp_act = 0U;
        s_pwr_d_excess = 0U;
#if (BUS_PWR_GUARD_EN != 0)
        /* 母线功率只在最终参考端由快速闭环限制 */
        foc_pwr_limit_single(&id_ref, &iq_ref,
                             (real32_T)DW.motor_main_feedback.mech_Spd,
                             DW.motor_anal.bus_Volt);
#endif
#if (VOLT_ELLIPSE_CLAMP_EN != 0)
        /* 电压可达性最终钳位: 限制裁功率, 椭圆裁物理可达(所有参考源/象限) */
        foc_volt_ellipse_clamp(&id_ref, &iq_ref);
#endif
        /* 实测相电流最终限制: 母线限制之外, 所有模式/速度/象限都执行 */
        foc_phase_current_limit(&id_ref, &iq_ref);
        /* ----- 电角度 sin/cos -----
           Park 用角度回退到 ADC 采样时刻: 电流在 CC4 触发(计数中点)采样,
           elec_angle 时间戳在 UEV 中断, 差 we*T(带符号, 正反转统一)。
           只影响 Park 反馈; iPark 输出角超前(FOC_FF_DELAY_TICKS)另算, 两者正交;
           不写回 elec_angle(故障基波解调/霍尔插值/PWM 位置核对仍用原值) */
        {
            uint16_T ang_park = DW.motor_main_feedback.elec_angle & 0x0FFFU;
#if (FOC_FB_DELAY_EN != 0)
            /* we 先低通(τ≈2ms)再补角度: 速度测量噪声/毛刺不进 Park 角(补偿杠杆仅
               50µs, 1000rpm 毛刺折合 1.2°e, 滤波后更小; 加减速 3000rpm/s 时
               2ms 滞后折合 <0.1 计数, 动态无损) */
            real32_T we_fb = (real32_T)DW.motor_main_feedback.mech_Spd
                             * ((real32_T)DW.motor_param.PP_num * 0.104719755F);
            s_we_fb_f += 0.05F * (we_fb - s_we_fb_f);
            we_fb = s_we_fb_f;
            int32_T  dth_fb = (int32_T)(we_fb * (FOC_FB_DELAY_TICKS * 1.0e-4F) * 651.8986F);
            ang_park = (uint16_T)(((uint32_T)((int32_T)ang_park - dth_fb)) & 0x0FFFU);
#endif
            sc.table_Angle = ang_park;
            SinCos_Table(&sc);
            sin_th = sc.table_Sin;
            cos_th = sc.table_Cos;
        }

        /* ----- Clarke (U/W): Alpha=Iu, Beta=-(Iu+2*Iw)/sqrt3 -----
           V 相在 signal_proc.c 中本就由 -(U+W) 计算得到, 此处直接用 U/W,
           与经 V 中转的 (Us+2*Vs)/sqrt3 代数等价(省一次加减, 少一层浮点传递) */
        clarke_u.Us    = DW.motor_anal.phaseU_Curr;
        clarke_u.Ws    = DW.motor_anal.phaseW_Curr;
        clarke_u.Alpha = clarke_u.Us;
        clarke_u.Beta  = -(clarke_u.Us + clarke_u.Ws * 2.0f) * 0.577350259f;

        /* ----- Park: 等价 PARK_Cale ----- */
        park_u.Alpha  = clarke_u.Alpha;
        park_u.Beta   = clarke_u.Beta;
        park_u.Sine   = sin_th;
        park_u.Cosine = cos_th;
        park_u.Ds = park_u.Alpha * park_u.Cosine + park_u.Beta * park_u.Sine;  /* Id 反馈 */
        park_u.Qs = park_u.Beta  * park_u.Cosine - park_u.Alpha * park_u.Sine; /* Iq 反馈 */

        /* 上一拍实际调制电压与本拍电流使用同一个反馈坐标系 */
        DW.motor_anal.Vd_buf = (s_valpha_applied * cos_th) + (s_vbeta_applied * sin_th);
        DW.motor_anal.Vq_buf = (s_vbeta_applied * cos_th) - (s_valpha_applied * sin_th);

#if (FOC_FEED_FORWARD_EN != 0)        /* ---- 前馈 V2: 指令值解耦 + 电压圆余量分配 + 输出角超前 ---- */
            real32_T we = (real32_T)DW.motor_main_feedback.mech_Spd
                          * ((real32_T)DW.motor_param.PP_num * 0.104719755F);
            /* 前馈与占圆限幅仅在 sub==1 生效, 其余模式 PI 用满圆(否则大 iq_ref 的
               vd_ff 会把 q 轴 PI 限幅压到反电势以下, 电流建立不起来) */
            const uint8_T ff_out_en = (DW.comm_contrl.contorl_mode_sub == 1U) ? 1U : 0U;
            real32_T vd_ff = 0.0F;
            real32_T vq_ff = 0.0F;
            real32_T vdc  = sqrtf(DW.motor_param.Vdc_square);
            real32_T lim, rem;

            if (ff_out_en != 0U) {
                real32_T ld_ff = Lookup_Ld(id_ref, iq_ref) * 1.0e-6F;   /* uH -> H, 按指令工作点 */
                real32_T lq_ff = Lookup_Lq(id_ref, iq_ref) * 1.0e-6F;
                vd_ff = -we * lq_ff * iq_ref;
                vq_ff =  we * (ld_ff * id_ref + DW.motor_param.flux_value);

                /* (2a) 可行性裁剪: vq_ff(反电势补偿)不削, 只削 vd_ff 进满圆;
                        vq_ff 本身超圆则钉到圆边(恒有 lim_q >= |vq_ff|) */
                real32_T vd_room2 = DW.motor_param.Vdc_square - vq_ff * vq_ff;
                if (vd_room2 < 0.0F) {
                    vq_ff = (vq_ff >= 0.0F) ? vdc : -vdc;
                    vd_ff = 0.0F;
                } else {
                    real32_T vd_cap = sqrtf(vd_room2);
                    if (fabsf(vd_ff) > vd_cap) {
                        vd_ff = (vd_ff >= 0.0F) ? vd_cap : -vd_cap;
                    }
                }
            }
            /* (2b) 前馈占圆后剩余给 PI: 每轴限幅 = sqrt(圆^2 - 对侧前馈^2);
               sub 0/2 前馈为 0, 即满圆限幅 */
            rem = DW.motor_param.Vdc_square - vq_ff * vq_ff;
            lim = (rem > 0.0F) ? sqrtf(rem) : 0.0F;
            DW.pi_ds.Ref = id_ref;
            DW.pi_ds.Fbk = park_u.Ds;
            DW.pi_qs.Fbk = park_u.Qs;
            foc_curr_fb_filter(&DW.pi_ds.Fbk, &DW.pi_qs.Fbk);
            PI_Contrl_Modern(&DW.pi_ds, 1U, lim);
            rem = DW.motor_param.Vdc_square - vd_ff * vd_ff;
            lim = (rem > 0.0F) ? sqrtf(rem) : 0.0F;
            DW.pi_qs.Ref = iq_ref;
            PI_Contrl_Modern(&DW.pi_qs, 1U, lim);
            if (ff_out_en != 0U)
            {
                v_d = vd_ff + DW.pi_ds.PI_out;
                v_q = vq_ff + DW.pi_qs.PI_out;
            }
            else
            {
                v_d = DW.pi_ds.PI_out;
                v_q = DW.pi_qs.PI_out;
            }
            /* (2c) 保底圆限幅及 PI 反算抗饱和 */
            real32_T vmag2 = v_d * v_d + v_q * v_q;
            if (vmag2 > DW.motor_param.Vdc_square) {
                real32_T vscale;
                s_volt_clamp_act = 1U;
                s_feas_clamp_act = 1U;
                vscale = sqrtf(DW.motor_param.Vdc_square / vmag2);
                v_d *= vscale;
                v_q *= vscale;
                DW.pi_ds.Ui += (v_d - vd_ff) - DW.pi_ds.PI_out;
                DW.pi_qs.Ui += (v_q - vq_ff) - DW.pi_qs.PI_out;
                DW.pi_ds.Ui = (DW.pi_ds.Ui > DW.pi_ds.upper_limit) ? DW.pi_ds.upper_limit :
                              ((DW.pi_ds.Ui < DW.pi_ds.lower_limit) ? DW.pi_ds.lower_limit : DW.pi_ds.Ui);
                DW.pi_qs.Ui = (DW.pi_qs.Ui > DW.pi_qs.upper_limit) ? DW.pi_qs.upper_limit :
                              ((DW.pi_qs.Ui < DW.pi_qs.lower_limit) ? DW.pi_qs.lower_limit : DW.pi_qs.Ui);
                DW.pi_ds.PI_out = v_d - vd_ff;
                DW.pi_qs.PI_out = v_q - vq_ff;
            }
            if ((cmode != (uint16_T)Control_Mode_Idq_Tset) &&
                (DW.comm_contrl.contorl_mode_sub == 1U)) {
                /* (3) 输出角超前: dtheta = we x 延迟拍数 x 100us, 折 12bit 计数(4096/2pi=651.9) */
                real32_T dth = we * (FOC_FF_DELAY_TICKS * 1.0e-4F) * 651.8986F;
                int32_T ang = (int32_T)DW.motor_main_feedback.elec_angle + (int32_T)dth;
                sc.table_Angle = (uint16_T)((uint32_T)ang & 0x0FFFU);
                SinCos_Table(&sc);
                sin_th = sc.table_Sin;   /* 仅 iPark(输出)用超前角; Park(反馈)已用采样角 */
                cos_th = sc.table_Cos;
            }
#else
        /* ----- d 轴电流环 -> Vd ----- */
        DW.pi_ds.Ref = id_ref;
        DW.pi_ds.Fbk = park_u.Ds;
        DW.pi_qs.Fbk = park_u.Qs;
        foc_curr_fb_filter(&DW.pi_ds.Fbk, &DW.pi_qs.Fbk);
        PI_Contrl_Modern(&DW.pi_ds, 0U, 0.0F);
        v_d = DW.pi_ds.PI_out;

        /* ----- Vdc 电压圆 -> Vq 动态限幅 ----- */
        vq_lim =DW.motor_param.Vdc_square - (v_d * v_d);
        if (vq_lim <= 0.0F) {
            if (DW.vd_err_cnt >= 5U) { DW.vd_err_cnt = 0U; lstate = 1U; }
            else { DW.vd_err_cnt++; lstate = 0U; }
            vq_lim = 0.0F;
        } else {
            lstate = 1U;
            vq_lim = sqrtf(vq_lim);//此处进行开方处理
        }

        /* ----- q 轴电流环 -> Vq ----- */
        DW.pi_qs.Ref = iq_ref;
        PI_Contrl_Modern(&DW.pi_qs, lstate, vq_lim);
        v_q = DW.pi_qs.PI_out;
#endif
	}
		/************参数上传读取********************/
		DW.up_data_param.target_speed     = DW.pi_speed.Ref;
    DW.up_data_param.id_current       = park_u.Ds;
    DW.up_data_param.iq_current       = park_u.Qs;
    DW.up_data_param.vd_voltage       = v_d;
    DW.up_data_param.vq_voltage       = v_q;
		DW.up_data_param.id_ref_current   = DW.pi_ds.Ref;
		DW.up_data_param.iq_ref_current   = DW.pi_qs.Ref;
   // FOC_LdLq_Estimate();
    /* ***** iPark: 等价 IPARK_Cale ***** */
    ipark_u.Ds     = v_d;
    ipark_u.Qs     = v_q;
    ipark_u.Sine   = sin_th;
    ipark_u.Cosine = cos_th;
    ipark_u.Alpha  = ipark_u.Ds * ipark_u.Cosine - ipark_u.Qs * ipark_u.Sine;
    ipark_u.Beta   = ipark_u.Qs * ipark_u.Cosine + ipark_u.Ds * ipark_u.Sine;

    /* ***** SVPWM: 等价 SVPWM_Cale (七段式扇区) ***** */
    svpwm_u.Ualpha = ipark_u.Alpha;
    svpwm_u.Ubeta  = ipark_u.Beta;
#if FOC_MODULATION_MODE == FOC_MOD_THI6
    svpwm_u.tmp1 = svpwm_u.Ubeta;
    svpwm_u.tmp2 = svpwm_u.Ubeta * 0.5f + svpwm_u.Ualpha * 0.8660254f;
    svpwm_u.tmp3 = svpwm_u.tmp2 - svpwm_u.tmp1;
    svpwm_u.VecSector = 3U;
    if (svpwm_u.tmp2 > 0.0f) { svpwm_u.VecSector--; }
    if (svpwm_u.tmp3 > 0.0f) { svpwm_u.VecSector--; }
    if (svpwm_u.tmp1 < 0.0f) { svpwm_u.VecSector = 7U - svpwm_u.VecSector; }

    if ((svpwm_u.VecSector == 1U) || (svpwm_u.VecSector == 4U)) {
        svpwm_u.Ta = svpwm_u.tmp2;
        svpwm_u.Tb = svpwm_u.tmp1 - svpwm_u.tmp3;
        svpwm_u.Tc = -svpwm_u.tmp2;
    } else if ((svpwm_u.VecSector == 2U) || (svpwm_u.VecSector == 5U)) {
        svpwm_u.Ta = svpwm_u.tmp3 + svpwm_u.tmp2;
        svpwm_u.Tb = svpwm_u.tmp1;
        svpwm_u.Tc = -svpwm_u.tmp1;
    } else if ((svpwm_u.VecSector == 3U) || (svpwm_u.VecSector == 6U)) {
        svpwm_u.Ta = svpwm_u.tmp3;
        svpwm_u.Tb = -svpwm_u.tmp3;
        svpwm_u.Tc = -(svpwm_u.tmp1 + svpwm_u.tmp2);
    } else {
        svpwm_u.Ta = 0.0f;
        svpwm_u.Tb = 0.0f;
        svpwm_u.Tc = 0.0f;
    }
#endif

    SVPWM_InjectZeroSeq(&svpwm_u);

    /* 限幅(过调制保护) + 占空比 (中心 + 标幺*半周期) */
        DW.motor_anal.svpwm_factor = foc_mod_vscale(DW.motor_anal.bus_Volt);
        real32_T km   = DW.motor_anal.svpwm_factor;
        real32_T half = (real32_T)DW.motor_anal.PWM_HalfPerMax;
        if (km < SVPWM_KM_FLOOR) { km = SVPWM_KM_FLOOR; }   /* 欠压/预充: 防 km→0 放大噪声 */

        if (svpwm_u.Ta > km) { svpwm_u.Ta = km; } else if (svpwm_u.Ta < -km) { svpwm_u.Ta = -km; }
        if (svpwm_u.Tb > km) { svpwm_u.Tb = km; } else if (svpwm_u.Tb < -km) { svpwm_u.Tb = -km; }
        if (svpwm_u.Tc > km) { svpwm_u.Tc = km; } else if (svpwm_u.Tc < -km) { svpwm_u.Tc = -km; }

        svpwm_u.SVPTa = (svpwm_u.Ta / km) * half + half;
        svpwm_u.SVPTb = (svpwm_u.Tb / km) * half + half;
        svpwm_u.SVPTc = (svpwm_u.Tc / km) * half + half;
				/*****************占空比限制*************************/
				if(svpwm_u.SVPTa<PWM_DUTY_MIN_LIMIT)
					svpwm_u.SVPTa=PWM_DUTY_MIN_LIMIT;
				else if(svpwm_u.SVPTa>PWM_DUTY_MAX_LIMIT)
					svpwm_u.SVPTa=PWM_DUTY_MAX_LIMIT;
				
				if(svpwm_u.SVPTb<PWM_DUTY_MIN_LIMIT)
					svpwm_u.SVPTb=PWM_DUTY_MIN_LIMIT;
				else if(svpwm_u.SVPTb>PWM_DUTY_MAX_LIMIT)
					svpwm_u.SVPTb=PWM_DUTY_MAX_LIMIT;
				
				if(svpwm_u.SVPTc<PWM_DUTY_MIN_LIMIT)
					svpwm_u.SVPTc=PWM_DUTY_MIN_LIMIT;
				else if(svpwm_u.SVPTc>PWM_DUTY_MAX_LIMIT)
					svpwm_u.SVPTc=PWM_DUTY_MAX_LIMIT;

				/*****************PWM输出*************************/
        /* 保存最终占空比对应的电压矢量, 回构比例与正向归一化同源 */
        {
            real32_T v_ap = foc_mod_vscale(DW.motor_anal.bus_Volt);
            real32_T ua = ((svpwm_u.SVPTa / half) - 1.0F) * v_ap;
            real32_T ub = ((svpwm_u.SVPTb / half) - 1.0F) * v_ap;
            real32_T uw = ((svpwm_u.SVPTc / half) - 1.0F) * v_ap;
            real32_T alpha = ((2.0F * ua) - ub - uw) / 3.0F;
            real32_T beta = (ub - uw) * MATH_1_SQRT3;
            s_valpha_applied = alpha;
            s_vbeta_applied = beta;
            if ((cmode == (uint16_T)Control_Mode_VF) ||
                (cmode == (uint16_T)Control_Mode_HighFre)) {
                DW.motor_anal.Vd_buf = (alpha * cos_th) + (beta * sin_th);
                DW.motor_anal.Vq_buf = (beta * cos_th) - (alpha * sin_th);
            }
        }
        DW.ctrl_hw_io.SVPTa = (uint16_T)svpwm_u.SVPTa;
        DW.ctrl_hw_io.SVPTb = (uint16_T)svpwm_u.SVPTb;
        DW.ctrl_hw_io.SVPTc = (uint16_T)svpwm_u.SVPTc;
    
}




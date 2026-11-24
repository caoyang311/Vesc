/******************************************************************************
 * 文件: foc_pi.c
 * 功能: PI 控制器函数实现 + 各环参数初始化
 * 创建日期: 2026-06-22
 * 修改日期: 2026-10-26
 * 修改内容: 饱和调度EXP=2时以乘法替代powf, 降低1kHz任务耗时
 * 修改人: rui.zhang
 ******************************************************************************/

#include "foc_data.h"
#include "foc_pi.h"
#include "foc_ctrl.h"            /* CURR_PI_SAT_SCHED_EN / CURR_PI_SAT_GAIN_MIN */
#include "MTPV_MAP.h"    /* Lookup_Ld/Lookup_Lq 饱和表 */
#include <math.h>

/* 与 foc_ctrl.h 开关族同源; 该头未生效(被并发编辑覆盖等)时的兜底默认 */
#ifndef CURR_PI_SAT_SCHED_EN
#define CURR_PI_SAT_SCHED_EN   1
#endif
#ifndef CURR_PI_SAT_GAIN_MIN
#define CURR_PI_SAT_GAIN_MIN   0.30F
#endif
#ifndef CURR_PI_SAT_EXP
#define CURR_PI_SAT_EXP        2.0F
#endif

/* 速度/位置环 PI: 设定值加权 b + 死区 + back-calculation 抗饱和 (源 PI_Contrl) */
void PI_Contrl(PI_Params *v)
{
    real32_T err   = (v->b * v->Ref) - v->Fbk;
    real32_T err_db = err;
    real32_T up, raw, sat, ui;

    /* 死区: |err| < DeadBand 时不响应 */
    if (fabsf(err) < v->DeadBand) {
        err_db = 0.0F;
    }

    /* 比例项 (含设定值加权) + 上一拍积分 */
    up  = v->Kp * err_db;
    raw = up + v->Ui;

    /* 预输出限幅, 得到饱和量 sat */
    if (raw > v->upper_limit)      { sat = v->upper_limit; }
    else if (raw < v->lower_limit) { sat = v->lower_limit; }
    else                           { sat = raw; }
    sat -= raw;

    /* 条件积分 + back-calculation: 误差与饱和方向不顶死时才积分 */
    if (((err_db <= 0.0F) || (sat >= 0.0F)) &&
        ((err_db >= 0.0F) || (sat <= 0.0F))) {
        ui = (v->Ki * err_db) + v->Ui + (v->Kaw * sat);
    } else {
        ui = v->Ui;
    }
    if (ui > v->upper_limit)      { ui = v->upper_limit; }
    else if (ui < v->lower_limit) { ui = v->lower_limit; }

    /* 输出限幅 */
    raw = up + ui;
    if (raw > v->upper_limit)      { v->PI_out = v->upper_limit; }
    else if (raw < v->lower_limit) { v->PI_out = v->lower_limit; }
    else                           { v->PI_out = raw; }

    v->Ui  = ui;
    v->Up  = up;
    v->Err = err;
}

	/* 电流/弱磁环 PI: 条件积分 + 动态限幅 (源 PI_Contrl_Modern / _Iq / _b) */
void PI_Contrl_Modern(PI_Params *v, uint8_T limit_state, real32_T Limit)
{
    real32_T upper = v->upper_limit;
    real32_T lower = v->lower_limit;
    real32_T err = v->Ref - v->Fbk;
    real32_T up, ui, vl;

    /* deadband: |err| < DeadBand stops integration */
    if (fabsf(err) < v->DeadBand) {
        err = 0.0F;
    }
    up = v->Kp * err;

    /* 方向性抗饱和: 饱和时只挡加深方向、放行回卷方向 */
    ui = (v->Ki * up) + v->Ui;
    if (v->PI_out != v->Vl) {   /* 输出正被钳位 */
        if (((v->PI_out >= v->upper_limit) && (ui > v->Ui)) ||
            ((v->PI_out <= v->lower_limit) && (ui < v->Ui))) {
            ui = v->Ui;   /* 只挡加深, 允许回卷 */
        }
    }
    /* 积分器钳位: 与输出同界, 防饱和期间积分自由累积后一次性释放 */
    if (ui > upper)      { ui = upper; }
    else if (ui < lower) { ui = lower; }
    vl = up + ui;

    /* 动态限幅 (Vdc 电压圆) */
    if (limit_state == 1U) {
        upper = Limit;
        lower = -Limit;
				v->lower_limit=lower;
				v->upper_limit=upper;
    }
    if (vl > upper)      { v->PI_out = upper; }
    else if (vl < lower) { v->PI_out = lower; }
    else                 { v->PI_out = vl; }

    v->Ui  = ui;
    v->Up  = up;
    v->Err = err;
    v->Vl  = vl;
}

/* PI 参数初始化 (与 MATLAB initialize 一致) */
void PI_Pare_init(void)
{
	    /* d 轴电流环 */
    DW.pi_ds.lower_limit = -20.0F;
    DW.pi_ds.Kp          = 0.2F;
    DW.pi_ds.Ki          = 0.0013F;
    DW.pi_ds.upper_limit = 41.1F;
    DW.pi_qs.DeadBand    = 0.0001F;
	
    /* q 轴电流环 */
    DW.pi_qs.lower_limit = -20.0F;
    DW.pi_qs.Kp          = 0.4F;
    DW.pi_qs.Ki          = 0.0013F;
    DW.pi_qs.upper_limit = 41.1F;
		DW.pi_ds.DeadBand    = 0.0001F;
    /* 速度环 (用 b/Kaw) */
    DW.pi_speed.lower_limit = -30.0F;
    DW.pi_speed.Kp          = 0.55F;
    DW.pi_speed.Ki          = 0.0015F;
    DW.pi_speed.upper_limit = 200.0F;
    DW.pi_speed.Kaw         = 0.0012F;
    DW.pi_speed.b           = 1.0F;

    /* 弱磁/电压环 */
    DW.pi_voltage.lower_limit = -5.0F;
    DW.pi_voltage.Kp          = 1.1F;
    DW.pi_voltage.Ki          = 0.001F;
    DW.pi_voltage.upper_limit = 5.0F;

    /* 位置环 (用 b/Kaw) */
    DW.pi_position.lower_limit = -200.0F;
    DW.pi_position.Kp          = 1.3F;
    DW.pi_position.Ki          = 0.0012F;
    DW.pi_position.upper_limit = 200.0F;
    DW.pi_position.Kaw         = 0.0012F;
    DW.pi_position.b           = 1.0F;

  /* comm_pi_bus (CAN A1 组可下载参数总线) 接入实际控制:
       上电未配置(全 0)时用上述硬编码默认填充, 保证闭环可用;
       CAN 下载新值后由 1kHz 任务中的 PI_Apply_CommParams 周期生效 */
    if (DW.comm_pi_bus.Iq_Kp <= 0.0F) {
        uint8_T i;
        DW.comm_pi_bus.Id_Kp       = DW.pi_ds.Kp;
        DW.comm_pi_bus.Id_Ki       = DW.pi_ds.Ki;
        DW.comm_pi_bus.Id_max      = DW.pi_ds.upper_limit;
        DW.comm_pi_bus.Iq_Kp       = DW.pi_qs.Kp;
        DW.comm_pi_bus.Iq_Ki       = DW.pi_qs.Ki;
        DW.comm_pi_bus.Iq_max      = DW.pi_qs.upper_limit;
        DW.comm_pi_bus.speed_Kp    = DW.pi_speed.Kp;
        DW.comm_pi_bus.speed_Ki    = DW.pi_speed.Ki;
        DW.comm_pi_bus.speed_max   = DW.pi_speed.upper_limit;
        DW.comm_pi_bus.voltage_Kp  = DW.pi_voltage.Kp;
        DW.comm_pi_bus.voltage_Ki  = DW.pi_voltage.Ki;
        DW.comm_pi_bus.voltage_max = DW.pi_voltage.upper_limit;
        DW.comm_pi_bus.position_Kp = DW.pi_position.Kp;
        DW.comm_pi_bus.position_Ki = DW.pi_position.Ki;
        DW.comm_pi_bus.position_max= DW.pi_position.upper_limit;
        for (i = 0U; i < 5U; i++) {
            DW.comm_pi_bus.Id_Kp_mul[i]    = 1.0F;
            DW.comm_pi_bus.Id_Ki_mul[i]    = 1.0F;
            DW.comm_pi_bus.Iq_Kp_mul[i]    = 1.0F;
            DW.comm_pi_bus.Iq_Ki_mul[i]    = 1.0F;
            DW.comm_pi_bus.speed_Kp_mul[i] = 1.0F;
            DW.comm_pi_bus.speed_Ki_mul[i] = 1.0F;
        }
    }
    PI_Apply_CommParams();
}

/* 将 comm_pi_bus 参数应用到实际控制环 (1kHz 周期调用)。
   分段 PI: 每个环各 5 段, 段宽由电机参数实时决定 ——
     电流环(Id/Iq): 自变量 = |本环电流指令|(pi_ds.Ref / pi_qs.Ref), 满量程 = 峰值电流 peak_curr;
     速度环:        自变量 = |机械转速反馈|(mech_Spd),          满量程 = 最大转速 peak_speed;
   两处都按满量程均分 5 段(第 1 段 0~满量程/5, 第 5 段至满量程, 超量程归第 5 段)。
   生效增益 = 基础增益 × 该段系数 mul[seg]。
   无效值处理: 该段 Kp 系数与 Ki 系数**任一 <= 0** 即该段无效 -> 不缩放(乘 1.0, 用基础增益);
   基础增益 <= 0 也视为无效 -> 不覆盖当前值; 限幅 <= 0 同理不覆盖。
   段宽每次实时算(每拍 2 次除法), 改 peak_curr/peak_speed 后段边界立即跟随。 */
static uint32_T PI_SegIndex(real32_T v, uint32_T full)
{
    uint32_T seg;

    if ((full == 0U) || (v <= 0.0F)) { return 0U; }
    seg = (uint32_T)(v * (5.0F / (real32_T)full));   /* 均分 5 段 */
    return (seg > 4U) ? 4U : seg;
}

/* 该段是否有效: Kp 与 Ki 系数都 > 0 才算配置过 */
static uint8_T PI_SegValid(real32_T kp_mul, real32_T ki_mul)
{
    return ((kp_mul > 0.0F) && (ki_mul > 0.0F)) ? 1U : 0U;
}

void PI_Apply_CommParams(void)
{
    const PI_Parameter_Bus *p = &DW.comm_pi_bus;
    uint32_T seg;
    real32_T km, ki;
    real32_T sat_rd = 1.0F, sat_rq = 1.0F;   /* 饱和调度系数(EN=0 时恒 1 = 不调度) */

#if (CURR_PI_SAT_SCHED_EN != 0)
    /* 饱和增益调度: L 随电流饱和下降, 固定增益 => 带宽∝1/L 上升, 深区极限环
       (台架数据10: |IqRef|>250A 后 Iq 实际 ±100A 振荡, 输出溢出→PWM异常→BKIN锁存)。
       系数 = (L_sat/L_nominal)^CURR_PI_SAT_EXP, 查既有 6x6 饱和表(电机适配本就配置该表,
       不新增整定量); 按上一拍指令工作点(平滑, 振荡不会经增益正反馈), 只降不升,
       下限 CURR_PI_SAT_GAIN_MIN。
       EXP=2 增量电感修正: 表是弦电感(ψ/i), 环增益跟增量电感(dψ/di)走, 深饱和增量<弦
       (Froelich 平方律; 台架数据12: EXP=1 把稳定区 250→~320A, 之上仍残余极限环,
        实测反推 深区 增量≈0.68×弦, r² 即 r×(增量/弦) 的自洽近似) */
    {
        real32_T id_op = DW.pi_ds.Ref;
        real32_T iq_op = fabsf(DW.pi_qs.Ref);
        real32_T ld_s  = Lookup_Ld(id_op, iq_op) * 1.0e-6F;   /* uH -> H */
        real32_T lq_s  = Lookup_Lq(id_op, iq_op) * 1.0e-6F;
        if ((ld_s > 1.0e-7F) && (DW.motor_param.Ld_value > 1.0e-7F)) {
            sat_rd = ld_s / DW.motor_param.Ld_value;
        }
        if ((lq_s > 1.0e-7F) && (DW.motor_param.Lq_value > 1.0e-7F)) {
            sat_rq = lq_s / DW.motor_param.Lq_value;
        }
        if (sat_rd > 1.0F) { sat_rd = 1.0F; }
        if (sat_rq > 1.0F) { sat_rq = 1.0F; }
        if (CURR_PI_SAT_EXP == 2.0F) {
            sat_rd *= sat_rd;
            sat_rq *= sat_rq;
        } else {
            sat_rd = powf(sat_rd, CURR_PI_SAT_EXP);
            sat_rq = powf(sat_rq, CURR_PI_SAT_EXP);
        }
        if (sat_rd < CURR_PI_SAT_GAIN_MIN) { sat_rd = CURR_PI_SAT_GAIN_MIN; }
        if (sat_rq < CURR_PI_SAT_GAIN_MIN) { sat_rq = CURR_PI_SAT_GAIN_MIN; }
    }
#endif

    /* ---- d 轴电流环: 按 |Id 指令| 分段, 满量程 = 峰值电流 ---- */
    seg = PI_SegIndex(fabsf(DW.pi_ds.Ref), (uint32_T)DW.motor_param.peak_curr);
    if (PI_SegValid(p->Id_Kp_mul[seg], p->Id_Ki_mul[seg]) != 0U) {
        km = p->Id_Kp_mul[seg];
        ki = p->Id_Ki_mul[seg];
    } else {
        km = 1.0F; ki = 1.0F;                 /* 该段无效: 不缩放 */
    }
    if (p->Id_Kp > 0.0F) { DW.pi_ds.Kp = p->Id_Kp * km * sat_rd; }
    if (p->Id_Ki > 0.0F) { DW.pi_ds.Ki = p->Id_Ki * ki * sat_rd; }
    if (p->Id_max > 0.0F) {
        DW.pi_ds.upper_limit =  p->Id_max;
        DW.pi_ds.lower_limit = -p->Id_max;
    }

    /* ---- q 轴电流环: 按 |Iq 指令| 分段, 满量程 = 峰值电流 ---- */
    seg = PI_SegIndex(fabsf(DW.pi_qs.Ref), (uint32_T)DW.motor_param.peak_curr);
    if (PI_SegValid(p->Iq_Kp_mul[seg], p->Iq_Ki_mul[seg]) != 0U) {
        km = p->Iq_Kp_mul[seg];
        ki = p->Iq_Ki_mul[seg];
    } else {
        km = 1.0F; ki = 1.0F;
    }
    if (p->Iq_Kp > 0.0F) { DW.pi_qs.Kp = p->Iq_Kp * km * sat_rq; }
    if (p->Iq_Ki > 0.0F) { DW.pi_qs.Ki = p->Iq_Ki * ki * sat_rq; }
    if (p->Iq_max > 0.0F) {
        DW.pi_qs.upper_limit =  p->Iq_max;
        DW.pi_qs.lower_limit = -p->Iq_max;
    }

    /* ---- 速度环: 按 |机械转速| 分段, 满量程 = 最大转速 ---- */
    seg = PI_SegIndex(fabsf((real32_T)DW.motor_main_feedback.mech_Spd),
                      (uint32_T)DW.motor_param.peak_speed);
    if (PI_SegValid(p->speed_Kp_mul[seg], p->speed_Ki_mul[seg]) != 0U) {
        km = p->speed_Kp_mul[seg];
        ki = p->speed_Ki_mul[seg];
    } else {
        km = 1.0F; ki = 1.0F;
    }
    if (p->speed_Kp > 0.0F) { DW.pi_speed.Kp = p->speed_Kp * km; }
    if (p->speed_Ki > 0.0F) { DW.pi_speed.Ki = p->speed_Ki * ki; }
    if (p->speed_max > 0.0F) {
        DW.pi_speed.upper_limit =  p->speed_max;
        DW.pi_speed.lower_limit = -p->speed_max;
    }

    /* ---- 弱磁/电压环 (无分段) ---- */
    if (p->voltage_Kp > 0.0F) { DW.pi_voltage.Kp = p->voltage_Kp; }
    if (p->voltage_Ki > 0.0F) { DW.pi_voltage.Ki = p->voltage_Ki; }
    if (p->voltage_max > 0.0F) {
        DW.pi_voltage.upper_limit =  p->voltage_max;
        DW.pi_voltage.lower_limit = -p->voltage_max;
    }

    /* ---- 位置环 (无分段) ---- */
    if (p->position_Kp > 0.0F) { DW.pi_position.Kp = p->position_Kp; }
    if (p->position_Ki > 0.0F) { DW.pi_position.Ki = p->position_Ki; }
    if (p->position_max > 0.0F) {
        DW.pi_position.upper_limit =  p->position_max;
        DW.pi_position.lower_limit = -p->position_max;
    }
}
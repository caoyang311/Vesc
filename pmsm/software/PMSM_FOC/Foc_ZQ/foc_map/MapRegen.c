/******************************************************************************
 * 文件: MapRegen.c
 * 功能: MTPA/MTPV MAP 在线重算 (移植上位机 IPMSM Calc v4.6 计算内核)
 * 创建日期: 2026-10-10
 * 修改日期: 2026-10-10
 * 修改内容: MTPA 牛顿精确解 + 弱磁电压椭圆右边缘解(退磁限/电流圆/Iq二分);
 *           重算收尾一并重建 Ld/Lq 饱和表(Froelich, 轴随 Imax 重排);
 *           按当前表轴网格逐扭矩行重算 Id/Iq 镜像(轴/尺寸不动), 带电压/扭矩自检
 * 修改人: rui.zhang
 ******************************************************************************/

/* 触发链: CAN 0x20/39 idx0 cmd=0x20 置 g_map_regen_req -> 主循环 1ms 任务
   (SignalProc_PwmZeroCalib) 以当前 DW.motor_param 启动 -> 每调用推进一扭矩行,
   电机使能中途拉起则中止。结果经 0x21/01 状态字读回, 统计量供调试器观察。
   参数口径: psi(Wb) Ld/Lq(H) Rs(Ohm) udc(V) imax(A) 均为 SI, 与 DW 一致。 */

#include <math.h>
#include <stdint.h>
#include "MTPV_MAP.h"

/* 与运行时可达性口径对齐: 电压椭圆留 5%, Id 上限收 0.95×电流圆 */
#define MAPREGEN_VFRAC      0.95F
#define MAPREGEN_ID_FRAC    0.95F

volatile uint8_t g_map_regen_req    = 0U;   /* CAN 置位, 主循环消费 */
uint8_t  g_map_regen_st     = MAPREGEN_ST_IDLE;
uint16_t g_map_regen_vlim   = 0U;           /* 电压受限格数(实际电压超限, 0.5V 容差) */
float    g_map_regen_errmax = 0.0F;         /* 非受限格最大扭矩偏差 N.m(信息量) */

static uint8_t  s_regen_run = 0U;
static uint16_t s_regen_row = 0U;
static uint16_t s_bad_cnt   = 0U;
static float s_psi, s_ld, s_lq, s_rs, s_pp, s_udc, s_imax;

/* MTPA 闭式 Id = k - sqrt(k^2 + iq^2), k = psi/(2(Lq-Ld)); Ld>=Lq(SPMSM)时 Id=0 */
static float regen_mtpa_id(float iq_mag, float psi, float ld, float lq)
{
    float k;
    if ((psi <= 0.0F) || ((lq - ld) <= 1.0e-12F)) { return 0.0F; }
    k = psi / (2.0F * (lq - ld));
    return k - sqrtf((k * k) + (iq_mag * iq_mag));
}

/* MTPA 精确解: 牛顿迭代逼近目标扭矩, 电流圆钳位 */
static void regen_mtpa(float te, float psi, float ld, float lq, float pp, float imax,
                       float *id_out, float *iq_out)
{
    float k, iq, id, is, sc;
    uint16_t it;

    if ((psi <= 0.0F) || (te <= 0.0F)) { *id_out = 0.0F; *iq_out = 0.0F; return; }
    if (fabsf(lq - ld) < 1.0e-12F) {
        iq = te / (1.5F * pp * psi);
        if (iq > imax) { iq = imax; }
        *id_out = 0.0F;
        *iq_out = iq;
        return;
    }
    k  = psi / (2.0F * (lq - ld));
    iq = te / (1.5F * pp * psi);
    if (iq < 0.001F) { iq = 0.001F; }
    for (it = 0U; it < 60U; it++) {
        float ks = (k * k) + (iq * iq);
        float sq = (ks > 1.0e-30F) ? sqrtf(ks) : 1.0e-30F;
        float te_c, dte, iq_new;

        id   = k - sq;
        te_c = 1.5F * pp * ((psi * iq) + ((ld - lq) * id * iq));
        dte  = 1.5F * pp * (psi + ((ld - lq) * (((-iq / sq) * iq) + id)));
        if (fabsf(dte) < 1.0e-12F) { break; }
        iq_new = iq + ((te - te_c) / dte);
        if (iq_new < 0.001F) { iq_new = 0.001F; }
        if (iq_new > imax)   { iq_new = imax; }
        if (fabsf(iq_new - iq) < 1.0e-5F) { iq = iq_new; break; }
        iq = iq_new;
    }
    id = k - sqrtf((k * k) + (iq * iq));
    is = sqrtf((id * id) + (iq * iq));
    if (is > imax) {
        sc = imax / is;
        id *= sc;
        iq *= sc;
    }
    if (id < -(MAPREGEN_ID_FRAC * imax)) { id = -(MAPREGEN_ID_FRAC * imax); }
    *id_out = id;
    *iq_out = iq;
}

/* 弱磁解: 电压椭圆右边缘(+根), 退磁限与电流圆约束, Id 钳位下二分最大可行 Iq */
static void regen_fw(float we, float iq_req, float psi, float ld, float lq, float rs,
                     float umax, float imax, float *id_out, float *iq_out)
{
    float id_floor, vd0, vq0, id, iq_act, vq;
    uint16_t it;

    id_floor = (ld > 0.0F) ? (psi / ld) : imax;   /* 退磁限与电流圆取严(正值) */
    if (id_floor > (MAPREGEN_ID_FRAC * imax)) { id_floor = MAPREGEN_ID_FRAC * imax; }

    if (we < 1.0e-6F) {                           /* 静止: MTPA 闭式 + 退磁限 */
        id = regen_mtpa_id(iq_req, psi, ld, lq);
        if (id < -id_floor) { id = -id_floor; }
        *id_out = id;
        *iq_out = iq_req;
        return;
    }
    vd0 = -we * lq * iq_req;
    vq0 =  (rs * iq_req) + (we * psi);
    if (sqrtf((vd0 * vd0) + (vq0 * vq0)) <= (umax + 1.0e-4F)) {
        /* 恒扭矩区: Id=0 点可行则 MTPA 点更可行(Ld*Id<0 压低 Vq) */
        id = regen_mtpa_id(iq_req, psi, ld, lq);
        if (id < -id_floor) { id = -id_floor; }
        *id_out = id;
        *iq_out = iq_req;
        return;
    }
    id = -id_floor;
    {
        float a = (rs * rs) + ((we * ld) * (we * ld));
        float A = we * lq * iq_req;
        float D = (rs * iq_req) + (we * psi);
        float b = (-2.0F * rs * A) + (2.0F * we * ld * D);
        float c = (A * A) + (D * D) - (umax * umax);
        float disc = (b * b) - (4.0F * a * c);
        if ((a > 1.0e-12F) && (disc >= 0.0F)) {
            float sq = sqrtf(disc);
            float id1 = (-b + sq) / (2.0F * a);
            float id2 = (-b - sq) / (2.0F * a);
            uint8_t found = 0U;
            if ((id1 <= 0.0F) && (id1 >= -id_floor) &&
                (sqrtf((id1 * id1) + (iq_req * iq_req)) <= (imax + 1.0e-4F))) {
                id = id1; found = 1U;
            } else if ((id2 <= 0.0F) && (id2 >= -id_floor) &&
                       (sqrtf((id2 * id2) + (iq_req * iq_req)) <= (imax + 1.0e-4F))) {
                id = id2; found = 1U;
            }
            if (found == 0U) { id = -id_floor; }
        }
    }
    /* Id 固定后 |V| 随 Iq 单调增: 二分最大可行 Iq */
    {
        float t_ = (imax * imax) - (id * id);
        iq_act = (t_ > 0.0F) ? sqrtf(t_) : 0.0F;
        if (iq_act > iq_req) { iq_act = iq_req; }
    }
    vq = (rs * iq_act) + (we * ((ld * id) + psi));   /* Vq 与 Iq 有关(含 Rs) */
    {
        float vdh = (rs * id) - (we * lq * iq_act);
        float err_hi = sqrtf((vdh * vdh) + (vq * vq)) - umax;
        if (err_hi > 0.0F) {
            float lo = 0.0F, hi = iq_act;
            if (fabsf(vq) - umax < 0.0F) {        /* Iq=0 仍欠压才有二分区间 */
                for (it = 0U; it < 40U; it++) {
                    float mid = 0.5F * (lo + hi);
                    float vdm = (rs * id) - (we * lq * mid);
                    float vqm = (rs * mid) + (we * ((ld * id) + psi));
                    float em  = sqrtf((vdm * vdm) + (vqm * vqm)) - umax;
                    if (em < 0.0F) { lo = mid; } else { hi = mid; }
                }
                iq_act = lo;
            } else {
                iq_act = 0.0F;
            }
        }
    }
    *id_out = id;
    *iq_out = iq_act;
}

/* (Te, we) -> (Id, Iq): MTPA 点电压可行取 MTPA, 否则弱磁; vlm=实际转矩低于需求 */
static void regen_idq(float te, float we, float rs, float psi, float ld, float lq,
                      float pp, float umax, float imax,
                      float *id_out, float *iq_out, uint8_t *vlm)
{
    float id_m, iq_m, vd, vq, id_f, iq_f, te_f;
    float ld_use = ld, lq_use = lq;

    *vlm = 0U;
    if (te == 0.0F) { *id_out = 0.0F; *iq_out = 0.0F; return; }
    regen_mtpa(fabsf(te), psi, ld_use, lq_use, pp, imax, &id_m, &iq_m);
    /* 一轮表查询修正: 用当前活动 Ld/Lq 表替换标量, 再解一次; 表未修好时仍回落标量 */
    {
        float ld1 = Lookup_Ld(id_m, iq_m) * 1.0e-6F;
        float lq1 = Lookup_Lq(id_m, iq_m) * 1.0e-6F;
        if ((lq1 > ld1) && (lq1 > 1.0e-9F) &&
            ((fabsf(ld1 - ld_use) > 1.0e-9F) || (fabsf(lq1 - lq_use) > 1.0e-9F))) {
            ld_use = ld1; lq_use = lq1;
            regen_mtpa(fabsf(te), psi, ld_use, lq_use, pp, imax, &id_m, &iq_m);
        }
    }
    if (te < 0.0F) { iq_m = -iq_m; }
    if (we <= 1.0e-6F) {
        *id_out = id_m;
        *iq_out = iq_m;
        return;
    }
    vd = (rs * id_m) - (we * lq_use * iq_m);
    vq = (rs * iq_m) + (we * ((ld_use * id_m) + psi));
    if (sqrtf((vd * vd) + (vq * vq)) <= (umax + 1.0e-3F)) {
        *id_out = id_m;
        *iq_out = iq_m;
        return;
    }
    regen_fw(we, fabsf(iq_m), psi, ld_use, lq_use, rs, umax, imax, &id_f, &iq_f);
    te_f = 1.5F * pp * ((psi * iq_f) + ((ld_use - lq_use) * id_f * iq_f));
    if (te < 0.0F) { iq_f = -iq_f; }
    *id_out = id_f;
    *iq_out = iq_f;
    if (fabsf(te_f) < (fabsf(te) - 1.0e-3F)) { *vlm = 1U; }
}

/* 启动重算: 校验电机参数并快照; 轴/尺寸/其余镜像内容不动 */
uint8_t MapStore_RegenStart(float psi, float ld, float lq, float rs,
                            float pp, float udc, float imax)
{
    if (!(psi > 1.0e-6F) || !(ld > 1.0e-9F) || !(lq > 1.0e-9F) ||
        (lq <= ld) ||                     /* 本 MTPA 公式只适用于 Lq>Ld */
        !(pp >= 1.0F) || !(udc >= 10.0F) || !(imax >= 1.0F)) {
        g_map_regen_st  = MAPREGEN_ST_REJ_PAR;
        g_map_rw_status = 0x03U;   /* 0x21/01: 数据超范围 */
        return 0U;
    }
    s_psi = psi; s_ld = ld; s_lq = lq; s_rs = rs;
    s_pp  = pp; s_udc = udc; s_imax = imax;
    s_regen_row = 0U;
    s_bad_cnt   = 0U;
    g_map_regen_vlim   = 0U;
    g_map_regen_errmax = 0.0F;
    s_regen_run = 1U;
    g_map_regen_st = MAPREGEN_ST_BUSY;
    return 1U;
}

uint8_t MapStore_RegenBusy(void)
{
    return s_regen_run;
}

/* 电机中途使能时中止: 已写完的行保留(行内自洽), 状态标记运行中止 */
void MapStore_RegenAbort(void)
{
    if (s_regen_run != 0U) {
        s_regen_run = 0U;
        g_map_regen_st = MAPREGEN_ST_REJ_RUN;
    }
}

/* Ld/Lq 饱和表不再由重算覆盖: MAP 只读当前活动表(或调用方先修好表),
   重算结束不得改 Ld_Id_Axis/Lq_* 与 Ld/Lq_Sat_Map —— 否则实测表会被 Froelich 冲掉。 */

/* 每调用推进一扭矩行(1ms 任务分步, 避免长阻塞); 末行收尾自检汇总 */
void MapStore_RegenTask(void)
{
    uint16_t j;
    float umax, te;

    if (s_regen_run == 0U) { return; }
    umax = s_udc * 0.5773502692F * MAPREGEN_VFRAC;
    te   = Torque_Table[s_regen_row];
    for (j = 0U; j < g_map_spd_pts; j++) {
        float we  = Speed_Table[j] * 0.1047197551F * s_pp;   /* rpm -> we(rad/s) */
        float id_, iq_, ud, uq, te_a, err;
        uint8_t vlm;

        regen_idq(te, we, s_rs, s_psi, s_ld, s_lq, s_pp, umax, s_imax, &id_, &iq_, &vlm);
        Id_Map[s_regen_row][j] = id_;
        Iq_Map[s_regen_row][j] = iq_;
        if (!((id_ > -1.0e6F) && (id_ < 1.0e6F) &&
              (iq_ > -1.0e6F) && (iq_ < 1.0e6F))) {      /* 含 NaN 防护 */
            s_bad_cnt++;
            continue;
        }
        ud = (s_rs * id_) - (we * s_lq * iq_);
        uq = (s_rs * iq_) + (we * ((s_ld * id_) + s_psi));
        if (sqrtf((ud * ud) + (uq * uq)) > (umax + 0.5F)) { g_map_regen_vlim++; }
        te_a = 1.5F * s_pp * ((s_psi * iq_) + ((s_ld - s_lq) * id_ * iq_));
        err  = fabsf(te_a - te);
        if ((vlm == 0U) && (err > g_map_regen_errmax)) { g_map_regen_errmax = err; }
    }
    s_regen_row++;
    if (s_regen_row >= g_map_tq_pts) {
        s_regen_run = 0U;
        g_map_regen_st  = ((s_bad_cnt == 0U) && (g_map_regen_vlim == 0U))
                          ? MAPREGEN_ST_OK : MAPREGEN_ST_WARN;
        g_map_rw_status = ((s_bad_cnt == 0U) && (g_map_regen_vlim == 0U))
                          ? 0x01U : 0x03U;   /* 0x21/01 */
    }
}

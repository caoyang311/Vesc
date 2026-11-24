/******************************************************************************
 * 文件: signal_proc.c
 * 功能: GPIO 输入识别/去抖 (按键/档位状态机) + 模拟量转换/滤波 + PWM 零位校准任务
 * 创建日期: 2026-06-22
 * 修改日期: 2026-10-26
 * 修改内容: MAP 在线重算与运行时共用同一电流圆接口
 * 修改人: rui.zhang
 ******************************************************************************/

#include <stdint.h>
#include "foc_data.h"
#include "foc_ctrl.h"
#include "signal_proc.h"
#include "temp_sensor.h"                /* 电阻值 -> 温度(℃) 转换 (移植自参考工程) */
#include "MotorCtr_parameter.h"   /* ADC_ZERO_TARGET, CONVERT_*, 去抖目标值 */
#include "encoder_pwm_u.h"              /* EncPwmSamp_* PWM raw 去卷绕采样器 */
#include "canopen.h"                /* g_calib_commit_req (标定固化请求) */
#include "MTPV_MAP.h"     /* MapStore_Regen* (MAP 在线重算, 0x20/01 cmd=0x20) */
#include "encoder_foc_sdk.h"            /* g_enc_foc, ENC_ZeroOffset_FromPwmRaw, g_zero_ref_save_pending */
#include "param_store.h"       /* ParamStore_ZeroRef_Save (存储请求处理) */
#include <math.h>
/* ******************** 参数 (取自 MATLAB MotorCtr_parameter / port_idf 调用) ******************** */
#define KEY_ACTIVE        1U      			/* 按键有效 */
#define KEY_INVALID      (!KEY_ACTIVE)  /* 按键无效 */
#define GPIO_ACTIVE       0U      /* GPIO 有效电平 */


#define CNT_BRAKE         50U      /* 刹车去抖目标 (ms) */
#define CNT_GPIO          50U      /* 一般 GPIO 去抖目标 (ms) */
#define GEAR_STABLE       50U      /* 档位电平稳定阈值 */
#define T_LONG            2000U   /* 长按阈值 (ms) */
#define T_CONTINUATE      500U    /* 连击(双击/三击)间隔窗口 (ms) */
#define T_STUCK           5000U   /* 卡死阈值 (ms) */

/* 按键状态机内部状态 */
typedef enum {
    K_IDLE = 0,        /* 空闲 */
    K_PRESS,           /* 第一下按下中 */
    K_WAIT_2ND,        /* 等待第二下 (多击) */
    K_PRESS_2ND,       /* 第二下按下中 (多击) */
    K_WAIT_3RD,        /* 等待第三下 (多击) */
    K_PRESS_3RD,       /* 第三下按下中 (多击) */
    K_LONG,            /* 长按态 */
    K_STUCK            /* 卡滞态 */
} KeyStateE;

typedef struct {
    uint16_t press_cnt;   /* 按住计时 (ms) */
    uint16_t wait_cnt;    /* 多击窗口计时 (ms) */
    uint8_t  state;       /* KeyStateE */
} KeyFsm;

/* 档位状态机 */
typedef enum { G_ECO = 0, G_NOR, G_SPORT } GearStateE;

/* ******************** 模块状态 (整数化, uint16 计数) ******************** */
struct {
    /* 5 个走状态机的按键 */
    KeyFsm   brake, boost, p, r, ccs;
    /* 各按键/信号去抖计数 + 上拍去抖输出(= key_valid 的 u_sta) */
    uint16_t cnt_brake, cnt_side, cnt_boost, cnt_p, cnt_r, cnt_ccs, cnt_burglar, cnt_acc,cnt_gear_bus[2];
    uint8_t  last_brake, last_side, last_boost, last_p, last_r, last_ccs, last_burglar, last_acc,last_gear_bus[2];
    /* 档位去抖 + 状态机 */
    uint16_t cnt_g1, cnt_g2, gear_cnt;
    uint8_t  raw_g1, raw_g2;       /* 上拍档位原始电平 (value_unchange 比较用) */
    uint8_t  db_g1, db_g2;         /* 去抖后档位电平 */
    uint8_t  gear_state;           /* GearStateE */
    uint8_t  gear_release;         /* 按键已释放标志 */
    uint8_t  gear_out;             /* 档位输出 (0/1/2/254/255) */
} s_gpio;

/* 模拟量滤波状态 */
static struct {
    uint16_t adc_cnt;                 /* 零位校准计数 */
    uint32_t sum_bus, sum_u, sum_v,sum_w;   /* 校准累加 */
    float    f_bus_curr, f_u_curr, f_v_curr,f_w_curr;   /* 电流 LPF */
    float    f_bus_volt, f_bus_volt_slow, f_mos_rs, f_motor_rs, f_acc; /* 电压/温度/油门 LPF */
} s_an;
/* PWM 零位校准状态: 0=等待条件 1=采集中 2=完成 3=放弃(条件不满足/采样失败) */
static uint8_t  s_zero_calib_state = 0U;
static uint16_t s_zero_calib_timer = 0U;   /* 状态计时 (ms) */
static uint8_t  s_zero_calib_retry = 0U;   /* 采样失败重试计数 */
static uint16_t s_zeroref_save_tick = 0U;  /* 零位基准写入重试节流(100ms) */

#define ZERO_CALIB_SPD_GUARD    30.0f    /* 静止守卫: |转速| < 30 RPM */
#define ZERO_CALIB_WAIT_MAX_MS  5000U    /* 等待条件满足的最长时间 */
#define ZERO_CALIB_SAMP_FRAMES  64U      /* 采样帧数 (~66ms @971Hz) */
#define ZERO_CALIB_SAMP_MAX_MS  1000U    /* 采样超时 (PWM信号丢失判定) */
#define ZERO_CALIB_SPREAD_MAX   60U      /* 采样展宽上限 (clocks, ≈5.3°机械) */
#define ZERO_CALIB_RETRY_MAX    3U       /* 采样失败重试次数 */
/* ******************** 去抖辅助 ******************** */

/* 按键型去抖: 连续 target 拍有效后输出锁存 1; 失效立即清 0 (源 key_valid/key_invalid) */
static uint8_t key_debounce(uint8_t gpio, uint16_t target, uint16_t *cnt, uint8_t *sta)
{
    if (gpio == GPIO_ACTIVE) {
        if (*cnt >= target) { *sta = 1U; *cnt = 0U; }
        else                { *cnt = (uint16_t)(*cnt + 1U); }   /* 计数中保持上拍输出 */
    } else {
        *cnt = 0U;
        *sta = 0U;
    }
    return *sta;
}

/* 电平型去抖: 电平稳定 GEAR_STABLE 拍后才更新输出 (源 value_unchange/value_change) */
static uint8_t level_debounce(uint8_t gpio, uint16_t *cnt, uint8_t *raw_last, uint8_t *out)
{
    if (gpio == *raw_last) {
        if (*cnt >= GEAR_STABLE) { *out = gpio; *cnt = 0U; }
        else                     { *cnt = (uint16_t)(*cnt + 1U); }
    } else {
        *cnt = 1U;   /* 电平变化, 输出保持, 重新计时 */
    }
    *raw_last = gpio;
    return *out;
}

/* 计数自增并封顶 65535 */
static inline void cnt_inc(uint16_t *c)
{
    if (*c < 65535U) { (*c)++; }
}

/* ******************** 按键状态机 ******************** */
/*
 * 参数:
 *   key      : 去抖后的按键电平 (0/1)
 *   f        : 状态机实例
 *   multi_en : 1=支持多击(单击/双击/三击), 0=仅单击/长按 (P/CCS=1, brake/boost/R=0)
 *   stuck_en : 1=使能卡滞(持续>T_STUCK 输出255), 0=不检测 (brake=0)
 * 返回: 按键输出码 (对齐接口文档, 见 signal_proc.h KEY_OUT_*)
 *   0=未按下/等待窗口, 1=第一下按下中, 2=单击, 3=第二下按下,
 *   4=双击, 5=长按, 6=第三下按下, 255=卡滞
 * 说明: 单击/双击在状态转移瞬间单拍输出, 下一拍回 0; 等待窗口期间输出 0。
 */
static uint8_t key_fsm(uint8_t key, KeyFsm *f, uint8_t multi_en, uint8_t stuck_en)
{
    uint8_t pressed = (uint8_t)(key == KEY_ACTIVE);
    uint8_t out = KEY_OUT_NONE;   /* 默认 0: 空闲 / 等待窗口 */

    switch (f->state) {
    /* ---------- 空闲 ---------- */
    case K_IDLE:
        if (pressed) {
            f->state = K_PRESS;
            f->press_cnt = 0U;
            out = KEY_OUT_HOLD;
        }
        break;

    /* ---------- 第一下按下中 (输出1) ---------- */
    case K_PRESS:
        out = KEY_OUT_HOLD;
        if (!pressed) {
            if (multi_en) {
                f->state = K_WAIT_2ND;     /* 多击: 进等待窗口, 输出0 */
                f->wait_cnt = 0U;
                out = KEY_OUT_SINGLE;
            } else {
                f->state = K_IDLE;         /* 简单型: 松开即单击 */
                out = KEY_OUT_SINGLE;
            }
        } else if (f->press_cnt >= T_LONG) {
            f->state = K_LONG;
            out = KEY_OUT_LONG;
        } else {
            cnt_inc(&f->press_cnt);
        }
        break;

    /* ---------- 等待第二下 (输出0) ---------- */
    case K_WAIT_2ND:
        if (pressed) {
            f->state = K_PRESS_2ND;
            f->press_cnt = 0U;
            out = KEY_OUT_PRESS_2ND;
        } else if (f->wait_cnt >= T_CONTINUATE) {
            f->state = K_IDLE;             /* 超时: 确认单击 */
            out = KEY_OUT_SINGLE;
        } else {
            cnt_inc(&f->wait_cnt);
        }
        break;

    /* ---------- 第二下按下中 (输出3) ---------- */
    case K_PRESS_2ND:
        out = KEY_OUT_PRESS_2ND;
        if (!pressed) {
            f->state = K_WAIT_3RD;         /* 进等待第三下窗口, 输出0 */
            f->wait_cnt = 0U;
            out = KEY_OUT_NONE;
        } else if (f->press_cnt >= T_LONG) {
            f->state = K_LONG;
            out = KEY_OUT_LONG;
        } else {
            cnt_inc(&f->press_cnt);
        }
        break;

    /* ---------- 等待第三下 (输出0) ---------- */
    case K_WAIT_3RD:
        if (pressed) {
            f->state = K_PRESS_3RD;
            f->press_cnt = 0U;
            out = KEY_OUT_PRESS_3RD;
        } else if (f->wait_cnt >= T_CONTINUATE) {
            f->state = K_IDLE;             /* 超时: 确认双击 */
            out = KEY_OUT_DOUBLE;
        } else {
            cnt_inc(&f->wait_cnt);
        }
        break;

    /* ---------- 第三下按下中 (输出6) ---------- */
    case K_PRESS_3RD:
        out = KEY_OUT_PRESS_3RD;
        if (!pressed) {
            f->state = K_IDLE;             /* 第三下松开: 双击延续 */
            out = KEY_OUT_DOUBLE;
        } else if (f->press_cnt >= T_LONG) {
            f->state = K_LONG;
            out = KEY_OUT_LONG;
        } else {
            cnt_inc(&f->press_cnt);
        }
        break;

    /* ---------- 长按 (输出5) ---------- */
    case K_LONG:
        out = KEY_OUT_LONG;
        if (!pressed) {
            f->state = K_IDLE;
            out = KEY_OUT_NONE;
        } else if (stuck_en && (f->press_cnt >= T_STUCK)) {
            f->state = K_STUCK;
            out = KEY_OUT_STUCK;
        } else {
            cnt_inc(&f->press_cnt);
        }
        break;

    /* ---------- 卡滞 (输出255) ---------- */
    case K_STUCK:
        out = KEY_OUT_STUCK;
        if (!pressed) {
            f->state = K_IDLE;
            out = KEY_OUT_NONE;
        }
        break;

    default:
        f->state = K_IDLE;
        break;
    }
    return out;
}

/* ******************** 档位状态机 (非循环: 0<->1<->2, 卡滞255/254) ******************** */
/* g1=SDH=升档(UP), g2=SDL=降档(DOWN)。去抖后两路电平在 ECO<->NOR<->SPORT 间非循环切换。
 * 一次按下一次换挡(gear_release 锁存, 两键都松开清零)。
 * 持续按下超 T_STUCK(5s): 升档键(g1)=255, 降档键(g2)=254; 两键松开恢复实际档位(清除卡滞)。
 * 注: GearStateE 枚举序 0/1/2 与 GEAR_OUT_ECO/NOR/SPORT 一致, 故 gear_out 可直接取 gear_state。 */
static void gear_fsm(uint8_t g1, uint8_t g2)
{
    uint8_t up   = (uint8_t)((g1 == KEY_ACTIVE) && (g2 == KEY_INVALID));   /* SDH 升档键按下 */
    uint8_t down = (uint8_t)((g1 == KEY_INVALID) && (g2 == KEY_ACTIVE));   /* SDL 降档键按下 */

    if (up && (s_gpio.gear_release == 0U)) {
        /* 升档: ECO->NOR->SPORT, 已在最高档则不变 */
        if (s_gpio.gear_state != (uint8_t)G_SPORT) {
            s_gpio.gear_state++;
            s_gpio.gear_out = s_gpio.gear_state;
        }
        s_gpio.gear_release = 1U;
        cnt_inc(&s_gpio.gear_cnt);
    } else if (down && (s_gpio.gear_release == 0U)) {
        /* 降档: SPORT->NOR->ECO, 已在最低档则不变 */
        if (s_gpio.gear_state != (uint8_t)G_ECO) {
            s_gpio.gear_state--;
            s_gpio.gear_out = s_gpio.gear_state;
        }
        s_gpio.gear_release = 1U;
        cnt_inc(&s_gpio.gear_cnt);
    } else if ((g1 == KEY_INVALID) && (g2 == KEY_INVALID)) {
        /* 两键都松开: 解除锁存, 恢复实际档位 (清除卡滞码) */
        s_gpio.gear_release = 0U;
        s_gpio.gear_cnt = 0U;
        s_gpio.gear_out = s_gpio.gear_state;
    } else if (s_gpio.gear_cnt >= T_STUCK) {
        /* 按键持续按下超过 5s: 卡滞。升档键(g1)=255, 降档键(g2)=254 */
        s_gpio.gear_out = (g1 == KEY_ACTIVE) ? GEAR_OUT_STUCK_UP : GEAR_OUT_STUCK_DN;
    } else {
			cnt_inc(&s_gpio.gear_cnt);
    }
}

/* ******************** 一阶低通滤波 y = a*x + (1-a)*y ******************** */
static inline float lpf(float y_prev, float x, float a)
{
    return (a * x) + ((1.0f - a) * y_prev);
}

/* ******************** 公共接口 ******************** */

void SignalProc_Init(void)
{
    uint32_t i;
    uint8_t *p = (uint8_t *)&s_gpio;
    for (i = 0U; i < sizeof(s_gpio); i++) { p[i] = 0U; }
    p = (uint8_t *)&s_an;
    for (i = 0U; i < sizeof(s_an); i++) { p[i] = 0U; }

    s_gpio.brake.state = (uint8_t)K_IDLE;
    s_gpio.boost.state = (uint8_t)K_IDLE;
    s_gpio.p.state     = (uint8_t)K_IDLE;
    s_gpio.r.state     = (uint8_t)K_IDLE;
    s_gpio.ccs.state   = (uint8_t)K_IDLE;
    s_gpio.gear_state  = (uint8_t)G_ECO;
    s_gpio.gear_release = 1U;   /* 与 MATLAB 初始化一致 */
}

void SignalProc_GpioIdentify(void)
{
    uint8_t b;

    /* 刹车: 去抖(50) -> 状态机(简单型, 无多击/无卡滞, 仅长按=5) */
    b = key_debounce(DW.ctrl_hw_io.brake_gpio, CNT_BRAKE, &s_gpio.cnt_brake, &s_gpio.last_brake);
    DW.ctrl_hw_io.brake_o = key_fsm(b, &s_gpio.brake, 0U, 0U);

    /* 边撑: 仅去抖 */
    DW.ctrl_hw_io.side_stand_o =
        key_debounce(DW.ctrl_hw_io.side_stand_gpio, CNT_GPIO, &s_gpio.cnt_side, &s_gpio.last_side);

    /* 档位: 两路电平去抖 -> 档位状态机 (非循环, g1=SDH升档/g2=SDL降档) */
    {
//			s_gpio.raw_g1 = key_debounce(DW.ctrl_hw_io.gear_bus_gpio[0], CNT_GPIO, &s_gpio.cnt_gear_bus[0], &s_gpio.last_gear_bus[0]);
//			s_gpio.raw_g2 =  key_debounce(DW.ctrl_hw_io.gear_bus_gpio[1], CNT_GPIO, &s_gpio.cnt_gear_bus[1], &s_gpio.last_gear_bus[1]);
        uint8_t g1 = key_debounce(DW.ctrl_hw_io.gear_bus_gpio[0], CNT_GPIO, &s_gpio.cnt_gear_bus[0], &s_gpio.last_gear_bus[0]);
        uint8_t g2 = key_debounce(DW.ctrl_hw_io.gear_bus_gpio[1], CNT_GPIO, &s_gpio.cnt_gear_bus[1], &s_gpio.last_gear_bus[1]);
        gear_fsm(g1, g2);
        DW.ctrl_hw_io.gear_o = s_gpio.gear_out;
    }

    /* 助推: 去抖 -> 状态机(简单型, 卡滞检测) */
    b = key_debounce(DW.ctrl_hw_io.boost_gpio, CNT_GPIO, &s_gpio.cnt_boost, &s_gpio.last_boost);
    DW.ctrl_hw_io.boost_o = key_fsm(b, &s_gpio.boost, 0U, 1U);

    /* P 档: 去抖 -> 状态机(多击型含三击, 卡滞检测) */
    b = key_debounce(DW.ctrl_hw_io.P_gpio, CNT_GPIO, &s_gpio.cnt_p, &s_gpio.last_p);
    DW.ctrl_hw_io.P_o = key_fsm(b, &s_gpio.p, 1U, 1U);

    /* R 档: 去抖 -> 状态机(简单型, 卡滞检测) */
    b = key_debounce(DW.ctrl_hw_io.R_gpio, CNT_GPIO, &s_gpio.cnt_r, &s_gpio.last_r);
    DW.ctrl_hw_io.R_o = key_fsm(b, &s_gpio.r, 0U, 1U);

    /* 定速巡航: 去抖 -> 状态机(多击型含三击, 卡滞检测) */
    b = key_debounce(DW.ctrl_hw_io.CCS_gpio, CNT_GPIO, &s_gpio.cnt_ccs, &s_gpio.last_ccs);
    DW.ctrl_hw_io.CCS_o = key_fsm(b, &s_gpio.ccs, 1U, 1U);

    /* 防盗 / 钥匙ACC: 仅去抖 */
    DW.ctrl_hw_io.burglar_o =
        key_debounce(DW.ctrl_hw_io.burglar_gpio, CNT_GPIO, &s_gpio.cnt_burglar, &s_gpio.last_burglar);
    DW.ctrl_hw_io.ACC_o =
        key_debounce(DW.ctrl_hw_io.ACC_gpio, CNT_GPIO, &s_gpio.cnt_acc, &s_gpio.last_acc);
}

void SignalProc_AnalogConvert(void)
{
    if (DW.adc_slmple.zero_flag == 0U) {
        /* ******************** 零位校准: 累加 ADC_ZERO_TARGET 次取平均作为零偏 ******************** */
        if (s_an.adc_cnt >= (uint16_t)ADC_ZERO_TARGET) {
            DW.adc_slmple.bus_Curr_offset    = (uint16_t)(s_an.sum_bus / (uint32_t)ADC_ZERO_TARGET);
            DW.adc_slmple.phaseU_Curr_offset = (uint16_t)(s_an.sum_u   / (uint32_t)ADC_ZERO_TARGET);
            DW.adc_slmple.phaseV_Curr_offset = (uint16_t)(s_an.sum_v   / (uint32_t)ADC_ZERO_TARGET);
					  DW.adc_slmple.phaseW_Curr_offset =  (uint16_t)(s_an.sum_w   / (uint32_t)ADC_ZERO_TARGET);
            s_an.sum_bus = 0U; s_an.sum_u = 0U; s_an.sum_v = 0U;s_an.sum_w=0U;
            s_an.adc_cnt = 0U;
            DW.adc_slmple.zero_flag = 1U;
        } else {
            s_an.sum_bus += DW.adc_slmple.bus_Curr;
            s_an.sum_u   += DW.adc_slmple.phaseU_Curr;
            s_an.sum_v   += DW.adc_slmple.phaseV_Curr;
						s_an.sum_w   += DW.adc_slmple.phaseW_Curr;
            s_an.adc_cnt = (uint16_t)(s_an.adc_cnt + 1U);
        }
    } else {
        /* ******************** 运行: (raw - offset)*系数, 一阶 LPF ******************** */
        float ib = (float)(int16_t)(DW.adc_slmple.bus_Curr    - DW.adc_slmple.bus_Curr_offset)    * CONVERT_BUS_CURR_CONST;
        float iu = (float)(int16_t)(DW.adc_slmple.phaseU_Curr - DW.adc_slmple.phaseU_Curr_offset) * CONVERT_PHASE_CURR_CONST;
        float iv = (float)(int16_t)(DW.adc_slmple.phaseV_Curr - DW.adc_slmple.phaseV_Curr_offset) * CONVERT_PHASE_CURR_CONST;
				float iw = (float)(int16_t)(DW.adc_slmple.phaseW_Curr - DW.adc_slmple.phaseW_Curr_offset) * CONVERT_PHASE_CURR_CONST;
        s_an.f_bus_curr = lpf(s_an.f_bus_curr, ib, 0.35f);
        s_an.f_u_curr   = lpf(s_an.f_u_curr,   iu, 0.85f);
        s_an.f_v_curr   = lpf(s_an.f_v_curr,   iv, 0.85f);
			  s_an.f_w_curr   = lpf(s_an.f_w_curr,   iw, 0.85f);
        DW.motor_anal.bus_Curr    = s_an.f_bus_curr;
        DW.motor_anal.phaseU_Curr = s_an.f_u_curr;
        DW.motor_anal.phaseV_Curr = -(s_an.f_u_curr+s_an.f_w_curr);
				DW.motor_anal.phaseW_Curr = s_an.f_w_curr;
    }

    /* ---- 母线电压 / 油门 一阶 LPF (始终运行)。
           温度(电机/控制器)改由主循环 Temp_Sensor_Update() 采集+解析, 不在此10kHz中断内做。 ---- */
    s_an.f_bus_volt = lpf(s_an.f_bus_volt, (CONVERT_BUS_VOLT_CONST  * (float)DW.adc_slmple.bus_Volt), 0.35f);
    s_an.f_acc      = lpf(s_an.f_acc,      CONVERT_HANDLEBAR_CONST * (float)DW.adc_slmple.handlebar_Volt, 0.35f);
		if(s_an.f_bus_volt<0)
			DW.motor_anal.bus_Volt=0;
		else
			DW.motor_anal.bus_Volt = s_an.f_bus_volt;
		/* 回退 20ms 慢滤: 数据15 外环相位滞后致母线 301A; 暂与 bus_Volt 同快,
		   保留独立字段, 以后只按 2~5ms 单项验证 */
		s_an.f_bus_volt_slow = s_an.f_bus_volt;
		//(Vdc*(1/sqrt(3)))^2
		DW.motor_param.Vdc_svpwm_buf=s_an.f_bus_volt_slow*MATH_1_SQRT3;
		DW.motor_param.Vdc_square=DW.motor_param.Vdc_svpwm_buf*DW.motor_param.Vdc_svpwm_buf;
    DW.motor_anal.handlebar_Volt = s_an.f_acc;
}
/* ******************** PWM 零位校准 (上电快速恢复) ******************** */

/* 启动/重新触发 PWM 零位校准: 任意状态下复位到等待条件态 */
void SignalProc_PwmZeroCalib_Start(void)
{
    s_zero_calib_state = 0U;
    s_zero_calib_timer = 0U;
    s_zero_calib_retry = 0U;
}

/* PWM 零位校准任务 (1ms 周期调用): 状态 0=等待 1=采集 2=完成 3=放弃;
   守卫 pwm_ref_valid + 输出已关断 + |转速|<30RPM, 5s 不满足放弃; 另处理零位基准写入请求 */
void SignalProc_PwmZeroCalib(void)
{
    /* 零位基准写入(仅停机时执行): 写后回读校验, 成功才清请求, 失败每 100ms 重试 */
    /* 标定固化请求(唯一调用点): 保存到 Flash / 恢复出厂, 与零位基准落盘同线程 */
    if (g_calib_commit_req != 0U) {
        if (DW.comm_contrl.control_enable == 0U) {   /* 仅停机时写 Flash */
            uint8_t rc_ = ParamStore_OnCommitCmd(g_calib_commit_req);
            if (rc_ == 0U) { g_calib_commit_req = 0U; }   /* 成功才清请求, 失败下次再试 */
            g_calib_state = (rc_ == 0U) ? 0x03U : 0x04U;   /* 保存完成/失败(失败仍重试) */
        }
    }

    /* MAP 在线重算(0x20/39 cmd=0x20): 以当前 DW 电机参数重算 MTPA Id/Iq 镜像;
       电流圆/电压取值与运行时同口径, 逐扭矩行分步执行, 电机中途使能则中止 */
    if (g_map_regen_req != 0U) {
        if (DW.comm_contrl.control_enable == 0U) {
            real32_T imax = FOC_CurrentCircle();
            real32_T udc  = (real32_T)DW.motor_param.rated_volt;   /* 重算用电机配置额定值, 不用实时母线 */
            if (udc < 20.0F) { udc = (real32_T)DW.sys_param.rated_volt; }
            if (udc < 20.0F) { udc = 72.0F; }
            if (MapStore_RegenStart(DW.motor_param.flux_value,
                                    DW.motor_param.Ld_value,
                                    DW.motor_param.Lq_value,
                                    DW.motor_param.Rs_value,
                                    (real32_T)DW.motor_param.PP_num,
                                    udc, imax) == 0U) {
                g_calib_state = 0xFFU;   /* 电机参数非法, 未开始计算 */
            }
            g_map_regen_req = 0U;   /* 一次性触发, 结果经 0x20/45 读回 */
        }
    }
    if (MapStore_RegenBusy() != 0U) {
        if (DW.comm_contrl.control_enable == 0U) {
            MapStore_RegenTask();
            if (MapStore_RegenBusy() == 0U) {   /* 本拍收尾: 成功回原状态, 否则错误 */
                g_calib_state = (g_map_regen_st == MAPREGEN_ST_OK)
                                ? g_calib_state_prev : 0xFFU;
            }
        } else {
            MapStore_RegenAbort();
            g_calib_state = 0xFFU;   /* 计算被运行中止 */
        }
    }

    if (g_zero_ref_save_pending != 0U) {
        if (DW.comm_contrl.control_enable == 0U) {
            if (++s_zeroref_save_tick >= 100U) {
                s_zeroref_save_tick = 0U;
                if ((ParamStore_ZeroRef_Save() == 0U) &&
                    (ParamStore_ZeroRef_Verify() == 0U)) {
                    g_zero_ref_save_pending = 0U;   /* 确认写进去了才清请求 */
                }
            }
        }
    } else {
        s_zeroref_save_tick = 0U;
    }

    /* 手动零位基准 (CAN 下发): 无需先自动找零, 输出未使能 + 转子静止 时,
       用当前 PWM raw 反算 zero_offset 并刷新句柄; 成功后挂到落盘请求上。 */
    if (DW.comm_contrl.zero_ref_manual_set != 0U) {
        if (((DW.comm_contrl.control_enable == 0U) || (DW.comm_contrl.control_mode == 0U)) &&
            (fabsf((float)DW.motor_main_feedback.mech_Spd) < ZERO_CALIB_SPD_GUARD)) {
            DW.comm_contrl.zero_ref_manual_set = 0U;
            if (ENC_ManualZeroRef_Apply(DW.comm_contrl.zero_ref_manual,
                                        (uint16_t)g_enc.raw) != 0U) {   /* 当前 PWM raw (ISR 写入, 0~4095) */
                g_zero_ref_save_pending = 1U;   /* 变更过 -> 固化(等输出关断后写) */
            }
        }
    }

    if (s_zero_calib_state >= 2U) { return; }

    /* 已有有效零位 (IF 校零/上次恢复成功) 则无需恢复 */
    if (g_enc_foc.zero_state != 0U) {
        s_zero_calib_state = 2U;
        return;
    }

    if (s_zero_calib_state == 0U) {
        /* 等待条件: 基准有效 + 输出关断 + 转子静止 */
        /* 条件: 基准可用 + **输出确实没在驱动**(看 mode 是否被使能, 而非只看命令位 ——
           基准无效时互锁会把 mode 归零, 只判 control_enable 会永远等不到) + 转子静止 */
        if ((g_enc_foc.pwm_ref_valid != 0U)
            && ((DW.comm_contrl.control_enable == 0U) || (DW.comm_contrl.control_mode == 0U))
            && (fabsf((float)DW.motor_main_feedback.mech_Spd) < ZERO_CALIB_SPD_GUARD)) {
            EncPwmSamp_Start();
            s_zero_calib_state = 1U;
            s_zero_calib_timer = 0U;
        } else if (++s_zero_calib_timer >= ZERO_CALIB_WAIT_MAX_MS) {
            s_zero_calib_state = 3U;   /* 放弃: 保持未校零 */
        }
    } else {   /* 1=采集中 */
        EncPwmSampResult_t res;

        /* 采集中条件破坏 (使能输出/转子转动) -> 中止并回等待态 */
        if (((DW.comm_contrl.control_enable != 0U) && (DW.comm_contrl.control_mode != 0U))
            || (fabsf((float)DW.motor_main_feedback.mech_Spd) >= ZERO_CALIB_SPD_GUARD)) {
            EncPwmSamp_Abort();
            s_zero_calib_state = 0U;
            s_zero_calib_timer = 0U;
            return;
        }

        EncPwmSamp_Poll();
        if (EncPwmSamp_Count() >= ZERO_CALIB_SAMP_FRAMES) {
            (void)EncPwmSamp_Stop(&res, ZERO_CALIB_SPREAD_MAX);
            if ((res.ok != 0U) && (ENC_ZeroOffset_FromPwmRaw(res.raw_avg) != 0U)) {
                s_zero_calib_state = 2U;   /* 恢复成功 */
            } else if (++s_zero_calib_retry >= ZERO_CALIB_RETRY_MAX) {
                s_zero_calib_state = 3U;   /* 多次失败, 放弃 */
            } else {
                s_zero_calib_state = 0U;   /* 重试 */
                s_zero_calib_timer = 0U;
            }
        } else if (++s_zero_calib_timer >= ZERO_CALIB_SAMP_MAX_MS) {
            /* 超时未采满 (PWM 信号丢失) -> 按失败重试/放弃 */
            EncPwmSamp_Abort();
            if (++s_zero_calib_retry >= ZERO_CALIB_RETRY_MAX) {
                s_zero_calib_state = 3U;
            } else {
                s_zero_calib_state = 0U;
                s_zero_calib_timer = 0U;
            }
        }
    }
}

/* 查询零位校准是否完成 */
uint8_t SignalProc_IsZeroCalibDone(void)
{
    return (uint8_t)((s_zero_calib_state == 2U) && (g_enc_foc.zero_state == 1U));
}

/* 查询零位校准状态: 0=等待条件 1=采集中 2=完成 3=放弃 */
uint8_t SignalProc_GetZeroCalibState(void)
{
    return s_zero_calib_state;
}
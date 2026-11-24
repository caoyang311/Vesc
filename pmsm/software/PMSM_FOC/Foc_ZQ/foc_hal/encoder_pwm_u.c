/******************************************************************************
 * 文件: encoder_pwm_u.c
 * 功能: UIME 1024线编码器 PWM 输出 + 无锁输入捕获解码 + 去卷绕采样器
 * 创建日期: 2026-06-22
 * 修改日期: 2026-08-02
 * 修改内容: 删除 PosIdf 一阶滤波采集; 新增 EncPwmSamp 去卷绕采样器;
 *           ISR 增加帧周期合法窗校验
 * 修改人: rui.zhang
 ******************************************************************************/

#include "encoder_pwm_u.h"
#include <string.h>

/* ********************* 无锁输入捕获解码 (TIM3 CH2, PA4) ********************* */

volatile EncRaw_t g_enc = {0};

static volatile uint32_t dc_rise1 = 0;
static volatile uint32_t dc_fall  = 0;
static volatile uint32_t dc_rise2 = 0;
static volatile uint8_t  dc_edge  = 0;
static volatile uint16_t dc_raw   = 0;
static volatile uint16_t dc_ax10  = 0;
static volatile uint8_t  s_pwm_enabled = 0u;   /* PWM 解码是否开启 */

/* 解码器初始化: 清零捕获状态, 配置 TIM3 CH2(PA4) 上升沿捕获中断 */
void Encoder_Decode_Init(void)
{
    dc_rise1 = 0;
    dc_fall  = 0;
    dc_rise2 = 0;
    dc_edge  = 0;
    memset((void *)&g_enc, 0, sizeof(g_enc));

    htim3.Instance->PSC  = 17;
    htim3.Init.Prescaler = 17;
    htim3.Instance->EGR  = TIM_EGR_UG;

    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    /* 策略(用户定): PWM 解码只在"自动找零"采样窗口内开启(很占 CPU)。
       此处只做配置, 不使能 —— 由 EncPwmSamp_Start/Stop/Abort 成对开关。 */
    dc_edge   = 0;
    s_pwm_enabled = 0u;
}

/* 开启 PWM 解码 (允许 TIM3 CH2 捕获中断) */
void Encoder_Decode_Enable(void)
{
    if (s_pwm_enabled != 0u) { return; }
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_2,
                                  TIM_INPUTCHANNELPOLARITY_RISING);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC2);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
    /* 重新开始计数: 使"开启后多久没数据"从这一刻算起, 不被关闭期间的旧帧干扰 */
    dc_edge         = 0;
    g_enc.frames    = 0u;
    s_pwm_enabled   = 1u;
}

/* 关闭 PWM 解码 (省 CPU) */
void Encoder_Decode_Disable(void)
{
    if (s_pwm_enabled == 0u) { return; }
    HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_2);
    __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC2);
    s_pwm_enabled = 0u;
}

uint8_t ENC_Pwm_IsEnabled(void)
{
    return s_pwm_enabled;
}

/* 解码中断服务: 三边沿(上升1/下降/上升2)捕获, 解码占空比→绝对位置 */
void Encoder_Decode_ISR(TIM_HandleTypeDef *htim)
{
    uint32_t cap;

    cap = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    g_enc.edges++;   /* 任何边沿都算: 帧会被判废, 边沿不会 —— 判"线是否还活着"用它 */

    switch (dc_edge) {

    case 0: /* 第1上升沿 */
        dc_rise1 = cap;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,
                                      TIM_INPUTCHANNELPOLARITY_FALLING);
        dc_edge = 1;
        break;

    case 1: /* 下降沿 */
        dc_fall = cap;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,
                                      TIM_INPUTCHANNELPOLARITY_RISING);
        dc_edge = 2;
        break;

    case 2: /* 第2上升沿: 完整一帧, 执行解码 */
        dc_rise2 = cap;
        {
            uint32_t period, high_ticks;
            int32_t  data_high;
            uint32_t u_pos;
            uint16_t u_ax10;

            if (dc_rise2 >= dc_rise1)
                period = dc_rise2 - dc_rise1;
            else
                period = (0xFFFFu - dc_rise1) + dc_rise2 + 1u;

            if (dc_fall >= dc_rise1)
                high_ticks = dc_fall - dc_rise1;
            else
                high_ticks = (0xFFFFu - dc_rise1) + dc_fall + 1u;

            data_high = (high_ticks >= ENC_HEADER_CLOCKS)
                      ? (int32_t)(high_ticks - ENC_HEADER_CLOCKS)
                      : 0;

            /* 帧周期合法窗校验: 窗外帧为毛刺/丢边沿, 丢弃不锁存 */
            if ((period >= ENC_PERIOD_MIN) && (period <= ENC_PERIOD_MAX) && (data_high >= 0))
            {
                u_pos  = ((uint32_t)data_high * ENC_POS_MAX + (ENC_DATA_CLOCKS / 2u))
                        / ENC_DATA_CLOCKS;
                if (u_pos > ENC_POS_MAX) u_pos = ENC_POS_MAX;

                u_ax10 = (uint16_t)((u_pos * 3600u + (ENC_POS_MAX / 2u)) / ENC_POS_MAX);

                /* 最新值无条件更新: 故障检测随时要读最新角度, 不能等主循环 Fetch */
                g_enc.angle_x10 = u_ax10;
                g_enc.raw       = (uint16_t)data_high;
                g_enc.ready     = 1;
                g_enc.frames++;            /* 合法帧累计 */
            }
        }

        dc_rise1 = dc_rise2;
        dc_edge  = 1;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,
                                      TIM_INPUTCHANNELPOLARITY_FALLING);
        break;

    default:
        dc_edge = 0;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,
                                      TIM_INPUTCHANNELPOLARITY_RISING);
        break;
    }
}

/* ********************* 主循环读取接口 (无锁) ********************* */

/* 无锁读取一帧解码结果: 有新帧则拷贝到 out 并清 ready, 返回 1; 无新帧返回 0 */
uint8_t Encoder_Decode_Fetch(EncRaw_t *out)
{
    if (g_enc.ready == 0) return 0;
    if (out) *out = g_enc;
    g_enc.ready = 0;
    return 1;
}


/* ********************* PWM raw 去卷绕采样器 *********************
 * 用途: 转子静止/准静止时采 PWM raw 并求均值 (双向校零记录点 / 上电快速恢复)。
 * 去卷绕: 首帧 raw 为参考, 后续帧差分展开到 ±2048, 消除 0/4095 跳变对平均的污染;
 * 展宽 spread = 展开值 max-min, 反映采样期间抖动/微动, 供调用方判质量。 */

 volatile uint8_t  s_samp_active = 0u;
 uint16_t s_samp_frames = 0u;   /* 已采帧数 */
 uint16_t s_samp_ref    = 0u;   /* 首帧 raw (卷绕参考) */
 int32_t  s_samp_unwrap = 0;    /* 当前展开值 (相对首帧) */
 int32_t  s_samp_sum    = 0;    /* 展开值累加 */
 int32_t  s_samp_min    = 0;    /* 展开值最小 */
 int32_t  s_samp_max    = 0;    /* 展开值最大 */

/* 启动采样: 清零状态并激活 */
void EncPwmSamp_Start(void)
{
		EncRaw_t dump;
    Encoder_Decode_Enable();            /* 找零开始: 打开 PWM 解码 */
    (void)Encoder_Decode_Fetch(&dump);  /* 冲净上一停点残留挂起帧, 防污染卷绕参考 */
	
    s_samp_frames = 0u;
    s_samp_ref    = 0u;
    s_samp_unwrap = 0;
    s_samp_sum    = 0;
    s_samp_min    = 0;
    s_samp_max    = 0;
    s_samp_active = 1u;
}

/* 采样喂入: 取新帧并做去卷绕累加, 无新帧直接返回; 采满 ENC_SAMP_CAP 自动停 */
void EncPwmSamp_Poll(void)
{
    EncRaw_t raw;
    int32_t  delta;

    if (s_samp_active == 0u) { return; }
    if (Encoder_Decode_Fetch(&raw) == 0u) { return; }

    if (s_samp_frames == 0u) {
        s_samp_ref    = raw.raw;
        s_samp_unwrap = 0;
        s_samp_sum    = 0;
        s_samp_min    = 0;
        s_samp_max    = 0;
    } else {
				int32_t prev = (int32_t)((uint32_t)(s_samp_ref + s_samp_unwrap) & 0x0FFFu);
				delta = (int32_t)raw.raw - prev;
				if (delta > 2048)  { delta -= 4096; }
				if (delta < -2048) { delta += 4096; }
        s_samp_unwrap += delta;
        s_samp_sum    += s_samp_unwrap;
        if (s_samp_unwrap < s_samp_min) { s_samp_min = s_samp_unwrap; }
        if (s_samp_unwrap > s_samp_max) { s_samp_max = s_samp_unwrap; }
    }
    s_samp_frames++;

    if (s_samp_frames >= ENC_SAMP_CAP) {
        s_samp_active = 0u;   /* 采满自动停, 结果留给 Stop 结算 */
    }
}

/* 已采帧数 */
uint16_t EncPwmSamp_Count(void)
{
    return s_samp_frames;
}

/* 是否激活 */
uint8_t EncPwmSamp_IsActive(void)
{
    return s_samp_active;
}

/* 中止采样 (不出结果) */
void EncPwmSamp_Abort(void)
{
    s_samp_active = 0u;
    s_samp_frames = 0u;
    Encoder_Decode_Disable();           /* 找零中断/放弃: 关闭 PWM 解码 */
}

/* 结束采样并出结果: 帧数>=ENC_SAMP_MIN 且 spread<=max_spread 时 ok=1 */
uint8_t EncPwmSamp_Stop(EncPwmSampResult_t *res, uint16_t max_spread)
{
    int32_t mean;
    int32_t avg;
    int32_t spread;

    s_samp_active = 0u;
    Encoder_Decode_Disable();           /* 找零结束: 关闭 PWM 解码 */

    if (res == 0) { return 0u; }

    res->frames = s_samp_frames;
    if (s_samp_frames < ENC_SAMP_MIN) {
        res->ok      = 0u;
        res->raw_avg = 0u;
        res->spread  = 0u;
        s_samp_frames = 0u;
        return 0u;
    }

    /* 展开均值 (四舍五入, 支持负值) */
    if (s_samp_sum >= 0) {
        mean = (s_samp_sum + (int32_t)(s_samp_frames / 2u)) / (int32_t)s_samp_frames;
    } else {
        mean = (s_samp_sum - (int32_t)(s_samp_frames / 2u)) / (int32_t)s_samp_frames;
    }

    /* 折回 0..4095 */
    avg = (int32_t)s_samp_ref + mean;
    avg %= 4096;
    if (avg < 0) { avg += 4096; }

    spread = s_samp_max - s_samp_min;

    res->raw_avg = (uint16_t)avg;
    res->spread  = (uint16_t)((spread > 4095) ? 4095 : spread);
    res->ok      = (uint8_t)((spread <= (int32_t)max_spread) ? 1u : 0u);

    s_samp_frames = 0u;
    return res->ok;
}

/* ********************* HAL 回调 ********************* */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        Encoder_Decode_ISR(htim);
    }
}

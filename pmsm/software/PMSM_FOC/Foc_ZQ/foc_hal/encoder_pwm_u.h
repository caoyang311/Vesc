/******************************************************************************
 * 文件: encoder_pwm_u.h
 * 功能: UIME 1024线磁编码器 PWM 输出 + 输入捕获解码 + 去卷绕采样器 接口
 * 创建日期: 2026-06-22
 * 修改日期: 2026-10-09
 * 修改内容: 删除未使用的 TIM4_ENC_PSC/ARR 宏, 修正捕获时基备注
 *           (时基在 TIM3: PSC=17 即 4MHz tick; TIM4 另作 ADC1 温度采集 1kHz 触发)
 * 修改人: rui.zhang
 ******************************************************************************/

/* 协议: 4MHz(250ns/tick), 帧 = 16H帧头 + 4095数据 + 8L帧尾 = 4119 clocks, 帧率≈971Hz
 * 硬件: PB9(TIM4_CH4)=PWM输出(保留), PA4(TIM3_CH2)=输入捕获解码 */

#ifndef __HW_ENCODER_PWM_U_H__
#define __HW_ENCODER_PWM_U_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "tim.h"

/* ********************* 协议参数 ********************* */

#define ENC_CLK_HZ           4000000u   /* 4 MHz 基频 (250 ns/tick)         */
#define ENC_HEADER_CLOCKS    16u        /* 帧头高电平 (4 µs)                 */
#define ENC_TAIL_CLOCKS      8u         /* 帧尾低电平 (2 µs)                 */
#define ENC_DATA_CLOCKS      4095u      /* 数据段 clocks (0=0°, 4095≈360°)  */
#define ENC_FRAME_CLOCKS     (ENC_HEADER_CLOCKS + ENC_DATA_CLOCKS + ENC_TAIL_CLOCKS) /* 4119 */
#define ENC_MIN_CCR          ENC_HEADER_CLOCKS                         /* 16   */
#define ENC_MAX_CCR          (ENC_FRAME_CLOCKS - ENC_TAIL_CLOCKS)      /* 4111 */
#define ENC_RESOLUTION       1024u      /* 位置分辨率 0..1023               */
#define ENC_POS_MAX          (ENC_RESOLUTION - 1u)                     /* 1023 */
#define ENC_DATA_RANGE       (ENC_MAX_CCR - ENC_MIN_CCR)               /* 4095 */

/* 帧周期合法窗: 4119±150 clocks (约 ±3.6%), 窗外帧视为毛刺/丢边沿, 丢弃不锁存 */
#define ENC_PERIOD_MIN       (ENC_FRAME_CLOCKS - 150u)                 /* 3969 */
#define ENC_PERIOD_MAX       (ENC_FRAME_CLOCKS + 150u)                 /* 4269 */

/* 捕获时基: TIM3 PSC=17 -> 72MHz/(17+1)=4MHz tick, 由 Encoder_Decode_Init 配置;
   TIM4 与解码无关 (工程中 TIM4 为 ADC1 温度采集的 1kHz 触发源) */

/* ********************* 解码结果 ********************* */

/* 原始解码数据 (ISR 填入, 主循环读取, 无锁) */
typedef struct {
    uint16_t angle_x10;       /* 解码角度 ×10 (如 1800 = 180.0°)       */
    uint16_t raw;             /* 原始值 (0~4095)                        */
    uint8_t  ready;           /* ISR置1=新帧已锁存, 主循环读后清零       */
    uint32_t frames;          /* 合法帧累计 (ISR 每成功解码一帧 +1)      */
    uint32_t edges;           /* 边沿累计 (任何一次捕获都算): 与 frames 一起区分"线断了"和"帧被判废" */
} EncRaw_t;

extern volatile EncRaw_t g_enc;   /* 全局解码结果 (ISR写入, 主循环读取)  */

/* ********************* 输入捕获解码 API (TIM3 CH2, PA4) ********************* */
void     Encoder_Decode_Init(void);
/* PWM 解码开关: 解码中断 ~971Hz + 三边沿解码很占 CPU, 只允许在需要的时段开启
   (上电建绝对基准/零点标定窗口)。判据必须先看 ENC_Pwm_IsEnabled, 只判"开着却没数据"。 */
void     Encoder_Decode_Enable(void);
void     Encoder_Decode_Disable(void);
uint8_t  ENC_Pwm_IsEnabled(void);
uint8_t  Encoder_Decode_Fetch(EncRaw_t *out);

/* ********************* PWM raw 去卷绕采样器 (静止/准静止校零用) ********************* */

/* 采样约束: 上限 512 帧 (采满自动停), 最少 8 帧才允许出结果 */
#define ENC_SAMP_CAP         512u
#define ENC_SAMP_MIN         8u

/* 采样结果: ok=1 时 raw_avg/spread/frames 有效 */
typedef struct {
    uint8_t  ok;        /* 1=采样有效 (帧数够且展宽达标) */
    uint16_t raw_avg;   /* 去卷绕平均 raw (0..4095) */
    uint16_t spread;    /* 去卷绕展宽 max-min (clocks) */
    uint16_t frames;    /* 实际采样帧数 */
} EncPwmSampResult_t;

/* 启动采样: 清零状态并激活 (首帧作为卷绕参考) */
void     EncPwmSamp_Start(void);

/* 采样喂入: 任务周期调用 (10kHz/1ms 均可), 内部无锁取新帧, 无新帧直接返回 */
void     EncPwmSamp_Poll(void);

/* 已采帧数 */
uint16_t EncPwmSamp_Count(void);

/* 是否激活 (Start 后 / Stop·Abort 前为 1) */
uint8_t  EncPwmSamp_IsActive(void);

/* 中止采样 (不出结果) */
void     EncPwmSamp_Abort(void);

/* 结束采样并出结果: 帧数>=ENC_SAMP_MIN 且 spread<=max_spread 时 ok=1。
   返回值 = res->ok (1=有效, 0=无效) */
uint8_t  EncPwmSamp_Stop(EncPwmSampResult_t *res, uint16_t max_spread);

/* HAL 回调 (内部使用) */
void Encoder_Decode_ISR(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* __HW_ENCODER_PWM_U_H__ */

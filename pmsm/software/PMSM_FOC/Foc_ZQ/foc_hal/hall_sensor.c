/*
 * 文件: hall_sensor.c
 * 功能: 霍尔传感器接口(三线 UVW 复用编码器 A/B/Z 引脚), 电角度插值与转速解算
 * 创建日期: 2026-10-09
 * 修改人: rui.zhang
 *
 * 移植自 BL_YD_HMotorCtrl 的 Hall_SensorDev (状态转移表保留其角度映射关系),
 * 针对"高速速度波动大"做了三处改造:
 *   1) 边沿捕获从轮询改为 EXTI 双边沿 + DWT 周期计数器时间戳(硬件级精度);
 *   2) 电角度在 60 度扇区内按上一边沿周期插值, 消除边沿间角度滞留
 *      (滞留最大 30 度电角, 是高速转矩纹波与速度波动的来源);
 *   3) 转速用边沿周期法(60 度/T)加一阶滤波, 低速无超时误报(超时钳零)。
 * 引脚复用与切换由 motor_param.sensor_type 决定(0=ABZ, 1=霍尔),
 * CAN 修改该参数后下一拍自动完成接口重配, 无需复位。
 */
#include "hall_sensor.h"
#include "foc_data.h"          /* DW.motor_param / motor_main_feedback */
#include "encoder_foc_sdk.h"   /* g_enc_foc(速度出口) / ENC_Init(切回 ABZ) */
#include "tim.h"               /* htim2 */
#include "main.h"

#define HALL_PORT        GPIOA
#define HALL_PINS        (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2)  /* U=PA0 V=PA1 W=PA2 */
#define HALL_STATE_MSK   0x07U

/* 边沿周期下限(周期计数): 低于它视为毛刺不采纳 —— 20000rpm 时边沿率 8kHz,
   64MHz 计数下周期 8000; 取 2000 已远超正常范围 */
#define HALL_MIN_PERIOD_CYC  2000U
/* 速度解算超时: 超过该周期计数无新边沿视为静止 */
#define HALL_SPD_TIMEOUT_CYC (SystemCoreClock / 10U)   /* 100ms(按实际主频折算; 原 100*64e6 会溢出 32 位) */

/* 状态 -> 扇区基角(12bit 电角度)。六个有效状态各占 60 度扇区, 边沿落在扇区边界:
   4096/6 = 682.67, 取整为 683/1365/2048/2731/3413 */
static const uint16_t s_state_angle[8] = {
    0U,                                   /* 0: 无效 */
    683U,   2048U,  1365U,                /* 1, 2, 3 */
    3413U,  0U,     2731U,                /* 4, 5, 6 */
    0U                                    /* 7: 无效 */
};
/* 正转(顺磁场方向)的下一状态环: 5->1->3->2->6->4->5, 与 BL_YD 转移表一致 */
static const uint8_t s_cw_next[8]  = { 0U, 3U, 6U, 2U, 5U, 1U, 4U, 0U };
static const uint8_t s_ccw_next[8] = { 0U, 5U, 3U, 1U, 6U, 4U, 2U, 0U };

Hall_Ctrl g_hall;

/* 边沿捕获状态(EXTI 上下文写, 任务上下文读; 32 位对齐, 原子) */
static volatile uint32_t s_t_last;         /* 最近边沿时刻(周期计数) */
static volatile uint32_t s_period;         /* 最近边沿间隔(周期计数) */
static volatile uint16_t s_angle_base;     /* 最近边沿处的扇区基角 */
static uint8_t s_hw_mode = 0U;             /* 当前硬件接口: 0=ABZ 1=霍尔 */
static float s_spd_filt = 0.0F;            /* 转速一阶滤波状态 */

/* 霍尔相位偏移(12bit 电角度): 运行期参数 A7/03 idx0, 由上位机标定后写入; 越界按 0 处理 */
static int16_t Hall_PhaseShift(void)
{
    int32_T v = DW.motor_param.hall_phase_shift;
    if ((v < -2048) || (v > 2047)) { v = 0; }
    return (int16_t)v;
}

extern uint32_t SystemCoreClock;           /* 周期计数 -> 时间换算 */

/* ******************** 硬件接口切换 ******************** */

static void hall_exti_cfg(uint32_t pin, uint32_t line, uint8_t enable)
{
    if (enable != 0U) {
        GPIO_InitTypeDef g = {0};
        g.Pin  = pin;
        g.Mode = GPIO_MODE_IT_RISING_FALLING;
        g.Pull = GPIO_PULLUP;              /* 与 ABZ 口径一致: 断线落在确定电平 */
        HAL_GPIO_Init(HALL_PORT, &g);
        EXTI->IMR |=  (1UL << line);
        EXTI->RTSR |= (1UL << line);
        EXTI->FTSR |= (1UL << line);
        EXTI->PR   =  (1UL << line);       /* 清历史挂起 */
        HAL_NVIC_SetPriority((line == 0U) ? EXTI0_IRQn :
                             (line == 1U) ? EXTI1_IRQn : EXTI2_TSC_IRQn, 5U, 0U);
        HAL_NVIC_EnableIRQ((line == 0U) ? EXTI0_IRQn :
                           (line == 1U) ? EXTI1_IRQn : EXTI2_TSC_IRQn);
    } else {
        HAL_NVIC_DisableIRQ((line == 0U) ? EXTI0_IRQn :
                            (line == 1U) ? EXTI1_IRQn : EXTI2_TSC_IRQn);
        EXTI->IMR  &= ~(1UL << line);
        EXTI->RTSR &= ~(1UL << line);
        EXTI->FTSR &= ~(1UL << line);
        EXTI->PR    =  (1UL << line);
    }
}

void Hall_IF_Activate(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    /* SYSCFG EXTI 线默认映射 PA 口(复位态), 显式确认一次 */
    SYSCFG->EXTICR[0] &= ~((uint32_t)0x0FFFUL);   /* line0/1/2 -> PA0/PA1/PA2 */

    HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
    hall_exti_cfg(GPIO_PIN_0, 0U, 1U);
    hall_exti_cfg(GPIO_PIN_1, 1U, 1U);
    hall_exti_cfg(GPIO_PIN_2, 2U, 1U);

    /* 初始状态预置: 读当前组合定扇区基角, 方向未知, 角度从扇区基角出发 */
    {
        uint8_t st = (uint8_t)(HALL_PORT->IDR & HALL_STATE_MSK);
        if ((st >= 1U) && (st <= 6U)) {
            g_hall.state      = st;
            g_hall.direction  = 0;
            s_angle_base      = s_state_angle[st];
            g_hall.elec_angle = (uint16_t)((s_angle_base + (uint16_t)Hall_PhaseShift())
                                           & (HALL_ANGLE_MAX - 1U));
        } else {
            g_hall.state = 0U;
            g_hall.err_invalid++;
        }
        s_t_last = DWT->CYCCNT;
        s_period = 0U;
        g_hall.invalid_ms = 0U;   /* 接口切换后重新计时 */
    }
    s_hw_mode = 1U;
}

void Hall_IF_Deactivate(void)
{
    hall_exti_cfg(GPIO_PIN_0, 0U, 0U);
    hall_exti_cfg(GPIO_PIN_1, 1U, 0U);
    hall_exti_cfg(GPIO_PIN_2, 2U, 0U);
    HAL_TIM_Encoder_DeInit(&htim2);        /* 释放 EXTI 占用的引脚与句柄 */
    MX_TIM2_Init();                        /* 重配 GPIO AF + 编码器模式定时器 */
    ENC_Init();                            /* 启动计数 + 重建软件句柄 */
    s_hw_mode = 0U;
}

void Hall_IF_Poll(uint8_t sensor_type_now)
{
    uint8_t want = (sensor_type_now == SENSOR_TYPE_HALL) ? 1U : 0U;
    if ((want != 0U) && (s_hw_mode == 0U)) {
        Hall_IF_Activate();
    } else if ((want == 0U) && (s_hw_mode != 0U)) {
        Hall_IF_Deactivate();
    }
}

uint8_t Hall_IF_IsActive(void)
{
    return s_hw_mode;
}

/* ******************** 边沿处理(EXTI 上下文, 尽量短) ******************** */

void Hall_EXTI_Handle(uint32_t pin)
{
    uint32_t now = DWT->CYCCNT;
    uint8_t  st  = (uint8_t)(HALL_PORT->IDR & HALL_STATE_MSK);
    uint32_t period;
    uint8_t  prev;

    (void)pin;
    if ((st == 0U) || (st == 7U)) {        /* 全低/全高: 传感器缺线或未接 */
        g_hall.err_invalid++;
        return;
    }
    prev = g_hall.state;
    if (st == prev) {                      /* 同状态边沿: 毛刺, 丢弃 */
        return;
    }
    period = now - s_t_last;
    if (period < HALL_MIN_PERIOD_CYC) {    /* 周期下限以下视为毛刺 */
        return;
    }
    if ((prev >= 1U) && (prev <= 6U)) {
        if (s_cw_next[prev] == st) {
            g_hall.direction = 1;
        } else if (s_ccw_next[prev] == st) {
            g_hall.direction = -1;
        } else {                           /* 跳变两个状态: 丢边沿或干扰 */
            g_hall.err_transition++;
        }
    }
    g_hall.state = st;
    s_angle_base = s_state_angle[st];
    s_t_last     = now;
    s_period     = period;
}

/* ******************** 10kHz: 角度插值 ******************** */

void Hall_Angle_Task(void)
{
    uint32_t dt;
    uint32_t per = s_period;
    uint16_t ang = s_angle_base;

    if ((per > HALL_MIN_PERIOD_CYC) &&
        (g_hall.state >= 1U) && (g_hall.state <= 6U)) {
        dt = DWT->CYCCNT - s_t_last;
        if (dt > (per + (per >> 2))) {     /* 超出上边沿周期 25%: 速度估计过期, 保持在基角 */
            dt = 0U;
        }
        if (dt != 0U) {                    /* 60 度(682.67 计数)在 dt 内的推进量 */
            uint32_t adv = ((uint32_t)683U * dt + (per >> 1)) / per;
            if (adv > 683U) { adv = 683U; }
            ang = (uint16_t)(ang + (uint16_t)adv);
        }
    }
    g_hall.elec_angle = (uint16_t)((ang + (uint16_t)Hall_PhaseShift())
                                   & (HALL_ANGLE_MAX - 1U));
    DW.motor_main_feedback.elec_angle = g_hall.elec_angle;
}

/* ******************** 模块状态 (整数化, uint16 计数) ******************** */

void Hall_Speed_Task(void)
{
    float spd;

    if ((s_period > HALL_MIN_PERIOD_CYC) &&
        ((DWT->CYCCNT - s_t_last) < HALL_SPD_TIMEOUT_CYC)) {
        /* 机械转速 = 电角速度/极对数: 每边沿 60 度电角,
           rpm = 周期频率(Hz) * 60 / (6 * PP) */
        spd = ((float)SystemCoreClock / (float)s_period) * 60.0F
              / (6.0F * (float)DW.motor_param.PP_num);
        if (g_hall.direction < 0) { spd = -spd; }
    } else {
        spd = 0.0F;                        /* 静止或超时 */
    }
    s_spd_filt += 0.3F * (spd - s_spd_filt);
    g_hall.speed_rpm = (int32_t)(s_spd_filt + ((s_spd_filt >= 0.0F) ? 0.5F : -0.5F));
    g_enc_foc.hAvrSpeedRPM = s_spd_filt;   /* 与 ABZ 同出口, 供 mech_Spd/故障层消费 */

    /* 无效组合(0/7)持续时间: 霍尔线断/短路时给出可观测证据, 上层按 HALL_INVALID_FAULT_MS 判故障 */
    if ((g_hall.state == 0U) || (g_hall.state == 7U)) {
        if (g_hall.invalid_ms < 0xFFFFU) { g_hall.invalid_ms++; }
    } else {
        g_hall.invalid_ms = 0U;
    }
}

/* ******************** EXTI 中断入口(直连寄存器, 不走 HAL 回调链) ******************** */

void EXTI0_IRQHandler(void)
{
    if ((EXTI->PR & EXTI_PR_PR0) != 0U) {
        EXTI->PR = EXTI_PR_PR0;
        Hall_EXTI_Handle(GPIO_PIN_0);
    }
}

void EXTI1_IRQHandler(void)
{
    if ((EXTI->PR & EXTI_PR_PR1) != 0U) {
        EXTI->PR = EXTI_PR_PR1;
        Hall_EXTI_Handle(GPIO_PIN_1);
    }
}

void EXTI2_TSC_IRQHandler(void)
{
    if ((EXTI->PR & EXTI_PR_PR2) != 0U) {
        EXTI->PR = EXTI_PR_PR2;
        Hall_EXTI_Handle(GPIO_PIN_2);
    }
}

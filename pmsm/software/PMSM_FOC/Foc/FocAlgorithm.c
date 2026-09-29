/**
 * @file FocAlgorithm.c
 * @brief FOC算法模块实现文件。
 *
 * 本模块用于实现Clarke变换、Park变换、反Park变换及SVPWM算法。
 */

#include "FocAlgorithm.h"

#include <math.h>
#include <stddef.h>

#define FOC_ALGORITHM_TWO_THIRDS          (0.6666666667F) /* 2/3 */
#define FOC_ALGORITHM_ONE_HALF            (0.5F)          /* 1/2 */
#define FOC_ALGORITHM_ONE_OVER_SQRT_THREE (0.5773502692F) /* 1/√3 */
#define FOC_ALGORITHM_SQRT_THREE          (1.7320508076F) /* √3 */
#define FOC_ALGORITHM_HALF_SQRT_THREE     (0.8660254038F) /* √3/2 */
#define FOC_ALGORITHM_ONE_QUARTER         (0.25F)          /* 1/4 */
#define FOC_ALGORITHM_PWM_ARR             (16800.0F-1.0F)        /* 重装载值 */
#define FOC_ALGORITHM_CONTROL_FREQUENCY   (10000.0F)       /* SVPWM执行频率，Hz */
#define FOC_ALGORITHM_PHASE_SCALE         (4294967296.0F)  /* 2^32相位单位 */
#define FOC_ALGORITHM_PHASE_OUTPUT_SHIFT  (16U)

/** @brief 根据输入频率生成内部电角度相位。 */
void FocAlgorithm_GenerateAngle(float frequency_hz,
                                uint16_t * phase);

/** @brief 执行七段式SVPWM并生成三相比较值。 */
void FocAlgorithm_Svpwm(const Foc_Volt_AlphaBetaType * alpha_beta,
                               const float * dc_bus_voltage,
                               Foc_PwmCompareType * compare);

/** @brief 执行反Park变换。 */
void FocAlgorithm_InversePark(const Foc_Volt_DqType * dq,
                                     const Foc_TrigType * trig,
                                     Foc_Volt_AlphaBetaType * alpha_beta);
/** @brief 执行Clarke变换。 */
void FocAlgorithm_Clarke(
    const Foc_PhaseCurrentType * phase_current,
    Foc_Curr_AlphaBetaType * alpha_beta);
 /** @brief 执行Park变换。 */
void FocAlgorithm_Park(const Foc_Curr_AlphaBetaType * alpha_beta,
                              const Foc_TrigType * trig,
                              Foc_Curr_DqType * dq);    


/**
 * @brief 根据输入频率生成电角度相位。
 *
 * 每次调用根据输入电角频率计算当前10kHz控制周期的32位相位增量，
 * 更新函数内部静态累加器并输出高16位相位。累加器溢出时自然回绕。
 *
 * @param[in] frequency_hz 电角度增长频率，单位Hz，有效范围为0至5000Hz。
 * @param[out] phase 完整电周期相位，指针不得为NULL。
 *
 * @post 参数有效时更新内部累加器和输出相位，否则不修改状态及输出。
 * @note 须与SVPWM在同一执行上下文中以10kHz频率调用；函数不可重入。
 */
void FocAlgorithm_GenerateAngle(float frequency_hz,
                                uint16_t * phase)
{
    static uint32_t accumulator = 0U;
    float phase_step_float = 0.0F;
    uint32_t phase_step = 0U;
    uint32_t next_accumulator = 0U;
    uint16_t next_phase = 0U;


    /* phase_step = frequency_hz * 2^32 / control_frequency_hz */
    phase_step_float = (frequency_hz * FOC_ALGORITHM_PHASE_SCALE) /
                        FOC_ALGORITHM_CONTROL_FREQUENCY;
    phase_step = (uint32_t)phase_step_float;

    /* accumulator(k+1) = accumulator(k) + phase_step，溢出自然回绕。 */
    next_accumulator = accumulator + phase_step;

    /* phase_u16 = accumulator[31:16]。 */
    next_phase = (uint16_t)(next_accumulator >>
                            FOC_ALGORITHM_PHASE_OUTPUT_SHIFT);

    accumulator = next_accumulator;
    *phase = next_phase;
}

/**
 * @brief 按七段式空间矢量调制计算三相PWM比较值。
 *
 * 根据Alpha-Beta参考电压判断扇区，计算相邻基本矢量作用时间，按对称
 * 七段式序列分配零矢量时间，并输出TIM1三个通道的比较值。
 *
 * @param[in] alpha_beta Alpha-Beta参考电压，单位为V，指针不得为NULL。
 * @param[in] dc_bus_voltage 实时直流母线电压，单位为V，须大于0，指针不得为NULL。
 * @param[out] compare TIM1三相比较值，范围为0至3599，指针不得为NULL。
 *
 * @post 参数有效时更新完整输出；参数无效时不修改输出。
 * @note 超出线性调制区时，T1和T2按相同比例缩限，使T1+T2等于Ts。
 */
void FocAlgorithm_Svpwm(const Foc_Volt_AlphaBetaType * alpha_beta,
                               const float * dc_bus_voltage,
                               Foc_PwmCompareType * compare)
{
    Foc_PwmCompareType result = {0};
    uint8_t a = 0U;
    uint8_t b = 0U;
    uint8_t c = 0U;
    uint8_t state = 0U;
    uint8_t sector = 1U;
    float u1 = 0.0F;
    float u2 = 0.0F;
    float u3 = 0.0F;
    float voltage_time_scale = 0.0F;
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    float t1 = 0.0F;
    float t2 = 0.0F;
    float time_scale = 1.0F;
    float ta = 0.0F;
    float tb = 0.0F;
    float tc = 0.0F;
    float tcmp_u = 0.0F;
    float tcmp_v = 0.0F;
    float tcmp_w = 0.0F;

    /*
        * 扇区判断中间变量：
        * U1 = Ubeta
        * U2 = (sqrt(3) / 2) * Ualpha - (1 / 2) * Ubeta
        * U3 = -(sqrt(3) / 2) * Ualpha - (1 / 2) * Ubeta
        */
    u1 = alpha_beta->V_beta;
    u2 = (FOC_ALGORITHM_HALF_SQRT_THREE * alpha_beta->V_alpha) -
            (FOC_ALGORITHM_ONE_HALF * alpha_beta->V_beta);
    u3 = (-FOC_ALGORITHM_HALF_SQRT_THREE * alpha_beta->V_alpha) -
            (FOC_ALGORITHM_ONE_HALF * alpha_beta->V_beta);

    if (u1 > 0.0F)
    {
        a = 1U;
    }
    else
    {
        a = 0U;
    }

    if (u2 > 0.0F)
    {
        b = 1U;
    }
    else
    {
        b = 0U;
    }

    if (u3 > 0.0F)
    {
        c = 1U;
    }
    else
    {
        c = 0U;
    }

    /* N = 4*C + 2*B + A。 */
    state = (uint8_t)((4U * c) + (2U * b) + a);

    /* N与扇区映射：3->1，1->2，5->3，4->4，6->5，2->6。 */
    if (state == 3U)
    {
        sector = 1U;
    }
    else if (state == 1U)
    {
        sector = 2U;
    }
    else if (state == 5U)
    {
        sector = 3U;
    }
    else if (state == 4U)
    {
        sector = 4U;
    }
    else if (state == 6U)
    {
        sector = 5U;
    }
    else if (state == 2U)
    {
        sector = 6U;
    }
    else
    {
        /* 零矢量或边界状态保持默认Sector 1。 */
    }

    /*
        * 伏秒平衡公共系数：K = sqrt(3) * Ts / Udc。
        * 本实现使用ARR表示一个PWM周期Ts，因此K的单位为计数/V。
        */
    voltage_time_scale = (FOC_ALGORITHM_SQRT_THREE *
                            FOC_ALGORITHM_PWM_ARR) /
                            *dc_bus_voltage;

    /*
        * 复用扇区判断结果，避免重复计算Alpha-Beta组合项：
        * X =  U1 * K
        * Y = -U3 * K
        * Z = -U2 * K
        */
    x = u1 * voltage_time_scale;
    y = -u3 * voltage_time_scale;
    z = -u2 * voltage_time_scale;

    /*
        * t1表示由Ta减至Tb的矢量时间Tx，t2表示由Tb减至Tc的
        * 矢量时间Ty。第二扇区为匹配PWM1比较值公式，按T2、T6排序：
        *
        * Sector 1: Tx=T4= U2*K=-Z, Ty=T6= U1*K= X
        * Sector 2: Tx=T2=-U2*K= Z, Ty=T6=-U3*K= Y
        * Sector 3: Tx=T2= U1*K= X, Ty=T3= U3*K=-Y
        * Sector 4: Tx=T1=-U1*K=-X, Ty=T3=-U2*K= Z
        * Sector 5: Tx=T1= U3*K=-Y, Ty=T5= U2*K=-Z
        * Sector 6: Tx=T4=-U3*K= Y, Ty=T5=-U1*K=-X
        */
    if (sector == 1U)
    {
        t1 = -z; /* T4 */
        t2 = x;  /* T6 */
    }
    else if (sector == 2U)
    {
        t1 = z;  /* Tx = T2 */
        t2 = y;  /* Ty = T6 */
    }
    else if (sector == 3U)
    {
        t1 = x;  /* T2 */
        t2 = -y; /* T3 */
    }
    else if (sector == 4U)
    {
        t1 = -x; /* T1 */
        t2 = z;  /* T3 */
    }
    else if (sector == 5U)
    {
        t1 = -y; /* T1 */
        t2 = -z; /* T5 */
    }
    else
    {
        t1 = y;  /* T4 */
        t2 = -x; /* T5 */
    }

    if ((t1 >= 0.0F) && (t2 >= 0.0F))
    {
        /*
            * 当T1 + T2 > Ts时按相同比例缩限：
            * k_limit = Ts / (T1 + T2)
            * T1' = T1 * k_limit
            * T2' = T2 * k_limit
            * 缩限后T0 = Ts - T1' - T2' = 0。
            */
        if ((t1 + t2) > FOC_ALGORITHM_PWM_ARR)
        {
            time_scale = FOC_ALGORITHM_PWM_ARR / (t1 + t2);
            t1 *= time_scale;
            t2 *= time_scale;
        }
        else
        {
            /* 线性调制区内保留原始T1和T2。 */
        }

        /*
            * PWM1模式、中心对齐计数下的七段式比较值：
            * Tx = t1，Ty = t2
            * Ta = (Ts + Tx + Ty) / 4
            * Tb = Ta - Tx / 2
            * Tc = Tb - Ty / 2
            *
            * 等价地，单个零矢量总作用时间为：
            * T0 = T7 = (Ts - Tx - Ty) / 2
            * Tc = T0 / 2 = (Ts - Tx - Ty) / 4
            * Tb = Tc + Ty / 2
            * Ta = Tb + Tx / 2
            * 因PWM1高有效输出在CNT < CCR时有效，所以较大的比较值对应
            * 较长的上桥臂导通时间。
            */
        ta = (FOC_ALGORITHM_PWM_ARR + t1 + t2) *
                FOC_ALGORITHM_ONE_QUARTER;
        tb = ta - (t1 * FOC_ALGORITHM_ONE_HALF);
        tc = tb - (t2 * FOC_ALGORITHM_ONE_HALF);

        /*
            * PWM1模式下各扇区的三相比较值映射：
            * S1: U=Ta, V=Tb, W=Tc    S2: U=Tb, V=Ta, W=Tc
            * S3: U=Tc, V=Ta, W=Tb    S4: U=Tc, V=Tb, W=Ta
            * S5: U=Tb, V=Tc, W=Ta    S6: U=Ta, V=Tc, W=Tb
            *
            * 以Sector 1为例：
            * CCR_U = T7/2 + T6/2 + T4/2 = Ta
            * CCR_V = T7/2 + T6/2        = Tb
            * CCR_W = T7/2               = Tc
            */
        if (sector == 1U)
        {
            tcmp_u = ta;
            tcmp_v = tb;
            tcmp_w = tc;
        }
        else if (sector == 2U)
        {
            tcmp_u = tb;
            tcmp_v = ta;
            tcmp_w = tc;
        }
        else if (sector == 3U)
        {
            tcmp_u = tc;
            tcmp_v = ta;
            tcmp_w = tb;
        }
        else if (sector == 4U)
        {
            tcmp_u = tc;
            tcmp_v = tb;
            tcmp_w = ta;
        }
        else if (sector == 5U)
        {
            tcmp_u = tb;
            tcmp_v = tc;
            tcmp_w = ta;
        }
        else
        {
            tcmp_u = ta;
            tcmp_v = tc;
            tcmp_w = tb;
        }

        /* CCRx = round(Tcmp_x)，死区由TIM1互补输出硬件插入。 */
        result.compare_u = (uint16_t)(tcmp_u + FOC_ALGORITHM_ONE_HALF);
        result.compare_v = (uint16_t)(tcmp_v + FOC_ALGORITHM_ONE_HALF);
        result.compare_w = (uint16_t)(tcmp_w + FOC_ALGORITHM_ONE_HALF);
        *compare = result;
    }
    else
    {
        /* 扇区与作用时间不一致时不修改输出。 */
    }
}

/**
 * @brief 对完整三相电流执行等幅值Clarke变换。
 *
 * 分别使用U、V、W三相电流计算Alpha轴和Beta轴电流，不依赖三相电流
 * 之和为零的假设。计算结果先保存在局部结构体中，确保任一指针无效时
 * 不修改输出。
 *
 * @param[in] phase_current 三相电流输入，各成员单位一致且须为有限单精度
 *                          浮点数；指针不得为NULL。
 * @param[out] alpha_beta_current Alpha轴与Beta轴电流输出，单位与输入一致；
 *                                指针不得为NULL。
 *
 * @post 输入输出指针均有效时更新输出结构体，否则不修改输出。
 * @note 调用方负责避免计算结果超出float表示范围。
 */
void FocAlgorithm_Clarke(
    const Foc_PhaseCurrentType * phase_current,
    Foc_Curr_AlphaBetaType * alpha_beta)
{
    Foc_Curr_AlphaBetaType result = {0};

        /* Ialpha = (2/3) * (Iu - Iv/2 - Iw/2) */
        result.I_alpha = FOC_ALGORITHM_TWO_THIRDS *
                       (phase_current->phase_u -
                        (FOC_ALGORITHM_ONE_HALF * phase_current->phase_v) -
                        (FOC_ALGORITHM_ONE_HALF * phase_current->phase_w));

        /* Ibeta = (Iv - Iw) / sqrt(3) */
        result.I_beta = FOC_ALGORITHM_ONE_OVER_SQRT_THREE *
                      (phase_current->phase_v - phase_current->phase_w);

        *alpha_beta = result;
}

/**
 * @brief 执行Park变换。
 *
 * 将Alpha轴和Beta轴分量变换为同步旋转坐标系D轴和Q轴分量。计算结果
 * 先保存在局部结构体中，确保任一指针无效时不修改输出。
 *
 * @param[in] alpha_beta 两相静止坐标系输入，指针不得为NULL。
 * @param[in] trig 电角度的正弦与余弦值，指针不得为NULL。
 * @param[out] dq D轴与Q轴输出，单位与输入一致，指针不得为NULL。
 *
 * @post 所有指针均有效时更新输出结构体，否则不修改输出。
 * @note 调用方负责保证输入为有限值，且正弦与余弦值对应同一电角度。
 */
void FocAlgorithm_Park(const Foc_Curr_AlphaBetaType * alpha_beta,
                              const Foc_TrigType * trig,
                              Foc_Curr_DqType * dq)
{
    Foc_Curr_DqType result = {0};

        /* D = Alpha * cos(theta) + Beta * sin(theta) */
        result.Id = (alpha_beta->I_alpha * trig->cosine) +
                        (alpha_beta->I_beta * trig->sine);

        /* Q = -Alpha * sin(theta) + Beta * cos(theta) */
        result.Iq = -(alpha_beta->I_alpha * trig->sine) +
                            (alpha_beta->I_beta * trig->cosine);

        *dq = result;
}

/**
 * @brief 执行反Park变换。
 *
 * 将同步旋转坐标系D轴和Q轴分量变换为Alpha轴和Beta轴分量。计算结果
 * 先保存在局部结构体中，确保任一指针无效时不修改输出。
 *
 * @param[in] dq D轴与Q轴输入，指针不得为NULL。
 * @param[in] trig 电角度的正弦与余弦值，指针不得为NULL。
 * @param[out] alpha_beta 两相静止坐标系输出，单位与输入一致，指针不得为NULL。
 *
 * @post 所有指针均有效时更新输出结构体，否则不修改输出。
 * @note 调用方负责保证输入为有限值，且正弦与余弦值对应同一电角度。
 */
void FocAlgorithm_InversePark(const Foc_Volt_DqType * dq,
                                     const Foc_TrigType * trig,
                                     Foc_Volt_AlphaBetaType * alpha_beta)
{
    Foc_Volt_AlphaBetaType result = {0};

        /* Alpha = D * cos(theta) - Q * sin(theta) */
        result.V_alpha = (dq->Vd * trig->cosine) -
                       (dq->Vq * trig->sine);

        /* Beta = D * sin(theta) + Q * cos(theta) */
        result.V_beta = (dq->Vd * trig->sine) +
                      (dq->Vq * trig->cosine);

        *alpha_beta = result;
}

/**
 * @brief 按母线电压对D/Q轴电压执行圆限幅。
 *
 * @param[in,out] voltage_dq D/Q轴电压，单位V，不得为NULL。
 * @param[in] dc_bus_voltage_v 母线电压，单位V，必须大于0。
 *
 * @return 输入有效并完成限幅处理时返回true，否则返回false。
 */
void FocAlgorithm_LimitVoltageCircle(Foc_Volt_DqType * voltage_dq,
                                     float dc_bus_voltage_v)
{
    float voltage_limit_v = 0.0F;
    float magnitude_squared = 0.0F;
    float limit_squared = 0.0F;
    float scale = 1.0F;

        voltage_limit_v = dc_bus_voltage_v *
                          FOC_ALGORITHM_ONE_OVER_SQRT_THREE;
        magnitude_squared = (voltage_dq->Vd * voltage_dq->Vd) +
                            (voltage_dq->Vq * voltage_dq->Vq);
        limit_squared = voltage_limit_v * voltage_limit_v;

        if (magnitude_squared > limit_squared)
        {
            scale = voltage_limit_v / sqrtf(magnitude_squared);
            voltage_dq->Vd *= scale;
            voltage_dq->Vq *= scale;
        }
        else
        {
            /* 电压矢量位于圆限幅范围内。 */
        }
}



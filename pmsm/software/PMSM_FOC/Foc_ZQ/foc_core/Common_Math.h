/******************************************************************************
 * 文件: Common_Math.h
 * 功能: 通用数学库接口 (sin/cos 查表, 限幅, 梯度斜坡, 滤波器, 延时)
 * 创建日期: 2026-06-22
 * 修改日期: 2026-06-29
 * 修改内容: 新增相电流→母线电流/电磁转矩/电功率/机械功率/磁链计算函数
 * 修改人: rui.zhang
 ******************************************************************************/

#ifndef FOC_COMMON_MATH_H_
#define FOC_COMMON_MATH_H_

/* 仅依赖 stdint, 不拉入 foc_config.h —— 使本头可被 FOC 算法层(用 MATLAB 类型/
 * model 结构体)安全包含, 规避 foc_config.h 的 PWM_HalfPerMax/Control_Mode_* 宏冲突。
 * 需要 foc_config.h 的 .c (本库自身/main 等) 各自显式包含。 */
#include <stdint.h>

#define Abs(A)    ((A>=0)?A:-A)
#define Min(A,B)  ((A<=B)?A:B)
#define Max(A,B)  ((A>=B)?A:B)

typedef struct {
    uint16_t table_Angle;
    float    table_Sin;
    float    table_Cos;
} Ang_SinCos, *p_Ang_SinCos;

#define Ang_SinCos_DEFAULTS {0, 0.0f, 0.0f}

typedef struct {
    float    XieLv_X;
    float    XieLv_Y;
    float    XieLv_Grad;
    uint8_t  Timer_Count;
    uint8_t  Grad_Timer;
} GXieLv, *p_GXieLv;

#define GXieLv_DEFAULTS {0.0f, 0.0f, 0.0f, 0, 0}

typedef struct {
    float P_last;
    float P_now;
    float K_data;
    float Q_cov;
    float R_cov;
    float data_last;
    float data;
    float data_out;
    float data_out_f;
} Kalman_filter_tye;

typedef struct {
    double a[3];
    double b[3];
    double w[3];
} BandPassFilter;

typedef struct {
    double a[2];
    double b[3];
    double x[2];
    double y[2];
} ButterworthBandPassFilter;

/* sin/cos 查找表 */
extern const float fSin[1025];
/* 用户定义 */
extern Kalman_filter_tye kalman_spd;



float    Limit_Sat(float Uint, float U_max, float U_min);
uint32_t IQSqrt(uint32_t M);
void     SinCos_Table(p_Ang_SinCos PV);
void     Grad_XieLv(p_GXieLv pV);
void     Delay_ms(uint32_t t);
void     Delay_us(uint32_t t);
void     Delay_100ns(uint32_t t);

void kalman_filter_init(Kalman_filter_tye* kalman_n);
void kalman_filter_test_f(float data_in, Kalman_filter_tye* kalman_n);
void Butterworth_LowPass_Init(BandPassFilter *filter, double cutoff_freq, double sample_freq);
void Butterworth_BandPass_Init(ButterworthBandPassFilter *filter, double center_freq, double Q, double sample_freq);
double Butterworth_LowPass_Update(BandPassFilter *filter, double input);
double Butterworth_BandPass_Update(ButterworthBandPassFilter *filter, double input);

/* ********************* 电机物理量计算 ********************* */

/* 永磁磁链(Wb): EMF_const 线电压峰峰值/Krpm, pole_pairs 极对数 */
float CM_CalcFluxLinkage(float emf_const, uint16_t pole_pairs);

/* 标定用磁链估算(Wb): 实际Vq/Id/Iq、角速度we、Ld、Rs；稳态且we非零时调用 */
float CM_EstimateFluxLinkage(float vq, float iq, float id, float we, float ld, float rs);

/* 电磁转矩(N·m): id/iq dq电流(A), Ld/Lq 电感(H), flux_pm 磁链(Wb), pole_pairs 极对数 */
float CM_CalcTorque(float id, float iq, float ld, float lq, float flux_pm, uint16_t pole_pairs);

/* 电功率(W): Vd/Vq dq电压(V), Id/Iq dq电流(A) */
float CM_CalcElecPower(float vd, float vq, float id, float iq);

/* 机械功率(W): torque 转矩(N·m), speed_rpm 机械转速(RPM) */
float CM_CalcMechPower(float torque, int32_t speed_rpm);

/* 母线电流(A): Pe 电功率(W), V_bus 母线电压(V), 忽略驱动器损耗时的直流侧电流 */
float CM_CalcBusCurrentFromPower(float pe, float v_bus);

/* 母线电流(A): 相电流 → 母线电流, 用 dq 电压/电流与母线电压估算 */
float CM_CalcBusCurrent(float vd, float vq, float id, float iq, float v_bus);

#endif /* FOC_COMMON_MATH_H_ */

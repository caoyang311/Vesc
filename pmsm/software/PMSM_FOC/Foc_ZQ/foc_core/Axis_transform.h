/******************************************************************************
 * 文件: Axis_transform.h
 * 功能: Clarke/Park/iPark 坐标变换 接口
 * 创建日期: 2026-06-22
 * 修改日期: 2026-06-22
 * 修改内容: 建立坐标变换接口
 * 修改人: rui.zhang
 ******************************************************************************/

#ifndef FOC_AXIS_TRANSFORM_H_
#define FOC_AXIS_TRANSFORM_H_

/* 仅依赖 stdint, 不拉入 foc_config.h —— 供 FOC 算法层(用 model 结构体)安全包含,
 * 规避 foc_config.h 的 PWM_HalfPerMax/Control_Mode_* 宏冲突。.c 自身另含 foc_config.h。 */
#include <stdint.h>

typedef struct {
    float  Us;
    float  Vs;
    float  Ws;
    float  Alpha;
    float  Beta;
} CLARKE, *p_CLARKE;

#define CLARKE_DEFAULTS {0,0,0,0,0}

typedef struct {
    float    Alpha;
    float    Beta;
    uint16_t Angle;
    float    Ds;
    float    Qs;
    float    Sine;
    float    Cosine;
} PARK, *p_PARK;

#define PARK_DEFAULTS {0,0,0,0,0,0,0}

typedef struct {
    float  Alpha;
    float  Beta;
    float  Angle;
    float  Ds;
    float  Qs;
    float  Sine;
    float  Cosine;
} IPARK, *p_IPARK;

#define IPARK_DEFAULTS {0,0,0,0,0,0,0}

/* Clarke 变换 (U/V 相): Alpha=Iu, Beta=(Iu+2*Iv)/sqrt3 */
#define CLARKE_FASK_MACRO(v)                            \
    v.Alpha = v.Us;                                     \
    v.Beta  = (v.Us + v.Vs*2)*0.57735026918f;

/* Clarke 变换 (U/W 相): 由 Iu,Iw 计算 (适用于 V 相由 -(U+W) 得到或 V 相不可靠)
 *   Alpha = Iu
 *   Beta  = -(Iu + 2*Iw)/sqrt3   (将 Iv=-(Iu+Iw) 代入 U/V 公式 (Iu+2*Iv)/sqrt3 化简)
 * 与"先算 Iv 再走 U/V Clarke"代数等价。 */
#define CLARKE_UW_FASK_MACRO(v)                         \
    v.Alpha = v.Us;                                     \
    v.Beta  = -(v.Us + v.Ws*2)*0.57735026918f;

/* Clarke 变换 (V/W 相): 由 Iv,Iw 反推 (Iu=-(Iv+Iw))
 *   Alpha = Iu      = -(Iv + Iw)
 *   Beta  = (Iv-Iw)/sqrt3
 * 适用于只采样 V/W 两相(或 U 相不可靠)的硬件配置。 */
#define CLARKE_VW_FASK_MACRO(v)                         \
    v.Alpha = -(v.Vs + v.Ws);                           \
    v.Beta  = (v.Vs - v.Ws)*0.57735026918f;

/* Park 变换 */
#define PARK_FASK_MACRO(v)                              \
    v.Ds = v.Alpha * v.Cosine + v.Beta * v.Sine;        \
    v.Qs = v.Beta * v.Cosine - v.Alpha * v.Sine;

/* 反 Park 变换 */
#define iPARK_FASK_MACRO(v)                             \
    v.Alpha = v.Ds * v.Cosine - v.Qs * v.Sine;          \
    v.Beta  = v.Ds * v.Sine  + v.Qs * v.Cosine;

void CLARKE_Cale(p_CLARKE pV);     /* U/V 相 Clarke */
void CLARKE_VW_Cale(p_CLARKE pV);  /* V/W 相 Clarke (Alpha=-(Iv+Iw), Beta=(Iv-Iw)/sqrt3) */
void CLARKE_UW_Cale(p_CLARKE pV);  /* U/W 相 Clarke (Alpha=Iu, Beta=-(Iu+2*Iw)/sqrt3) */
void PARK_Cale(p_PARK pV);
void IPARK_Cale(p_IPARK pV);

#endif /* FOC_AXIS_TRANSFORM_H_ */

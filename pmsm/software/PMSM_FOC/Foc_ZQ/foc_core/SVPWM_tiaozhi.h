/******************************************************************************
 * 文件: SVPWM_tiaozhi.h
 * 功能: SVPWM 空间矢量调制 接口
 * 创建日期: 2026-06-22
 * 修改日期: 2026-10-24
 * 修改内容: SVPWM 与三/六次零序注入接口
 * 修改人: rui.zhang
 ******************************************************************************/

#ifndef FOC_SVPWM_TIAOZHI_H_
#define FOC_SVPWM_TIAOZHI_H_

/* 仅依赖 stdint, 不拉入 foc_config.h (同 Axis_transform.h)。.c 自身另含 foc_config.h。 */
#include <stdint.h>

#define FOC_MOD_SVPWM 0
#define FOC_MOD_THI3  1
#define FOC_MOD_THI6  2

#ifndef FOC_MODULATION_MODE
#define FOC_MODULATION_MODE FOC_MOD_SVPWM
#endif

#if (FOC_MODULATION_MODE != FOC_MOD_SVPWM) && \
    (FOC_MODULATION_MODE != FOC_MOD_THI3) && \
    (FOC_MODULATION_MODE != FOC_MOD_THI6)
#error "FOC_MODULATION_MODE must be FOC_MOD_SVPWM, FOC_MOD_THI3 or FOC_MOD_THI6"
#endif

#ifndef FOC_THI6_GAIN
#define FOC_THI6_GAIN (1.0F / 6.0F)
#endif

#if FOC_MODULATION_MODE != FOC_MOD_SVPWM
#include <math.h>
#endif

typedef struct {
    float    Ualpha;
    float    Ubeta;
    float    Ta;
    float    Tb;
    float    Tc;
    float    SVPTa;
    float    SVPTb;
    float    SVPTc;
    float    tmp1;
    float    tmp2;
    float    tmp3;
    uint16_t VecSector;
} SVPWM, *p_SVPWM;

#define SVPWM_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0}

static inline void SVPWM_InjectZeroSeq(p_SVPWM pV)
{
#if FOC_MODULATION_MODE == FOC_MOD_THI6
    float m = sqrtf((pV->Ualpha * pV->Ualpha) + (pV->Ubeta * pV->Ubeta));
    float u0 = 0.0F;

    if (m > 1.0e-6F)
    {
        float c = pV->Ualpha / m;
        float c3 = (4.0F * c * c * c) - (3.0F * c);
        float c6 = (2.0F * c3 * c3) - 1.0F;
        u0 = -FOC_THI6_GAIN * m * c6;
    }

    pV->Ta += u0;
    pV->Tb += u0;
    pV->Tc += u0;
#else
    float ua = pV->Ualpha;
    float ub = -0.5F * pV->Ualpha + 0.8660254F * pV->Ubeta;
    float uc = -0.5F * pV->Ualpha - 0.8660254F * pV->Ubeta;
    float u0;

#if FOC_MODULATION_MODE == FOC_MOD_SVPWM
    float max_phase = ua;
    float min_phase = ua;

    if (ub > max_phase) { max_phase = ub; }
    if (uc > max_phase) { max_phase = uc; }
    if (ub < min_phase) { min_phase = ub; }
    if (uc < min_phase) { min_phase = uc; }
    u0 = -(max_phase + min_phase) * 0.5F;
#else
    float m = sqrtf((pV->Ualpha * pV->Ualpha) + (pV->Ubeta * pV->Ubeta));
    u0 = 0.0F;
    if (m > 1.0e-6F)
    {
        float c = pV->Ualpha / m;
        float c3 = (4.0F * c * c * c) - (3.0F * c);
        u0 = -(m / 6.0F) * c3;
    }
#endif

    pV->Ta = ua + u0;
    pV->Tb = ub + u0;
    pV->Tc = uc + u0;
#endif
}

void SVPWM_Cale(p_SVPWM pV);

#endif /* FOC_SVPWM_TIAOZHI_H_ */

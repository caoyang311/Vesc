/******************************************************************************
 * 文件: SVPWM_tiaozhi.c
 * 功能: SVPWM 空间矢量调制 实现
 * 创建日期: 2026-06-22
 * 修改日期: 2026-10-24
 * 修改内容: SVPWM 与三/六次零序注入实现
 * 修改人: rui.zhang
 ******************************************************************************/

#include "foc_config.h"   /* My_SQRT3_2, My_div2, Svpwm_Km, PWM_HalfPerMax */
#include "SVPWM_tiaozhi.h"

void SVPWM_Cale(p_SVPWM pV)
{
    pV->tmp1 = pV->Ubeta;
    pV->tmp2 = pV->Ubeta * My_div2 + pV->Ualpha * My_SQRT3_2;
    pV->tmp3 = pV->tmp2 - pV->tmp1;

    pV->VecSector = 3;
    pV->VecSector = (pV->tmp2 > 0) ? (pV->VecSector - 1) : pV->VecSector;
    pV->VecSector = (pV->tmp3 > 0) ? (pV->VecSector - 1) : pV->VecSector;
    pV->VecSector = (pV->tmp1 < 0) ? (7 - pV->VecSector)  : pV->VecSector;

#if FOC_MODULATION_MODE == FOC_MOD_THI6
    if (pV->VecSector == 1 || pV->VecSector == 4)
    {
        pV->Ta = pV->tmp2;
        pV->Tb = pV->tmp1 - pV->tmp3;
        pV->Tc = -pV->tmp2;
    }
    else if (pV->VecSector == 2 || pV->VecSector == 5)
    {
        pV->Ta = pV->tmp3 + pV->tmp2;
        pV->Tb = pV->tmp1;
        pV->Tc = -pV->tmp1;
    }
    else if (pV->VecSector == 3 || pV->VecSector == 6)
    {
        pV->Ta = pV->tmp3;
        pV->Tb = -pV->tmp3;
        pV->Tc = -(pV->tmp1 + pV->tmp2);
    }
    else
    {
        pV->Ta = 0;
        pV->Tb = 0;
        pV->Tc = 0;
    }
#endif

    SVPWM_InjectZeroSeq(pV);

/******注意!*********/
/******此处为宏定义参考值与实际并不一样!*********/
    pV->SVPTa = (pV->Ta / Svpwm_Km_def) * PWM_HalfPerMax_def + PWM_HalfPerMax_def;
    pV->SVPTb = (pV->Tb / Svpwm_Km_def) * PWM_HalfPerMax_def + PWM_HalfPerMax_def;
    pV->SVPTc = (pV->Tc / Svpwm_Km_def) * PWM_HalfPerMax_def + PWM_HalfPerMax_def;
}

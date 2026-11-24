/******************************************************************************
 * 文件: Axis_transform.c
 * 功能: Clarke/Park/iPark 坐标变换 实现
 * 创建日期: 2026-06-22
 * 修改日期: 2026-06-22
 * 修改内容: 建立坐标变换实现
 * 修改人: rui.zhang
 ******************************************************************************/

#include "foc_config.h"   /* My_SQRT3_3 (Axis_transform.h 已不再拉入) */
#include "Axis_transform.h"

void CLARKE_Cale(p_CLARKE pV)
{
    pV->Alpha = pV->Us;
    pV->Beta  = (pV->Us + pV->Vs * 2.0f) * My_SQRT3_3;
}

/* V/W 相 Clarke: 由 Iv,Iw 反推 (三相和为零 Iu = -(Iv+Iw))
 *   Alpha = Iu      = -(Iv + Iw)
 *   Beta  = (Iv - Iw) / sqrt3
 */
void CLARKE_VW_Cale(p_CLARKE pV)
{
    pV->Alpha = -(pV->Vs + pV->Ws);
    pV->Beta  = (pV->Vs - pV->Ws) * My_SQRT3_3;
}

/* U/W 相 Clarke: 适用于 V 相由 -(U+W) 得到(或 V 相不可靠)的硬件配置
 *   Alpha = Iu
 *   Beta  = -(Iu + 2*Iw) / sqrt3
 * 与"先算 Iv=-(Iu+Iw) 再走 U/V Clarke"代数等价。
 */
void CLARKE_UW_Cale(p_CLARKE pV)
{
    pV->Alpha = pV->Us;
    pV->Beta  = -(pV->Us + pV->Ws * 2.0f) * My_SQRT3_3;
}

void PARK_Cale(p_PARK pV)
{
    pV->Ds = pV->Alpha * pV->Cosine + pV->Beta * pV->Sine;
    pV->Qs = pV->Beta * pV->Cosine  - pV->Alpha * pV->Sine;
}

void IPARK_Cale(p_IPARK pV)
{
    pV->Alpha = pV->Ds * pV->Cosine - pV->Qs * pV->Sine;
    pV->Beta  = pV->Qs * pV->Cosine + pV->Ds * pV->Sine;
}

/******************************************************************************
 * 文件: foc_pi.h
 * 功能: PI 控制器接口 (速度/位置环, 电流/弱磁环) + 参数初始化
 * 创建日期: 2026-06-22
 * 修改日期: 2026-06-22
 * 修改内容: PI 改为函数式 (就地操作 PI_Params)
 * 修改人: rui.zhang
 ******************************************************************************/

/* PI_Params 定义在 model/PI_revise.h; 调用前写 v->Ref/v->Fbk, 函数内就地更新 PI_out 等 */

#ifndef FOC_PI_H_
#define FOC_PI_H_

#include "PI_revise.h"

/* 速度/位置环 PI (设定值加权 b + 死区 DeadBand + back-calculation 抗饱和 Kaw)
 * 调用前写 v->Ref, v->Fbk; 返回后 v->PI_out 为输出 */
void PI_Contrl(PI_Params *v);

/* 电流/弱磁环 PI (条件积分 + 可选动态限幅)
 * limit_state=1 时用 ±Limit 覆盖上下限 (电流环 Iq 传 sqrt(Vdc^2 - Vd^2)) */
void PI_Contrl_Modern(PI_Params *v, uint8_T limit_state, real32_T Limit);

/* PI 参数初始化 (各环 Kp/Ki/Kaw/b/上下限) */
void PI_Pare_init(void);

/* 将 comm_pi_bus (CAN A1 参数总线) 应用到实际控制环*/
void PI_Apply_CommParams(void);
#endif /* FOC_PI_H_ */

/******************************************************************************
 * 文件: MTPV_MAP.h
 * 功能: MTPV MAP + 标定表 查表接口 (Id/Iq 二维, Rs/Psi 温补, Ld/Lq 饱和)
 * 创建日期: 2026-06-22
 * 修改日期: 2026-10-10
 * 修改内容: 删除 RAM 镜像: 表去 const 由 0x21 直接读写(上电 C 初始值装载);
 *           新增量程/间隔独立变量声明
 *           Rs/磁链/Ld/Lq 表有效点数接口(idx3/idx4 写入)
 * 修改人: rui.zhang
 ******************************************************************************/

#ifndef MTPV_MAP_H_
#define MTPV_MAP_H_
#include <stdint.h>
/* ===== 代码版本标识 =====
   2 = MAP 已标定写死版 (当前): 直接运行 Id_2D_Map.c 里的标定表, 不做任何上电重算
   仅作版本标识, 不触发任何逻辑。 */
#define EMOTOR_CODE_VERSION 2

/* --- Id/Iq 扭矩MAP --- */
#define TORQUE_POINTS    32
#define SPEED_POINTS     32
extern float Torque_Table[TORQUE_POINTS];
extern float Speed_Table[SPEED_POINTS];
extern float Id_Map[TORQUE_POINTS][SPEED_POINTS];
extern float Iq_Map[TORQUE_POINTS][SPEED_POINTS];
float Lookup_Id(float torque, float speed);
float Lookup_Iq(float torque, float speed);

/* --- Rs 温度补偿 --- */
#define RS_TEMP_POINTS   10
extern float Rs_Temp_Table[RS_TEMP_POINTS];
extern float Rs_Value_Table[RS_TEMP_POINTS];
float Lookup_Rs(float temp_C);

/* --- Psi 温度补偿 --- */
#define PSI_TEMP_POINTS  10
extern float Psi_Temp_Table[PSI_TEMP_POINTS];
extern float Psi_Value_Table[PSI_TEMP_POINTS];
float Lookup_Psi(float temp_C);

/* --- Ld/Lq 饱和 2D 查表 (返回 uH; 轴外钳位到边界, 轴等间隔 120A) --- */
float Lookup_Ld(float id_a, float iq_a);
float Lookup_Lq(float id_a, float iq_a);

/* --- Ld/Lq 饱和表 --- */
#define LD_SAT_ID_PTS    6
#define LD_SAT_IQ_PTS    6
#define LQ_SAT_ID_PTS    6
#define LQ_SAT_IQ_PTS    6
extern float Ld_Id_Axis[LD_SAT_ID_PTS];
extern float Ld_Iq_Axis[LD_SAT_IQ_PTS];
extern float Ld_Sat_Map[LD_SAT_ID_PTS][LD_SAT_IQ_PTS];
extern float Lq_Id_Axis[LQ_SAT_ID_PTS];
extern float Lq_Iq_Axis[LQ_SAT_IQ_PTS];
extern float Lq_Sat_Map[LQ_SAT_ID_PTS][LQ_SAT_IQ_PTS];




/* ============ 标定表运行状态与量程 (表本体即上方数组, 去 const 后 0x21 直接读写;
   上电由 C 初始值装载。D/Q 表展平: idx = 转矩索引*SPEED_POINTS + 转速索引) ============ */
extern uint8_t g_map_rw_status;      /* 0x21/01: 0x01成功 0x02索引超范围 0x03数据超范围 */
extern float   g_map_torque_step;    /* 扭矩间隔 N.m (0x20/03 idx1 只读) */
extern float   g_map_speed_step;     /* 转速间隔 rpm (0x20/03 idx1 只读) */
extern float   g_map_tq_max;         /* 扭矩满量程 N.m (0x20/03 idx0 写/读) */
extern float   g_map_spd_max;        /* 转速满量程 rpm (0x20/03 idx0 写/读) */
uint8_t MapStore_Read(uint8_t sub, uint16_t idx, float *v);   /* 0x21 读: 1=成功 */
uint8_t MapStore_Write(uint8_t sub, uint16_t idx, float v);   /* 0x21 写: 1=成功 */
void    MapStore_RebuildAxis(void);  /* 按间隔重算等间隔轴 */

/* ---- MTPA 有效尺寸(运行时可配): 存储仍按满 32 布局, 有效矩阵=前 tq_pts 行 x 前 spd_pts 列;
   展平索引仍按 32 步长(idx = 转矩索引*32 + 转速索引), 越出有效矩阵的格读写被拒。
   尺寸为 RAM 态, 上电回满 32(表不落盘口径一致); 写入走 0x20/03 idx2(标定闸门内) ---- */
extern uint8_t g_map_tq_pts;         /* 扭矩轴有效点数: 3..TORQUE_POINTS */
extern uint8_t g_map_spd_pts;        /* 转速轴有效点数: 3..SPEED_POINTS */
uint8_t MapStore_SetSize(uint8_t tq_pts, uint8_t spd_pts);   /* 1=成功; <3 或 >32 拒绝(状态 0x02) */

/* ---- Rs/磁链/Ld/Lq 表有效点数(运行时可写, 只改窗口; 0x20/03 idx3/idx4, 标定闸门内) ---- */
extern uint8_t g_map_rs_pts;     /* Rs 温度轴有效点数: 3..RS_TEMP_POINTS */
extern uint8_t g_map_psi_pts;    /* 磁链温度轴有效点数: 3..PSI_TEMP_POINTS */
extern uint8_t g_map_ld_id_pts;  /* Ld/Lq 表 Id 轴有效点数: 3..LD_SAT_ID_PTS */
extern uint8_t g_map_ld_iq_pts;  /* Ld/Lq 表 Iq 轴有效点数: 3..LD_SAT_IQ_PTS */
uint8_t MapStore_SetAuxSize(uint8_t rs_pts, uint8_t psi_pts);   /* 1=成功; 越界拒绝 */
uint8_t MapStore_SetLdSize(uint8_t id_pts, uint8_t iq_pts);     /* 1=成功; 越界拒绝 */

/* ---- MAP 在线重算 (MapRegen.c; CAN 0x20/01 idx0 cmd=0x20 触发, 主循环分步执行) ---- */
#define MAPREGEN_ST_IDLE    0x00U   /* 无请求/无结果 */
#define MAPREGEN_ST_OK      0x01U   /* 成功 */
#define MAPREGEN_ST_REJ_RUN 0x02U   /* 拒绝/中止: 电机运行中 */
#define MAPREGEN_ST_REJ_PAR 0x03U   /* 拒绝: 电机参数非法 */
#define MAPREGEN_ST_WARN    0x04U   /* 完成但存在非有限值格 */
#define MAPREGEN_ST_BUSY    0x05U   /* 进行中 */
extern volatile uint8_t g_map_regen_req;    /* 重算请求: CAN 置位, 主循环消费 */
extern uint8_t  g_map_regen_st;             /* 重算状态(调试观察) */
extern uint16_t g_map_regen_vlim;           /* 电压受限格数(调试观察) */
extern float    g_map_regen_errmax;         /* 非受限格最大扭矩偏差 N.m */
uint8_t MapStore_RegenStart(float psi, float ld, float lq, float rs,
                            float pp, float udc, float imax);   /* 1=已启动 */
uint8_t MapStore_RegenBusy(void);                                /* 1=进行中 */
void    MapStore_RegenAbort(void);                               /* 运行中中止 */
void    MapStore_RegenTask(void);                                /* 每调用推进一扭矩行 */
#endif /* MTPV_MAP_H_ */

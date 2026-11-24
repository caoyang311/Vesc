/******************************************************************************
 * 文件: temp_sensor.c
 * 功能: 温度传感器电阻值 -> 摄氏温度 (PT100/PT1000/KTY84 分度表 + NTC 公式)
 * 创建日期: 2026-06-22
 * 修改日期: 2026-10-27
 * 修改内容: 新增金宇星PT1000 ADC直查表(-50~200℃, 类型2, 源金宇星PT1000.xlsx);
 *           查表函数方向自适应; MOS采样电阻使用正确的ADC输入通道
 * 修改人: rui.zhang
 ******************************************************************************/

#include "temp_sensor.h"
#include <math.h>
#include "adc.h"                         /* hadc1 */
#include "foc_data.h"                    /* DW */
#include "MotorCtr_parameter.h"    /* CONVERT_MOS_RS_CONST / CONVERT_MOTOR_RS_CONST */
#if(TEMP_CALCULATE_MODE==0)

/* ******************** NTC (Steinhart-Hart) 参数 (源 MotorCtr_parameter.h) ******************** */
#define A_COEF      0.001307f      /* Steinhart-Hart 系数 A */
#define B_COEF      0.000234f      /* Steinhart-Hart 系数 B */
#define R_TO_TEMP_C(R_ntc)   ((1.0f / (A_COEF + B_COEF * logf(R_ntc)) - 273.15f) * 0.825f)

/* ******************** 分段插值常量 (表外延伸的每℃电阻变化, 源参考工程) ******************** */
#define PT100_constL    0.3965f
#define PT100_constH    0.381f
#define PT1000_constL   39.644f
#define PT1000_constH   37.986f
#define KTY84_constL    32.0f
#define KTY84_constH    61.0f

/* ******************** 分度表 -50℃~100℃, 每 10℃ 一段 {温度, 电阻} ******************** */
/* PT100 */
static const float pt100_table[16][2] = {
    {-50, 80.306f}, {-40, 84.271f}, {-30, 88.222f}, {-20, 92.160f}, {-10, 96.085f},
    {0, 100.000f}, {10, 103.896f}, {20, 107.789f}, {30, 111.668f}, {40, 115.540f},
    {50, 119.400f}, {60, 123.240f}, {70, 127.080f}, {80, 130.890f}, {90, 134.700f},
    {100, 138.510f}
};
/* PT1000 */
static const float pt1000_table[16][2] = {
    {-50, 803.063f}, {-40, 842.707f}, {-30, 882.217f}, {-20, 921.599f}, {-10, 960.859f},
    {0, 1000.000f}, {10, 1039.025f}, {20, 1077.935f}, {30, 1116.729f}, {40, 1155.408f},
    {50, 1193.971f}, {60, 1232.419f}, {70, 1270.751f}, {80, 1308.968f}, {90, 1347.069f},
    {100, 1385.055f}
};
/* KTY84-130 */
static const float KTY84_table[16][2] = {
    {-50, 328.0f}, {-40, 359.0f}, {-30, 391.0f}, {-20, 424.0f}, {-10, 460.0f},
    {0, 498.0f}, {10, 538.0f}, {20, 581.0f}, {30, 626.0f}, {40, 672.0f},
    {50, 722.0f}, {60, 773.0f}, {70, 826.0f}, {80, 882.0f}, {90, 940.0f},
    {100, 1000.0f}
};

/* ******************** 通用分度表插值 (合并参考工程三个分支的相同逻辑) ******************** */
/* 电阻单调随温度递增, 故表项按电阻升序排列, 线性插值; 表外用首/末段斜率外延。 */
static float table_lookup(const float table[16][2], float Res_in,
                          float slope_low, float slope_high)
{
    int i;

    /* 低于表下限: 以首段每℃斜率向下外延 */
    if (Res_in < table[0][1]) {
        return (Res_in - table[0][1]) / slope_low * 10.0f + table[0][0];
    }
    /* 表内: 找到所在区间线性插值 */
    for (i = 0; i < 15; i++) {
        if ((Res_in >= table[i][1]) && (Res_in < table[i + 1][1])) {
            float span = table[i + 1][1] - table[i][1];
            return (Res_in - table[i][1]) / span * 10.0f + table[i][0];
        }
    }
    /* 高于表上限: 以末段每℃斜率向上外延 */
    return (Res_in - table[15][1]) / slope_high * 10.0f + table[15][0];
}

/* ******************** 对外接口 ******************** */

float Read_Temp_fun(int S_type, float Res_in)
{
    switch (S_type) {
    case 100:  return table_lookup(pt100_table,  Res_in, PT100_constL,  PT100_constH);
    case 1000: return table_lookup(pt1000_table, Res_in, PT1000_constL, PT1000_constH);
    case 84:   return table_lookup(KTY84_table,  Res_in, KTY84_constL,  KTY84_constH);
    default:   return 0.0f;
    }
}

float Temp_NTC_FromRes(float R_ntc)
{
    if (R_ntc <= 0.0f) { return 0.0f; }   /* 防 log 非法入参 */
    return R_TO_TEMP_C(R_ntc);
}
float Temp_FromResistance(uint8_t sensor_type, float Rs)
{
    switch (sensor_type) {
    case 0:  return Read_Temp_fun(84,   Rs);  /* KTY84-130 */
    case 1:  return Read_Temp_fun(100,  Rs);  /* PT100 */
    case 2:  return Read_Temp_fun(1000, Rs);  /* PT1000 */
    case 3:  return Read_Temp_fun(84,   Rs);  /* KTY83: 参考工程无独立分度表, 暂用 KTY84 表近似 */
    case 4:  return Temp_NTC_FromRes(Rs);     /* NTC-10K: Steinhart-Hart */
    default: return 0.0f;
    }
}

/* ******************** 主循环 ADC1 双路温度采集 (无DMA) ******************** */
/* 硬件: PC0=ADC1_IN6=电机温度(rank1), PC1=ADC1_IN7=控制器(MOS)温度(rank2)。
 * ADC1 由 TIM4_TRGO 触发扫描两路, 不用 DMA, 单数据寄存器无法区分通道, 故先
 * Stop+Start 让规则序列指针归零, 再顺序轮询两次 EOC, 第1次必是 rank1, 第2次
 * 必是 rank2, 保证数据与通道一一对应。TIM4≈1kHz, 等一次触发+转换 < 5ms。 */

#define TEMP_ADC_TIMEOUT_MS   5U

static float s_motor_rs = 0.0f;   /* 电机采样电阻 LPF 状态 */
static float s_mos_rs   = 0.0f;   /* 控制器(MOS)采样电阻 LPF 状态 */
static float s_motor_temp = 0.0f; /* 电机温度输出 LPF 状态 */
static float s_mos_temp   = 0.0f; /* MOS 温度输出 LPF 状态 */

/* 一阶低通: y = a*x + (1-a)*y_prev */
static inline float temp_lpf(float y_prev, float x, float a)
{
    return (a * x) + ((1.0f - a) * y_prev);
}
void Temp_Sensor_Update(void)
{
		uint16_t motor_raw;
    uint16_t mos_raw;

    /* DMA 后台 1kHz 刷新 g_temp_adc, 这里只需读取快照。
       单个 uint16_t 读取本身原子, 温度变化慢, 两通道差一拍(1ms)可忽略, 无需关中断。 */
    motor_raw = g_temp_adc[0];   /* rank1 = CH6 = PC0 = 电机温度 */
    mos_raw   = g_temp_adc[1];   /* rank2 = CH7 = PC1 = MOS温度  */

    /* 原始 ADC 值 (供故障检测) */
    DW.adc_slmple.mototr_Temp = motor_raw;
    DW.adc_slmple.MOS_Temp    = mos_raw;

    /* ADC -> 采样电阻 + 一阶低通 */
    s_motor_rs = temp_lpf(s_motor_rs, (UP_RES_CONSTANT1 * (float)motor_raw)/(ADC_CONSTANT1 - 3.3*(float)motor_raw), 0.35f);
		//R_NTC = (V_adc_ref × R_pullup) × ADC / (V_pullup × ADC_MAX - V_adc_ref × ADC)
    s_mos_rs   = temp_lpf(s_mos_rs,   (UP_RES_CONSTANT1 * (float)mos_raw)/(ADC_CONSTANT1 - 3.3*(float)mos_raw),   0.35f);
    DW.motor_anal.motor_temp_Rs = s_motor_rs;
    DW.motor_anal.mos_temp_Rs   = s_mos_rs;

    /* 电阻 -> 温度 (按传感器类型查表/NTC公式), 再对温度输出做一阶低通 */
    DW.motor_anal.motor_temp = temp_lpf(s_motor_temp,
                                        Temp_FromResistance(DW.motor_param.temp_sensor, s_motor_rs),
                                        TEMP_LPF_ALPHA);
    s_motor_temp = DW.motor_anal.motor_temp;
    DW.motor_anal.mos_temp = temp_lpf(s_mos_temp,
                                      Temp_FromResistance(DW.comm_contrl.mos_temp_sensor, s_mos_rs),
                                      TEMP_LPF_ALPHA);
    s_mos_temp = DW.motor_anal.mos_temp;
}

#elif(TEMP_CALCULATE_MODE==1)
/** @brief 温度查表节点 */
typedef struct {
    int16_t  temp;      								/* 温度值, 单位0.1°C, 如-300表示-30.0°C */
    uint16_t adc_raw;   								/* 对应ADC原始值(12bit, 0~4095) */
} TempLookupNode_t;

/** @brief 温度查表表头 */
typedef struct {
    const TempLookupNode_t *table;
    uint16_t                size;
    int16_t                 temp_min;   /* 最小温度(0.1°C) */
    int16_t                 temp_max;   /* 最大温度(0.1°C) */
} TempLookupTable_t;
/* **************************************** */
/*            表1: B=3450, -30~120°C        */
/* **************************************** */
static const TempLookupNode_t g_ntc_b3450_table[] = {
    {  -300, 4071 },  /*  -30°C -> 3.2806V */
    {  -250, 4051 },  /*  -25°C -> 3.2647V */
    {  -200, 4027 },  /*  -20°C -> 3.2449V */
    {  -150, 3996 },  /*  -15°C -> 3.2205V */
    {  -100, 3960 },  /*  -10°C -> 3.1909V */
    {   -50, 3915 },  /*   -5°C -> 3.1552V */
    {     0, 3863 },  /*    0°C -> 3.1127V      */
    {    50, 3801 },  /*    5°C -> 3.0628V */
    {   100, 3729 },  /*   10°C -> 3.0048V */
    {   150, 3646 },  /*   15°C -> 2.9379V */
    {   200, 3552 },  /*   20°C -> 2.8625V */
    {   250, 3447 },  /*   25°C -> 2.7777V */
    {   300, 3331 },  /*   30°C -> 2.6842V */
    {   350, 3205 },  /*   35°C -> 2.5826V */
    {   400, 3069 },  /*   40°C -> 2.4731V */
    {   450, 2927 },  /*   45°C -> 2.3587V */
    {   500, 2776 },  /*   50°C -> 2.2368V */
    {   550, 2621 },  /*   55°C -> 2.1123V */
    {   600, 2465 },  /*   60°C -> 1.9865V */
    {   650, 2306 },  /*   65°C -> 1.8584V */
    {   700, 2152 },  /*   70°C -> 1.7346V */
    {   750, 1999 },  /*   75°C -> 1.6107V */
    {   800, 1851 },  /*   80°C -> 1.4917V */
    {   850, 1710 },  /*   85°C -> 1.3783V */
    {   900, 1575 },  /*   90°C -> 1.2693V */
    {   950, 1442 },  /*   95°C -> 1.1618V */
    {  1000, 1323 },  /*  100°C -> 1.0658V */
    {  1050, 1213 },  /*  105°C -> 0.9776V */
    {  1100, 1106 },  /*  110°C -> 0.8913V */
    {  1150, 1015 },  /*  115°C -> 0.8176V */
    {  1200,  930 },  /*  120°C -> 0.7494V */
		{  1250,  840 },  /*  120°C -> 0.6772V */
};
static const TempLookupTable_t g_ntc_b3450 = {
    .table    = g_ntc_b3450_table,
    .size     = sizeof(g_ntc_b3450_table) / sizeof(g_ntc_b3450_table[0]),
    .temp_min = -300,   /* -30.0°C */
    .temp_max = 1250,   /* 125.0°C */
};
/* **************************************** */
/*  表2: ZHNTC-QSF B=3435, -40~180°C        */
/* **************************************** */
static const TempLookupNode_t g_ntc_zhntc_qsf_table[] = {
    {  -400, 3851 },  /*   -40°C -> 3.102271V */
    {  -350, 3841 },  /*   -35°C -> 3.094940V */
    {  -300, 3830 },  /*   -30°C -> 3.085656V */
    {  -250, 3816 },  /*   -25°C -> 3.074016V */
    {  -200, 3798 },  /*   -20°C -> 3.059578V */
    {  -150, 3776 },  /*   -15°C -> 3.041844V */
    {  -100, 3749 },  /*   -10°C -> 3.020298V */
    {   -50, 3717 },  /*    -5°C -> 2.994393V */
    {     0, 3678 },  /*     0°C -> 2.963529V */
    {    50, 3633 },  /*     5°C -> 2.927215V */
    {   100, 3581 },  /*    10°C -> 2.884862V */
    {   150, 3520 },  /*    15°C -> 2.836005V */
    {   200, 3451 },  /*    20°C -> 2.780280V */
    {   250, 3373 },  /*    25°C -> 2.717391V */
    {   300, 3286 },  /*    30°C -> 2.647220V */
    {   350, 3190 },  /*    35°C -> 2.569872V */
    {   400, 3085 },  /*    40°C -> 2.485505V */
    {   450, 2972 },  /*    45°C -> 2.394632V */
    {   500, 2852 },  /*    50°C -> 2.297697V */
    {   550, 2726 },  /*    55°C -> 2.195862V */
    {   600, 2594 },  /*    60°C -> 2.089775V */
    {   650, 2458 },  /*    65°C -> 1.980591V */
    {   700, 2321 },  /*    70°C -> 1.869644V */
    {   750, 2182 },  /*    75°C -> 1.757983V */
    {   800, 2044 },  /*    80°C -> 1.646760V */
    {   850, 1907 },  /*    85°C -> 1.536555V */
    {   900, 1775 },  /*    90°C -> 1.429702V */
    {   950, 1645 },  /*    95°C -> 1.325576V */
    {  1000, 1521 },  /*   100°C -> 1.225689V */
    {  1050, 1403 },  /*   105°C -> 1.130319V */
    {  1100, 1291 },  /*   110°C -> 1.039813V */
    {  1150, 1185 },  /*   115°C -> 0.954861V */
    {  1200, 1087 },  /*   120°C -> 0.875720V */
    {  1250,  996 },  /*   125°C -> 0.802156V */
    {  1300,  910 },  /*   130°C -> 0.733418V */
    {  1350,  831 },  /*   135°C -> 0.669526V */
    {  1400,  759 },  /*   140°C -> 0.611595V */
    {  1450,  692 },  /*   145°C -> 0.557913V */
    {  1500,  632 },  /*   150°C -> 0.509208V */
    {  1550,  577 },  /*   155°C -> 0.464671V */
    {  1600,  527 },  /*   160°C -> 0.424827V */
    {  1650,  482 },  /*   165°C -> 0.388573V */
    {  1700,  440 },  /*   170°C -> 0.354610V */
    {  1750,  403 },  /*   175°C -> 0.324821V */
    {  1800,  370 },  /*   180°C -> 0.297799V */
};
static const TempLookupTable_t g_ntc_zhntc_qsf = {
    .table    = g_ntc_zhntc_qsf_table,
    .size     = sizeof(g_ntc_zhntc_qsf_table) / sizeof(g_ntc_zhntc_qsf_table[0]),
    .temp_min = -400,   /* -40.0°C */
    .temp_max = 1800,   /* 150.0°C */
};
/* **************************************** */
/*  表3: 金宇星 PT1000, -50~200°C (源 金宇星PT1000.xlsx)        */
/* **************************************** */
/* 电路: PT1000 + 1.5k 上拉到 5V -> 电压跟随器(V_f) -> 0.625 衰减 -> ADC(3.3V/12bit)
   adc_raw = V_f*0.625*4095/3.3, 5℃ 步进线性插值;*/
static const TempLookupNode_t g_pt1000_jyx_table[] = {
    { -500, 1352 },  /*  -50℃ -> R= 803.06Ω V_f=1.7435V V_dsp=1.0897V */
    { -450, 1374 },  /*  -45℃ -> R= 822.90Ω V_f=1.7713V V_dsp=1.1070V */
    { -400, 1395 },  /*  -40℃ -> R= 842.71Ω V_f=1.7986V V_dsp=1.1241V */
    { -350, 1416 },  /*  -35℃ -> R= 862.48Ω V_f=1.8254V V_dsp=1.1409V */
    { -300, 1436 },  /*  -30℃ -> R= 882.22Ω V_f=1.8517V V_dsp=1.1573V */
    { -250, 1456 },  /*  -25℃ -> R= 901.92Ω V_f=1.8775V V_dsp=1.1734V */
    { -200, 1476 },  /*  -20℃ -> R= 921.60Ω V_f=1.9029V V_dsp=1.1893V */
    { -150, 1495 },  /*  -15℃ -> R= 941.24Ω V_f=1.9278V V_dsp=1.2049V */
    { -100, 1514 },  /*  -10℃ -> R= 960.86Ω V_f=1.9523V V_dsp=1.2202V */
    {  -50, 1533 },  /*   -5℃ -> R= 980.44Ω V_f=1.9763V V_dsp=1.2352V */
    {    0, 1551 },  /*    0℃ -> R=1000.00Ω V_f=2.0000V V_dsp=1.2500V */
    {   50, 1569 },  /*    5℃ -> R=1019.53Ω V_f=2.0233V V_dsp=1.2645V */
    {  100, 1587 },  /*   10℃ -> R=1039.03Ω V_f=2.0461V V_dsp=1.2788V */
    {  150, 1604 },  /*   15℃ -> R=1058.49Ω V_f=2.0686V V_dsp=1.2929V */
    {  200, 1621 },  /*   20℃ -> R=1077.94Ω V_f=2.0907V V_dsp=1.3067V */
    {  250, 1638 },  /*   25℃ -> R=1097.35Ω V_f=2.1124V V_dsp=1.3203V */
    {  300, 1655 },  /*   30℃ -> R=1116.73Ω V_f=2.1338V V_dsp=1.3336V */
    {  350, 1671 },  /*   35℃ -> R=1136.08Ω V_f=2.1549V V_dsp=1.3468V */
    {  400, 1687 },  /*   40℃ -> R=1155.41Ω V_f=2.1756V V_dsp=1.3597V */
    {  450, 1703 },  /*   45℃ -> R=1174.70Ω V_f=2.1959V V_dsp=1.3725V */
    {  500, 1719 },  /*   50℃ -> R=1193.97Ω V_f=2.2160V V_dsp=1.3850V */
    {  550, 1734 },  /*   55℃ -> R=1213.21Ω V_f=2.2357V V_dsp=1.3973V */
    {  600, 1749 },  /*   60℃ -> R=1232.42Ω V_f=2.2552V V_dsp=1.4095V */
    {  650, 1764 },  /*   65℃ -> R=1251.60Ω V_f=2.2743V V_dsp=1.4214V */
    {  700, 1778 },  /*   70℃ -> R=1270.75Ω V_f=2.2932V V_dsp=1.4332V */
    {  750, 1793 },  /*   75℃ -> R=1289.87Ω V_f=2.3117V V_dsp=1.4448V */
    {  800, 1807 },  /*   80℃ -> R=1308.97Ω V_f=2.3300V V_dsp=1.4562V */
    {  850, 1821 },  /*   85℃ -> R=1328.03Ω V_f=2.3480V V_dsp=1.4675V */
    {  900, 1835 },  /*   90℃ -> R=1347.07Ω V_f=2.3657V V_dsp=1.4786V */
    {  950, 1848 },  /*   95℃ -> R=1366.08Ω V_f=2.3832V V_dsp=1.4895V */
    { 1000, 1862 },  /*  100℃ -> R=1385.06Ω V_f=2.4004V V_dsp=1.5003V */
    { 1050, 1875 },  /*  105℃ -> R=1404.00Ω V_f=2.4174V V_dsp=1.5108V */
    { 1100, 1888 },  /*  110℃ -> R=1422.93Ω V_f=2.4341V V_dsp=1.5213V */
    { 1150, 1901 },  /*  115℃ -> R=1441.82Ω V_f=2.4506V V_dsp=1.5316V */
    { 1200, 1913 },  /*  120℃ -> R=1460.68Ω V_f=2.4668V V_dsp=1.5417V */
    { 1250, 1926 },  /*  125℃ -> R=1479.51Ω V_f=2.4828V V_dsp=1.5518V */
    { 1300, 1938 },  /*  130℃ -> R=1498.32Ω V_f=2.4986V V_dsp=1.5616V */
    { 1350, 1950 },  /*  135℃ -> R=1517.10Ω V_f=2.5142V V_dsp=1.5714V */
    { 1400, 1962 },  /*  140℃ -> R=1535.84Ω V_f=2.5295V V_dsp=1.5809V */
    { 1450, 1974 },  /*  145℃ -> R=1554.56Ω V_f=2.5447V V_dsp=1.5904V */
    { 1500, 1985 },  /*  150℃ -> R=1573.25Ω V_f=2.5596V V_dsp=1.5997V */
    { 1550, 1997 },  /*  155℃ -> R=1591.91Ω V_f=2.5743V V_dsp=1.6089V */
    { 1600, 2008 },  /*  160℃ -> R=1610.54Ω V_f=2.5888V V_dsp=1.6180V */
    { 1650, 2019 },  /*  165℃ -> R=1629.15Ω V_f=2.6032V V_dsp=1.6270V */
    { 1700, 2030 },  /*  170℃ -> R=1647.72Ω V_f=2.6173V V_dsp=1.6358V */
    { 1750, 2041 },  /*  175℃ -> R=1666.27Ω V_f=2.6313V V_dsp=1.6446V */
    { 1800, 2051 },  /*  180℃ -> R=1684.78Ω V_f=2.6450V V_dsp=1.6532V */
    { 1850, 2062 },  /*  185℃ -> R=1703.27Ω V_f=2.6586V V_dsp=1.6617V */
    { 1900, 2072 },  /*  190℃ -> R=1721.73Ω V_f=2.6721V V_dsp=1.6700V */
    { 1950, 2083 },  /*  195℃ -> R=1740.16Ω V_f=2.6853V V_dsp=1.6783V */
    { 2000, 2093 },  /*  200℃ -> R=1758.56Ω V_f=2.6984V V_dsp=1.6865V */
};
static const TempLookupTable_t g_pt1000_jyx = {
    .table    = g_pt1000_jyx_table,
    .size     = sizeof(g_pt1000_jyx_table) / sizeof(g_pt1000_jyx_table[0]),
    .temp_min = -500,   /* -50.0°C */
    .temp_max = 2000,   /* 200.0°C */
};
#endif
/**
 * @brief 线性插值计算温度
 * @param lut   查表指针
 * @param adc   当前ADC采样值
 * @return      温度值, 单位0.1°C; 超范围返回边界值
 *
 * @note 查表按ADC降序排列(温度升序), 使用二分查找定位区间
 */
static int16_t NTC_LookupTemp(const TempLookupTable_t *lut, uint16_t adc)
{
    const TempLookupNode_t *tbl = lut->table;
    uint16_t n = lut->size;
    uint16_t low = 0, high = n - 1, mid;

    /* ---- 方向自适应: NTC(上拉接法) ADC 随温度降; PT 类(正温度系数) ADC 随温度升 ---- */
    if (tbl[n - 1].adc_raw > tbl[0].adc_raw) {
        /* PT 类: ADC 升序表, adc 大 -> 温度高 */
        if (adc <= tbl[0].adc_raw)  { return lut->temp_min; }
        if (adc >= tbl[n - 1].adc_raw) { return lut->temp_max; }
        while (high - low > 1) {
            mid = (low + high) >> 1;
            if (adc > tbl[mid].adc_raw) { low = mid; }
            else                        { high = mid; }
        }
        {
            int16_t  t0  = tbl[low].temp,  t1 = tbl[high].temp;
            uint16_t adc0 = tbl[low].adc_raw, adc1 = tbl[high].adc_raw;   /* adc1 > adc0 */
            int32_t  temp = (int32_t)t0 + (((int32_t)t1 - t0) * ((int32_t)adc - adc0)) / ((int32_t)adc1 - adc0);
            return (int16_t)temp;
        }
    }

    /* ---- 越界保护 (NTC 降序表) ---- */
    if (adc >= tbl[0].adc_raw) {
        return lut->temp_min;       /* ADC太大 -> 温度低于最小值 */
    }
    if (adc <= tbl[n - 1].adc_raw) {
        return lut->temp_max;       /* ADC太小 -> 温度高于最大值 */
    }

    /* ---- 二分查找: 找 adc 所在的区间 [tbl[mid].adc_raw, tbl[mid+1].adc_raw) ---- */
    while (high - low > 1) {
        mid = (low + high) >> 1;
        if (adc > tbl[mid].adc_raw) {
            high = mid;             /* ADC偏大 -> 往低温方向(数组左边) */
        } else {
            low = mid;              /* ADC偏小 -> 往高温方向(数组右边) */
        }
    }

    /* ---- 线性插值 ----
     * tbl[low]  : (T0, ADC0)  低温点, ADC较大
     * tbl[high] : (T1, ADC1)  高温点, ADC较小
     *
     * T = T0 + (T1 - T0) * (ADC - ADC0) / (ADC1 - ADC0)
     *   其中 ADC0 >= adc >= ADC1, 所以分母(ADC1 - ADC0) < 0
     *   改用正值计算避免符号问题:
     * T = T0 - (T1 - T0) * (ADC0 - adc) / (ADC0 - ADC1)
     */
    {
        int16_t  t0  = tbl[low].temp;           /* 0.1°C */
        int16_t  t1  = tbl[high].temp;          /* 0.1°C */
        uint16_t adc0 = tbl[low].adc_raw;       /* ADC0 > ADC1 */
        uint16_t adc1 = tbl[high].adc_raw;
        int32_t  delta_t = (int32_t)t1 - (int32_t)t0;       /* > 0 */
        uint16_t delta_adc = adc0 - adc1;                    /* > 0 */
        uint16_t adc_offset = adc0 - adc;                    /* >= 0 */

        /* temp = t0 + delta_t * adc_offset / delta_adc */
        int32_t temp = (int32_t)t0 + (delta_t * (int32_t)adc_offset) / (int32_t)delta_adc;

        return (int16_t)temp;
    }
}
/* **************************************** */
/*             对外接口                     */
/* **************************************** */

/**
 * @brief 获取指定传感器的量程信息
 * @param sensor_id  0=B3450, 1=ZHNTC_QSF, 2=PT1000金宇星
 * @param temp_min   输出: 最小温度(0.1°C)
 * @param temp_max   输出: 最大温度(0.1°C)
 */
void NTC_GetRange(uint8_t sensor_id, int16_t *temp_min, int16_t *temp_max)
{
    const TempLookupTable_t *lut;
    if (sensor_id == 0) {
        lut = &g_ntc_b3450;
    } else if (sensor_id == 2) {
        lut = &g_pt1000_jyx;
    } else {
        lut = &g_ntc_zhntc_qsf;
    }
    if (temp_min) *temp_min = lut->temp_min;
    if (temp_max) *temp_max = lut->temp_max;
}
int16_t NTC_GetTemp(uint8_t sensor_type,uint16_t adc_raw)
{
		switch (sensor_type) {
			case 0:  return 0;  /* KTY84-130 */
			case 1:  return 0;  /* PT100 */
			case 2:  return  NTC_LookupTemp(&g_pt1000_jyx, adc_raw);/* 金宇星PT1000(ADC直查) */
			case 3:  return 0;  /* KTY83 */
			case 4:  return  NTC_LookupTemp(&g_ntc_b3450, adc_raw);/* NTC-10K: Steinhart-Hart */
			case 5:  return  NTC_LookupTemp(&g_ntc_zhntc_qsf, adc_raw);/* NTC-10K: Steinhart-Hart */
			default: return 0.0f;
    }
}
/* 一阶低通: y = a*x + (1-a)*y_prev */
static inline float temp_lpf(float y_prev, float x, float a)
{
    return (a * x) + ((1.0f - a) * y_prev);
}
void Temp_Sensor_Update(void)
{
		static float temp_lsat1[2]={0,0};
		int16_t temp_buf[2];
    /* DMA 后台 1kHz 刷新 g_temp_adc, 这里只需读取快照。
       单个 uint16_t 读取本身原子, 温度变化慢, 两通道差一拍(1ms)可忽略, 无需关中断。 */
    /* 原始 ADC 值 (供故障检测) */
    DW.adc_slmple.mototr_Temp = g_temp_adc[0]; /* rank1 = CH6 = PC0 = 电机温度 */
    DW.adc_slmple.MOS_Temp    =  g_temp_adc[1];/* rank2 = CH7 = PC1 = MOS温度  */

    /* ADC -> 电压值*/
    DW.motor_anal.motor_temp_Rs = DW.adc_slmple.mototr_Temp*CONVERT_ADC_CONST;
    DW.motor_anal.mos_temp_Rs   =DW.adc_slmple.MOS_Temp*CONVERT_ADC_CONST;

    /* ADC值 -> 温度 -> 低通浮点输出(按传感器类型查表/NTC公式) */
		/* 传感器类型参数驱动: 2=金宇星PT1000 4=NTC-B3450 5=ZHNTC-QSF;
		   越界/未配置回默认(电机5/MOS4), 与原硬编码行为一致 */
		{
			uint8_t t_mot = DW.motor_param.temp_sensor;
			uint8_t t_mos = DW.comm_contrl.mos_temp_sensor;
			if ((t_mot != 2U) && (t_mot != 4U) && (t_mot != 5U)) { t_mot = 5U; }
			if ((t_mos != 2U) && (t_mos != 4U) && (t_mos != 5U)) { t_mos = 4U; }
			temp_buf[0]=NTC_GetTemp(t_mot,DW.adc_slmple.mototr_Temp);
			temp_buf[1]=NTC_GetTemp(t_mos,DW.adc_slmple.MOS_Temp);
		}
		
//		DW.motor_anal.motor_temp=temp_buf[0]*0.1;
//		DW.motor_anal.mos_temp=temp_buf[1]*0.1;
    DW.motor_anal.motor_temp = temp_lpf(temp_lsat1[0],((float)temp_buf[0]*0.1),TEMP_LPF_ALPHA);
    DW.motor_anal.mos_temp   = temp_lpf(temp_lsat1[1],((float)temp_buf[1]*0.1),TEMP_LPF_ALPHA);
		temp_lsat1[0]=DW.motor_anal.motor_temp;
		temp_lsat1[1]=DW.motor_anal.mos_temp;
}

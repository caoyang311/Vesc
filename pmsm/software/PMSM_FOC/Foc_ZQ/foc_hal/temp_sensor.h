/******************************************************************************
 * 文件: temp_sensor.h
 * 功能: 温度传感器电阻值 -> 摄氏温度 转换接口
 * 创建日期: 2026-06-22
 * 修改日期: 2026-06-23
 * 修改内容: 新增 Temp_Sensor_Update: 主循环采集 ADC1 双路温度(无DMA, TIM4触发)
 * 修改人: rui.zhang
 ******************************************************************************/

#ifndef TEMP_SENSOR_H_
#define TEMP_SENSOR_H_

#include <stdint.h>
#define TEMP_CALCULATE_MODE 1 //0:RS计算方法 1：电压直接查表方法
/* 温度转换输出一阶低通: y = a*x + (1-a)*y_prev
   Temp_Sensor_Update 按 1ms 调用; a=0.05 -> τ≈20ms (约 60ms 到 95%)
   温度保护确认时间 200ms 量级, 20ms 滤波不改变保护时序。 */
#define TEMP_LPF_ALPHA 0.05f
#if(TEMP_CALCULATE_MODE==0)

#define UP_RES_CONSTANT1 (560.0f*3.3f)  //电阻常数 = 上拉电阻*ADC参考电压
#define ADC_CONSTANT1 	 (4095.0f*5.0f) //ADC常数

#elif(TEMP_CALCULATE_MODE==1)


#endif
extern volatile uint16_t g_temp_adc[2];
/* 分度表法读取温度: S_type 分度号(100=PT100 1000=PT1000 84=KTY84-130), Res_in 电阻(Ω); 返回℃, 不支持返回0 */
float Read_Temp_fun(int S_type, float Res_in);

/* NTC 电阻值 -> 温度 (Steinhart-Hart): R_ntc 电阻(Ω); 返回℃ */
float Temp_NTC_FromRes(float R_ntc);

/* 按传感器类型(0:KTY84 1:PT100 2:PT1000 3:KTY83 4:NTC-10K)由电阻 Rs(Ω) 换算温度(℃) */
float Temp_FromResistance(uint8_t sensor_type, float Rs);

/* 主循环温度采集任务: 读 ADC1 两路(PC0电机/PC1控制器, TIM4触发, 无DMA)
 * -> 转采样电阻 -> 查表得温度, 结果写入 DW.motor_anal。在 while(1) 里周期调用。 */
void Temp_Sensor_Update(void);

#endif /* TEMP_SENSOR_H_ */

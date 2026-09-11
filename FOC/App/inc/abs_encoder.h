/**
 * *****************************************************************************
 * @file        abs_encoder.h
 * @brief       绝对值编码器模块
 * @author      caoyang
 * @date        2023-11-13
 * @copyright   欢阳技有限公司
 * *****************************************************************************
 */
#ifndef ABS_ENCODER_H
#define ABS_ENCODER_H
/*----------------------------------include-----------------------------------*/
#include "stm32f4xx.h"
#include "motor_pmsm.h"
#include "uart.h"
/*-----------------------------------macro------------------------------------*/
//编码器相关参数定义
#define ENCODER_MAX_VALUE                      (131071)                     /*17位绝对值编码器最大计数值 */
#define ENC_AVERAGING_FIFO_DEPTH               (6)                          //滑动平均滤波窗口大小
#define ENC_SPEED_ARRAY_SIZE                   (ENC_AVERAGING_FIFO_DEPTH+1) //编码器值缓存buffer大小
#define VALID_VALUE             0x55
#define INVALID_VALUE           0xAA
//电机方向判断相关宏定义
#define UNKOWN                  0x00           //方向没确定
#define CW                      0x01           //逆时针（正转）
#define CCW                     0x02           //顺时针（反转）
#define DIRECTION_TIME          (u16)(10)            //方向判断预估时间：10ms
#define ENCODER_CYCLE           (u16)(2000)           //编码器任务执行周期：2000HZ
#define DIRECTION_CNT           (u32)(DIRECTION_TIME*ENCODER_CYCLE)
//编码器指令
#define CF_DATA_ID_0            0x02   //Data readout
#define CF_DATA_ID_1            0x8A   //Data readout
#define CF_DATA_ID_2            0x92   //Data readout
#define CF_DATA_ID_3            0x1A   //Data readout

#define CF_DATA_ID_6            0x32   //writing to eeprom
#define CF_DATA_ID_D            0xEA   //raedout from eeprom

#define CF_DATA_ID_7            0xBA   //Reset
#define CF_DATA_ID_8            0xC2   //Reset
#define CF_DATA_ID_C            0x62   //Reset
//编码器值换算电角度
#define FACTOR                (float)((TWOPI * MOTOR_PP) / ENCODER_MAX_VALUE) //每一个编码器值对应的弧度值
#define TWOPI_CODER           (u16)(ENCODER_MAX_VALUE / MOTOR_PP) //0~2pi 对应编码器 0~26214
/*----------------------------------typedef-----------------------------------*/
typedef struct
{
  float     EAngle;           //电角度
  uint32_t  Encoder_Value;    //编码器值
  float   AvrMecSpeed;      //平均机械转速
  uint32_t	Direction;        //电机方向
  uint32_t	InitPhaseShift;   //零点偏移
  uint8_t   Valid_Flag;       //零点校准标志位
  int32_t  Previous_Capture; //上一次编码器值
  int32_t	Current_Capture;  //当前编码器值
  int32_t  DeltaCapturesBuffer[ENC_SPEED_ARRAY_SIZE];//滑动窗口缓存 
  uint8_t   Buff_Full_Flag;  // 缓存buffer满标志
  uint8_t   Buff_Counter;    //缓存计数
  int32_t		OverallAngleVariation;//窗口总计数
  int32_t 	ENC_Period_Avg;   //平均脉冲计数  
} ABS_ENCODER;
/*----------------------------------variable----------------------------------*/
extern ABS_ENCODER ABS_Encoder;
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void ABS_Encoder_DMA_DataProcess(u8 *txdata,u8 *rxdata,u16 size);
void ABS_Encoder_DMA_SendData(u8 id,u16 size);
float ABS_Encoder_CalcAngle(void);
void ABS_Encoder_Calibration(void);
void ABS_Encoder_Init(void);
void ABS_Encoder_Reset(void);
void ABS_Encoder_CalcAvrgMecSpeed(void);
/*------------------------------------test------------------------------------*/
#endif


/**
 * *****************************************************************************
 * @file        encoder.h
 * @brief       编码器模块
 * @author      caoyang
 * @date        2023-11-13
 * @copyright   欢阳技有限公司
 * *****************************************************************************
 */
#ifndef ABZ_ENCODER_H
#define ABZ_ENCODER_H
/*----------------------------------include-----------------------------------*/
#include "stm32f4xx.h"
#include "motor_pmsm.h"
#include "timer.h"
/*-----------------------------------macro------------------------------------*/
//编码器定时器相关参数定义
#define M1_ENCODER_PPR             (1000u)                     /*编码器线束 */
#define M1_PULSE_NBR               ((4u * (M1_ENCODER_PPR))-1) /*重装载值 */
#define M1_ENC_IC_FILTER            (12u) 
//编码器速度缓存长度
#define ENC_AVERAGING_FIFO_DEPTH        (6) 
#define ENC_SPEED_ARRAY_SIZE  (ENC_AVERAGING_FIFO_DEPTH+1)    
#define ENC_MAX_OVERFLOW_NB     (1)
//角度偏移
#define S16_180_PHASE_SHIFT  (0xffff/2)
#define S16_60_PHASE_SHIFT   (0xffff/6)
#define S16_90_PHASE_SHIFT   (0xffff/4)
//计数方向
#define TIM_COUNTERDIRECTION_UP             (0x00000000U)          /*!< Timer counter counts up */
#define TIM_COUNTERDIRECTION_DOWN           (TIM_CR1_DIR)          /*!< Timer counter counts down */
//速度环的执行频率
#define	MEDIUM_FREQUENCY_TASK_RATE		(2000.0f) //2Khz

/*----------------------------------typedef-----------------------------------*/
typedef struct
{
  float EAngle;               //电角度
  float hAvrMecSpeed;         //平均机械转速
  float PulseNumber;         //编码器一圈的脉冲数 
  uint8_t SpeedBufferSize;      //滑动窗口大小   
  volatile uint16_t TimerOverflowNb; //脉冲计数溢出计数   
  uint16_t PreviousCapture;     //上一次脉冲计数值             
  int32_t DeltaCapturesBuffer[ENC_SPEED_ARRAY_SIZE];//滑动窗口缓存 	
  int32_t 	ENC_Period_Avg;   //平均脉冲计数
  int32_t		wOverallAngleVariation;//窗口总计数
  uint32_t	Current_CntCapture;//  当前捕获的脉冲数
  uint32_t	directionSample;   //电机方向
  int16_t	  InitPhaseShift;    //零点偏移
} ABZ_ENCODER_Handle_t;
/*----------------------------------variable----------------------------------*/
extern ABZ_ENCODER_Handle_t ABZ_Encoder;
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void ABZ_Encoder_Init(void);
uint32_t TIM_GetDirection(TIM_TypeDef *TIMx);
float ABZ_Encoder_CalcAngle( void );
void ABZ_Encoder_CalcAvrgMecSpeed(void);
void ABZ_Encoder_Start_Up(void);
void Reset_ABZ_Encoder(void);
int16_t ABZ_Encoder_Zero_Calibration(void);
/*------------------------------------test------------------------------------*/
#endif


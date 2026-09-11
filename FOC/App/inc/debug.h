/**
 * *****************************************************************************
 * @file        debug.h
 * @brief       波形调试模块
 * @author      caoyang
 * @date        2023-11-13
 * @copyright   欢阳技有限公司
 * *****************************************************************************
 */
#ifndef DEBUG_H
#define DEBUG_H
/*----------------------------------include-----------------------------------*/
#include "stm32f4xx.h"
/*-----------------------------------macro------------------------------------*/
#define WAVE_BUFF_MAX (16U)
/*----------------------------------typedef-----------------------------------*/
typedef union {
    float    float_data;
	u8       char_table[4];
} Union_Typedef;

typedef struct Rec_Para_Typedef{
    float speed_ref;
    float speed_kp;
    float speed_ki;
} Rec_Para;
/*----------------------------------variable----------------------------------*/
extern Union_Typedef send_wave[WAVE_BUFF_MAX];
extern Union_Typedef rece_arry[2];
extern uint8_t rece_flag;
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void Delay_Us(u32 us);
void Uart_SendWave(float data1,float data2,float data3);
void Debug_Task(void);
/*------------------------------------test------------------------------------*/
#endif





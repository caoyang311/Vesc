/**
 * *****************************************************************************
 * @file        key.h
 * @brief       按键模块
 * @author      caoyang
 * @date        2023-11-13
 * @copyright   欢阳技有限公司
 * *****************************************************************************
 */

#ifndef KEY_H
#define KEY_H
/*----------------------------------include-----------------------------------*/
#include "stdint.h" 
/*-----------------------------------macro------------------------------------*/
#define LONG_PRESS_TIME 	200 
#define CLICK_MIN_TIME 		5	/* if key press_cnt time less than this -> invalid click */
#define CLICK_MAX_TIME 		50	/* if key press_cnt time more than this -> invalid click */
/*----------------------------------typedef-----------------------------------*/
typedef enum
{
	KEY_NULL,
	KEY_DOWN,
	KEY_PRESS,
	KEY_UP,
}KeyActionType;
 
typedef enum
{
	BUTTON_NULL,
	BUTTON_SINGLE,
	BUTTON_LONG_PRESS,
}ButtonActionType;
 
typedef struct 
{
	GPIO_TypeDef * GPIO_Port;		//按键端口
	uint16_t GPIO_Pin;				//按键PIN
	KeyActionType key;				//按键类型
	uint16_t hold_cnt;				//按压计数器
	uint8_t press_flag;				//按压标志
	uint8_t release_flag;			//松手标志
	ButtonActionType buttonAction;	//按键键值
}buttonType;
/*----------------------------------variable----------------------------------*/
extern buttonType button[1];
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void Key_Config(void);
void Key_Task(void);
/*------------------------------------test------------------------------------*/
#endif


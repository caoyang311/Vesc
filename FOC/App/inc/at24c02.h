/**
 * *****************************************************************************
 * @file        AT24C02.h
 * @brief       AT24C02模块
 * @author      caoyang
 * @date        2023-11-13
 * @copyright   欢阳技有限公司
 * *****************************************************************************
 */
#ifndef AT24C02_H
#define AT24C02_H
/*----------------------------------include-----------------------------------*/
#include "stm32f4xx.h"
/*-----------------------------------macro------------------------------------*/
#define I2C_PageSize  (8u)
/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void IIC_Write_AT24C02_OneByte(uint8_t addr,uint8_t data);
void IIC_Waite_AT24C02_Standby(void);
void IIC_AT24C02_PageWrite(uint8_t *pBuffer,uint8_t WriteAddr,uint8_t NumByte);
uint8_t IIC_Read_AT24C02_OneByte(uint8_t address);
void IIC_AT24C02_PageRead(uint8_t *pBuffer,uint8_t ReadAddr,uint8_t NumByte);
void IIC_AT24C02_BufferWrite(uint8_t *pBuffer,uint8_t WriteAddr,uint8_t NumByteToWrite );
void ATC24C02_Test(void);
/*------------------------------------test------------------------------------*/
#endif





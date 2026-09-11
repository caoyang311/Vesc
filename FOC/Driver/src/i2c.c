#include "i2c.h"
/**
 * *****************************************************************************
 * @file        i2c.c
 * @brief       硬件IIC模块
 * @author       (caoyang)
 * @date        2023-11-13
 * @copyright   欢阳技有限公司
 * *****************************************************************************
 */

void iic_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;  //定义结构体

    I2C_InitTypeDef I2C_InitStructure;  
 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);   //打开IIC2外设时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);   //打开GPIOH口的时钟I2C_CLK--PH4

    //IIC端口设置 IIC_CLK和IIC_SDA引脚均要设置为复用开漏不带上拉输出
    GPIO_PinAFConfig(I2C2_SCL_GPIO_Port,GPIO_PinSource4,GPIO_AF_I2C2);  //开启PH4的复用功能连接至I2C2
    GPIO_PinAFConfig(I2C2_SDA_GPIO_Port,GPIO_PinSource5,GPIO_AF_I2C2);  //开启PH5的复用功能连接至I2C2

    GPIO_InitStructure.GPIO_Pin=I2C2_SCL_PIN | I2C2_SDA_PIN;   //选择PH4和PH5引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           //开启PH4和PH5的复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;    //PH4和PH5设置为开漏输出
    GPIO_InitStructure.GPIO_PuPd =GPIO_PuPd_NOPULL;   //PH4和PH5口不带上拉电阻
    GPIO_Init(I2C2_SCL_GPIO_Port, &GPIO_InitStructure);

    I2C_DeInit(I2C2); //将外设IIC的各个寄存器恢复到复位以后的值
    I2C_InitStructure.I2C_ClockSpeed=100000;  //标准模式 时钟频率为100KHZ
    I2C_InitStructure.I2C_Mode=I2C_Mode_I2C;   //选中I2C模式
    I2C_InitStructure.I2C_DutyCycle=I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1=0X00;      //当I2C出于从模式时,自身的地址
    I2C_InitStructure.I2C_Ack=I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C2, &I2C_InitStructure); 

    I2C_AcknowledgeConfig(I2C2,ENABLE);   //在接收到一个字节后返回一个应答ACK
    I2C_Cmd(I2C2,ENABLE);                 //开启外设IIC模块  
}


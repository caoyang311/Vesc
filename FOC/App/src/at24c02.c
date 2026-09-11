
/**
 * *****************************************************************************
 * @file        AT24C02.c
 * @brief       AT24C02 模块
 * @author       (caoyang)
 * @date        2023-11-13
 * @copyright   欢阳技有限公司
 * *****************************************************************************
 */

#include "at24c02.h"
#include "i2c.h"

/**
  * @brief   This function write one byte to at24c02.
  * @param  None
  * @retval None
  */
void IIC_Write_AT24C02_OneByte(uint8_t addr,uint8_t data)
{
 volatile uint16_t flag;
//判断BUSY是否为1,BUSY=1总线处于忙状态
 while(I2C_GetFlagStatus(I2C2,I2C_FLAG_BUSY));  //如果BUSY=1则等待一直等到BUSY=0
 
//主机产生起始条件
 I2C_GenerateSTART(I2C2,ENABLE); 
 
//判断SB是否为1,SB=1起始条件已经发送 即判断EV5
 while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT)==ERROR);  //当产生了起始条件,设备就进入了主模式
 
//发送eepROM的地址0XA0
 I2C_Send7bitAddress(I2C2,0XA0,I2C_Direction_Transmitter); //软件读取SR1寄存器后,写数据寄存器的操作将清除SB位
//判断ADDR是否为1,ADDR=1地址发送结束 即判断EV6
 while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==ERROR);
  flag=I2C2->SR2; //软件读取SR1寄存器后,对SR2寄存器的读操作将清除ADDR位
//发送AT24C02的内部地址:即数据要写入的地址
 I2C_SendData(I2C2,addr); 
 
//判断TXE是否为1,TXE=1移位寄存器不为空,数据寄存器为空 即判断EV8
 while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTING)==ERROR);
 
//发送数据到要写入的地址
 I2C_SendData(I2C2,data);    //对数据寄存器的写操作将清除TXE位
 
//判断TXE和BTF是否为1,TXE=1,BTF=1请求设置停止位 即判断EV8_2
 while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED)==ERROR);
 
 //关闭通讯,产生停止条件
 I2C_GenerateSTOP(I2C2,ENABLE); //产生停止条件时,硬件自动清除TXE和BTF位
}

/**
  * @brief   This function waite for eeprom standby state.
  * @param   none
 * @retval  None
  */
/*AT24C02在接收完数据后,启动内部周期写入数据的时间内不会对主机的请求做出应答的特性,因此此
  函数循环发送起始信号,若检测到AT24C02的应答,则说明AT24C02已完成上一步的数据写入,进入稳定状态
  可以进行下一次的操作,因此这个函数在数据写入完成后必须调用此函数,判断AT24C02是否进入稳定状态*/
void IIC_Waite_AT24C02_Standby(void)
{
  volatile uint16_t SR1_Flag;
 
    I2C2->SR1=0X0000;  //清除I2C2_SR1寄存器
    do
    {
    I2C_GenerateSTART(I2C2,ENABLE);  //产生起始条件
    
    SR1_Flag=I2C2->SR1;  //软件读取SR1寄存器后,写数据寄存器的操作将清除SB位
    
    I2C_Send7bitAddress(I2C2,0XA0,I2C_Direction_Transmitter); //发送7位AT24C02的地址
    }while((I2C2->SR1&0X0002)==0X00);  //当ADDR=1时,跳出此循环表明AT24C02写入数据已完成
    
    I2C_ClearFlag(I2C2,I2C_FLAG_AF);  //清除应答失败位
    
    I2C_GenerateSTOP(I2C2,ENABLE);   //发送停止信号
}

/*
 * 函数名：IIC_AT24C02_PageWrite
 * 描述  ：在EEPROM的一个写循环中可以写多个字节，但一次写入的字节数
 *         不能超过EEPROM页的大小。AT24C02每页有8个字节。
 * 输入  ：-pBuffer 缓冲区指针
 *         -WriteAddr 接收数据的EEPROM的地址
 *         -NumByte 要写入EEPROM的字节数
 * 输出  ：无
 * 返回  ：无
 * 调用  ：外部调用
 */
void IIC_AT24C02_PageWrite(uint8_t *pBuffer,uint8_t WriteAddr,uint8_t NumByte)
{
 volatile uint16_t state;
//判断BUSY是否为1,BUSY=1总线处于忙状态
 while(I2C_GetFlagStatus(I2C2,I2C_FLAG_BUSY));  //如果BUSY=1则等待一直等到BUSY=0
 
//主机产生起始条件
 I2C_GenerateSTART(I2C2,ENABLE); 
 
//判断SB是否为1,SB=1起始条件已经发送 即判断EV5
 while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT)==ERROR);  //当产生了起始条件,设备就进入了主模式
 
//发送eepROM的地址0XA0
 I2C_Send7bitAddress(I2C2,0XA0,I2C_Direction_Transmitter); //软件读取SR1寄存器后,写数据寄存器的操作将清除SB位
//判断ADDR是否为1,ADDR=1地址发送结束 即判断EV6
 while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==ERROR);
  state=I2C2->SR2; //软件读取SR1寄存器后,对SR2寄存器的读操作将清除ADDR位
 
//发送AT24C02的内部地址:即数据要写入的地址
 I2C_SendData(I2C2,WriteAddr); 
 
//判断TXE是否为1,TXE=1移位寄存器不为空,数据寄存器为空 即判断EV8
 while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTING)==ERROR);
 
 for(;NumByte>0;NumByte--)
 {
  //发送数据到要写入的地址
   I2C_SendData(I2C2,*pBuffer);    //对数据寄存器的写操作将清除TXE位
 
  //缓冲区指针加1指向下一个要写入的数据
  pBuffer++;
 
  //判断TXE和BTF是否为1,TXE=1,BTF=1请求设置停止位 即判断EV8_2
   while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED)==ERROR);
 }
 
 //关闭通讯,产生停止条件
 I2C_GenerateSTOP(I2C2,ENABLE); //产生停止条件时,硬件自动清除TXE和BTF位
}

/**
  * @brief   This function read one byte from slect address.
  * @param   none
 * @retval  None
  */
/*选择性读取允许主器件对AT24C02寄存器的任意字节读操作,具体步骤如下:
1--主器件发送起始信号,从器件地址,和他想读取的字节数据的地址执行一个伪写操作
2--AT24C02应答之后,主器件从新发送起始信号和从器件地址,此时R/W位置1。
3--AT24C02响应并发送应答信号,然后输出一个所要求的8位字节数据。
4--主器件不发送应答信号,但产生一个停止信号。*/
uint8_t IIC_Read_AT24C02_OneByte(uint8_t address)
{
volatile  uint16_t temp,flag;
//判断BUSY是否为1,BUSY=1总线处于忙状态
 while(I2C_GetFlagStatus(I2C2,I2C_FLAG_BUSY));  //如果BUSY=1则等待一直等到BUSY=0
 
//主机产生起始条件
 I2C_GenerateSTART(I2C2,ENABLE); 
 
//判断SB是否为1,SB=1起始条件已经发送 即判断EV5
 while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT)==ERROR);  //当产生了起始条件,设备就进入了主模式
 
//发送eepROM的地址0XA0
 I2C_Send7bitAddress(I2C2,0XA0,I2C_Direction_Transmitter); //软件读取SR1寄存器后,写数据寄存器的操作将清除SB位
//判断ADDR是否为1,ADDR=1地址发送结束 即判断EV6
 while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==ERROR);
  flag=I2C2->SR2; //软件读取SR1寄存器后,对SR2寄存器的读操作将清除ADDR位
//发送想要读取字节的地址
 I2C_SendData(I2C2,address); 
//判断TXE是否为1,TXE=1移位寄存器不为空,数据寄存器为空 即判断EV8
 while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTING)==ERROR);
 
//主机重新发送起始信号
 I2C_GenerateSTART(I2C2,ENABLE); 
 
//判断SB是否为1,SB=1起始条件已经发送 即判断EV5
 while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT)==ERROR);  //当产生了起始条件,设备就进入了主模式
//主机重新发送AT24C02的地址,这是R/W位置1
  I2C_Send7bitAddress(I2C2,0XA0,I2C_Direction_Receiver);
 
//判断ADDR是否为1,ADDR=1地址发送结束 即判断EV6
 while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)==ERROR);
  flag=I2C2->SR2; //软件读取SR1寄存器后,对SR2寄存器的读操作将清除ADDR位
 I2C_AcknowledgeConfig(I2C2,DISABLE);
 I2C_GenerateSTOP(I2C2,ENABLE);
 while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_RECEIVED)==ERROR);
 
 temp=I2C_ReceiveData(I2C2);
 
 I2C_AcknowledgeConfig(I2C2, ENABLE); //打开应答信号使其回到初始状态
 return temp;
}

/*
 * 函数名：IIC_AT24C02_PageRead
 * 描述  ：从EEPROM里面读取一块数据。
 * 输入  ：-pBuffer 存放从EEPROM读取的数据的缓冲区指针。
 *         -ReadAddr 接收数据的EEPROM的地址。
 *         -NumByte 要从EEPROM读取的字节数。
 * 输出  ：无
 * 返回  ：无
 * 调用  ：外部调用
 */
void IIC_AT24C02_PageRead(uint8_t *pBuffer,uint8_t ReadAddr,uint8_t NumByte)
{
 volatile uint16_t  flag1,flag2;
 
//判断BUSY是否为1,BUSY=1总线处于忙状态
 while(I2C_GetFlagStatus(I2C2,I2C_FLAG_BUSY));  //如果BUSY=1则等待一直等到BUSY=0
 
//主机产生起始条件
 I2C_GenerateSTART(I2C2,ENABLE); 
 
//判断SB是否为1,SB=1起始条件已经发送 即判断EV5
 while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT)==ERROR);  //当产生了起始条件,设备就进入了主模式
//发送eepROM的地址0XA0
 I2C_Send7bitAddress(I2C2,0XA0,I2C_Direction_Transmitter); //软件读取SR1寄存器后,写数据寄存器的操作将清除SB位
//判断ADDR是否为1,ADDR=1地址发送结束 即判断EV6
 while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==ERROR);
  flag1=I2C2->SR2; //软件读取SR1寄存器后,对SR2寄存器的读操作将清除ADDR位
 
//发送想要读取字节的地址
 I2C_SendData(I2C2,ReadAddr); 
//判断TXE是否为1,TXE=1移位寄存器不为空,数据寄存器为空 即判断EV8
 while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTING)==ERROR);
 
//主机重新发送起始信号
 I2C_GenerateSTART(I2C2,ENABLE); 
 
//判断SB是否为1,SB=1起始条件已经发送 即判断EV5
 while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT)==ERROR);  //当产生了起始条件,设备就进入了主模式
//主机重新发送AT24C02的地址,这是R/W位置1
  I2C_Send7bitAddress(I2C2,0XA0,I2C_Direction_Receiver);
//判断ADDR是否为1,ADDR=1地址发送结束 即判断EV6
 while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)==ERROR);
  flag2=I2C2->SR2; //软件读取SR1寄存器后,对SR2寄存器的读操作将清除ADDR位
 
//接收AT24C02发送的数据
    while(NumByte)
    {
        if(NumByte==1)   //等于1时数据已经发送完成
        {
        //主机不再发送响应信号
        I2C_AcknowledgeConfig(I2C2,DISABLE);
        //主机产生停止信号
        I2C_GenerateSTOP(I2C2,ENABLE);
        }
        //检查RXNE是否为1,即判断EV7
        if(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_RECEIVED)==SUCCESS)
        {
        /* Read a byte from the EEPROM */
        *pBuffer = I2C_ReceiveData(I2C2);  //软件读取数据寄存器的值,讲清除RXNE位

        /* Point to the next location where the byte read will be saved */
            pBuffer++;

        /* Decrement the read bytes counter */
            NumByte--;  
        }
    }
 I2C_AcknowledgeConfig(I2C2, ENABLE); //打开应答信号使其回到初始状态
}

/*
 * 函数名：IIC_AT24C02_BufferWrite
 * 描述  ：将缓冲区中的数据写到I2C EEPROM中
 * 输入  ：-pBuffer 缓冲区指针
 *         -WriteAddr 接收数据的EEPROM的地址
 *         -NumByteToWrite 要写入EEPROM的字节数
 * 输出  ：无
 * 返回  ：无
 */
void IIC_AT24C02_BufferWrite(uint8_t *pBuffer,uint8_t WriteAddr,uint8_t NumByteToWrite )
{
 uint8_t NumofPage=0;    //要写入的数据块按页操作所需的次数(AT24C02页写,可一次写入8个字节数据)
 uint8_t NumofSignal=0;  //要写入的数据块按页写操作完成后剩余的字节数
 uint8_t addr=0;         //要写入数据块的首地址是否为8的倍数
 uint8_t count=0;        //

 addr=WriteAddr%I2C_PageSize; //判断要写入的数据块首地址是否为8的倍数,addr=0则为8的整数倍
//如果要写入的数据的起始地址不是8的倍数,则addr不为0.算出与8的差值。
//写入数据的起始地址+这个差值=下一个为8的倍数的地址
 
 count=8-addr;
 
 NumofPage=NumByteToWrite/I2C_PageSize; //计算写入的数据块按页写需要进行页操作的次数
 NumofSignal=NumByteToWrite%I2C_PageSize; //页操作剩余的字节数
 
/* 如果要写入数据块的首地址为8的倍数,即addr=0只需操作几次页写函数即可 */
 if(addr==0)   //首先判断起始地址是否为8的整数倍,在判断要写的数据长度是否大于8个字节
 {
  //如果写入的数据块长度小于8个字节
  if(NumofPage==0)
  {
   IIC_AT24C02_PageWrite(pBuffer,WriteAddr,NumofSignal);
   IIC_Waite_AT24C02_Standby();
  }
  else    //要写入的数据块长度大于8个字节,即按页操作
  {
   while(NumofPage--)
   {
    IIC_AT24C02_PageWrite(pBuffer,WriteAddr,8);  //进行页操作一次可以连续写入8个字节数据
    IIC_Waite_AT24C02_Standby();
    WriteAddr+=8;       //写入数据的地址加8
    pBuffer+=8;         //将要写入的数据的指针加8
   }
   if(NumofSignal!=0)  //按页操作完成之后的数据不够8个字节的
   {
    IIC_AT24C02_PageWrite(pBuffer,WriteAddr,NumofSignal);  //将剩余的字节数写入到对应的地址中
    IIC_Waite_AT24C02_Standby();
   }
  }
 }
 else   //如果写入数据的起始地址不为8的倍数,即addr！=0.
 {
  //如果写入的数据块长度小于8个字节
  if(NumofPage==0)
  {
   IIC_AT24C02_PageWrite(pBuffer,WriteAddr,NumofSignal);
   IIC_Waite_AT24C02_Standby();
  }
  else    //要写入的数据块长度大于8个字节,即按页操作
  {
   //首先要先写入count个数据,即算出写完counter个字节数据后NumByteToWrite的新值
   
   NumByteToWrite-=count;
   NumofPage=NumByteToWrite/I2C_PageSize; //计算写入的数据块按页写需要进行页操作的次数
     NumofSignal=NumByteToWrite%I2C_PageSize; //页操作剩余的字节数
   
   if(count!=0)
   {
    IIC_AT24C02_PageWrite(pBuffer,WriteAddr,count);
     IIC_Waite_AT24C02_Standby();
    WriteAddr+=count;
    pBuffer+=count;
   }
   while(NumofPage--)
   {
    IIC_AT24C02_PageWrite(pBuffer,WriteAddr,8);  //进行页操作一次可以连续写入8个字节数据
    IIC_Waite_AT24C02_Standby();
    WriteAddr+=8;       //写入数据的地址加8
    pBuffer+=8;         //将要写入的数据的指针加8
   }
   if(NumofSignal!=0)  //按页操作完成之后的数据不够8个字节的
   {
    IIC_AT24C02_PageWrite(pBuffer,WriteAddr,NumofSignal);  //将剩余的字节数写入到对应的地址中
    IIC_Waite_AT24C02_Standby();
   }
  }
 }
}

/**
 * @brief       ATC24C02测试代码
 * 
 * @param       void     
 * @return      void
 */
uint8_t I2C_Data =0;
uint8_t PageWrite[8]={1,2,3,4,5,6,7,8};
uint8_t PageRead[8];
uint8_t SequenceWrite[15]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
uint8_t SequenceRead[15];

void ATC24C02_Test(void)
{
    uint8_t *p,*q,*r,*s;
    p=PageWrite;
    q=PageRead;
    r=SequenceWrite;
    s=SequenceRead;    
    //单字节的读与写    
    IIC_Write_AT24C02_OneByte(200,0x78);
    IIC_Waite_AT24C02_Standby();
    I2C_Data=IIC_Read_AT24C02_OneByte(200);

    //页写和连续读取
    IIC_AT24C02_PageWrite(p,0,8);
    IIC_Waite_AT24C02_Standby();
    IIC_AT24C02_PageRead(q,0,8);  

    //连续写与读
    IIC_AT24C02_BufferWrite(r,16,15);
    IIC_Waite_AT24C02_Standby();
    IIC_AT24C02_PageRead(s,16,15);   
}





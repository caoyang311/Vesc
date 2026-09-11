/**
 * *****************************************************************************
 * @file        uart_1.c
 * @brief       串口1底层驱动
 * @author       (caoyang)
 * @date        2023-11-17
 * @copyright   欢阳技有限公司
 * *****************************************************************************
 */
#include "uart.h"
#include "debug.h"
#include "abs_encoder.h"
#include "gpio.h"
#include "string.h"

uint8_t USART1_TX_FLAG;
uint8_t USART3_Tx_Buf[USART3_MAX_TX_LEN];
uint8_t USART3_Rx_Buf[USART3_MAX_RX_LEN];
uint8_t USART3_Flag_Rx_Busy;
uint8_t USART3_Flag_Tx_Busy;
//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif

/**
 * @brief       uart1串口初始化
 * 
 */
void uart1_init(void)
{
	
    GPIO_InitTypeDef 	GPIO_InitStructure;
    USART_InitTypeDef 	USART_InitStructure;
    NVIC_InitTypeDef 	NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//IO时钟使能
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//串口时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE); //DMA时钟使能
    //IO复用为串口x
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1);
    
    //IO初始化
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;   //速度 25MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
    GPIO_Init(GPIOB,&GPIO_InitStructure);               //初始化
    
    USART_InitStructure.USART_BaudRate = USART1_BAUDRATE;                //波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_Parity = USART_Parity_No;      //无校验
    USART_InitStructure.USART_StopBits = USART_StopBits_1;     //1个停止位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//收发模式
    USART_Init(USART1, &USART_InitStructure);                  //初始化串口
    
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);/* Enable USARTx DMA TX request */
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);/* Enable USARTx DMA RX request */
	USART_Cmd(USART1, ENABLE);//使能串口
		
    // DMA2 数据流7 通道4配置
    DMA_DeInit(USART1_DMA_TX_STREAM);//复位DMA2_Stream7 
	while( DMA_GetCmdStatus(USART1_DMA_TX_STREAM) != DISABLE );/* Wait for the specified DMAx Streamx reset to complete. */
		
    DMA_InitStructure.DMA_Channel = USART1_TX_DMA_CHANNEL; //通道选择：通道4
	DMA_InitStructure.DMA_DIR =DMA_DIR_MemoryToPeripheral;/* Data from memory to peripheral */
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)send_wave;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	
	DMA_InitStructure.DMA_BufferSize = 16;
	
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(USART1_DMA_TX_STREAM,&DMA_InitStructure);
	
	DMA_ITConfig(USART1_DMA_TX_STREAM, DMA_IT_TC, ENABLE);
	DMA_ClearFlag(USART1_DMA_TX_STREAM,USART1_DMA_TC_FLAG);
	DMA_Cmd(USART1_DMA_TX_STREAM,ENABLE);//Enable DMA  tx   
		
    // DMA2 数据流5 通道4配置
    DMA_DeInit(USART1_DMA_RX_STREAM);//复位DMA2_Stream5 
	while( DMA_GetCmdStatus(USART1_DMA_RX_STREAM) != DISABLE );/* Wait for the specified DMAx Streamx reset to complete. */
		
    DMA_InitStructure.DMA_Channel = USART1_RX_DMA_CHANNEL; 
	DMA_InitStructure.DMA_DIR =DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rece_arry;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	
	DMA_InitStructure.DMA_BufferSize = 8;
	
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(USART1_DMA_RX_STREAM,&DMA_InitStructure);
	
	DMA_ITConfig(USART1_DMA_RX_STREAM, DMA_IT_TC, ENABLE);
	DMA_ClearFlag(USART1_DMA_RX_STREAM,USART1_DMA_RC_FLAG);
	DMA_Cmd(USART1_DMA_RX_STREAM,ENABLE);//Enable DMA  Rx   		
 
    NVIC_InitStructure.NVIC_IRQChannel = USART1_DMA_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;  //抢占优先级 2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;        //响应优先级 0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;          //IRQ 通道使能
    NVIC_Init(&NVIC_InitStructure);    //根据指定的参数初始化 VIC 寄存器
		
    NVIC_InitStructure.NVIC_IRQChannel = USART1_DMA_RX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;  //抢占优先级 2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;        //响应优先级 0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;          //IRQ 通道使能
    NVIC_Init(&NVIC_InitStructure);    //根据指定的参数初始化 VIC 寄存器		
		
    
}

/**
 * @brief       uart3串口初始化
 * 
 */
void uart3_init(void)
{
	GPIO_InitTypeDef    GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef     DMA_InitStructure;
    NVIC_InitTypeDef    NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE); 

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_USART3);/*UART3-TX*/
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_USART3);/*UART3-RX*/

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;                                                   
    GPIO_Init(GPIOC, &GPIO_InitStructure);

	 DMA_DeInit(DMA1_Stream3);
	/*DMA通道配置*/
    while(DMA_GetCmdStatus(DMA1_Stream3)!=DISABLE){}    
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = USART3_DR;                                           /*外设地址*/
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)USART3_Tx_Buf;                                /*内存地址*/
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                                         /*dma传输方向*/
    DMA_InitStructure.DMA_BufferSize = USART3_MAX_TX_LEN;                                           /*设置DMA在传输时缓冲区的长度*/
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                                /*设置DMA的外设递增模式，一个外设*/
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                                         /*设置DMA的内存递增模式*/
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;                         /*外设数据字长*/
    DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;                             /*内存数据字长*/
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                                                   /*设置DMA的传输模式*/
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                                             /*设置DMA的优先级别*/
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                                          /*指定如果FIFO模式或直接模式将用于指定的流:不使能FIFO模式*/  
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;                               /*指定了FIFO阈值水平*/  
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                                     /*指定的Burst转移配置内存传输*/
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;                             /*指定的Burst转移配置外围转移*/
    DMA_Init(DMA1_Stream3, &DMA_InitStructure);                                                     /*配置DMA1的通道*/    
    DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);                                                    /*使能中断*/
	/*DMA发送中断*/
	NVIC_InitStructure.NVIC_IRQChannel=DMA1_Stream3_IRQn;              
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; /*抢占优先级1*/
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	DMA_DeInit(DMA1_Stream1);                                                                       /*串口接收DMA配置*/
	while(DMA_GetCmdStatus(DMA1_Stream1)!=DISABLE){}        
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = USART3_DR;                      						/*外设地址*/
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)USART3_Rx_Buf;                                   /*内存地址*/
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                                         /*DMA传输方向*/
    DMA_InitStructure.DMA_BufferSize = USART3_MAX_RX_LEN;                                           /*设置DMA在传输时缓冲区的长度*/
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                                /*设置DMA的外设递增模式，一个外设*/
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                                         /*设置DMA的内存递增模式*/
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;                         /*外设数据字长*/
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;                                 /*内存数据字长*/
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                                                   /*设置DMA的传输模式*/
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                                         /*设置DMA的优先级别*/
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                                          /*指定如果FIFO模式或直接模式将用于指定的流 ： 不使能FIFO模式*/    
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;                               /*指定了FIFO阈值水平*/    
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                                     /*指定的Burst转移配置内存传输*/      
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;                             /*指定的Burst转移配置外围转移*/
    DMA_Init(DMA1_Stream1, &DMA_InitStructure);                                                     /*配置DMA1的通道*/  
    DMA_Cmd(DMA1_Stream1,ENABLE);                                                                   /*使能通道*/

	USART_InitStructure.USART_BaudRate = USART3_BAUDRATE;                                           /*配置串口*/
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;                                               /*通道设置为串口中断*/  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;                                       /*中断占先等级*/
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;                                              /*中断响应优先级*/
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                                                 /*打开中断*/  
    NVIC_Init(&NVIC_InitStructure);                                                                 /*配置中断*/ 

    USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);                                                    /*采用DMA方式发送*/
    USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);                                                    /*采用DMA方式接收*/
    USART_ITConfig(USART3,USART_IT_TC,DISABLE);                                                     /*中断配置*/
    USART_ITConfig(USART3,USART_IT_RXNE,DISABLE);
    USART_ITConfig(USART3,USART_IT_TXE,DISABLE);
    USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);                                                    /*开启USART3空中断*/
    USART_Cmd(USART3, ENABLE);                                                                      /*启动串口*/ 	
}

/**
 * @brief       USART1串口DMA发送中断
 * 
 */
void USART1_DMA_TX_IRQHandler(void)
{
	if(DMA_GetFlagStatus(USART1_DMA_TX_STREAM,USART1_DMA_TC_FLAG)!=RESET)
	{
		DMA_Cmd(USART1_DMA_TX_STREAM,DISABLE);   //DISABLE DMA  Tx   	
		USART1_TX_FLAG=0;						//USART2发送标志(关闭)
		DMA_ClearFlag(USART1_DMA_TX_STREAM,USART1_DMA_TC_FLAG);    //清除DMA发送完成标志 
		USART_ClearFlag(USART1,USART_FLAG_TC);	                 //清除串口1的标志位   
	}

}
/**
 * @brief       USART1串口DMA接收中断
 * 
 */
void USART1_DMA_RX_IRQHandler(void)
{
	if(DMA_GetFlagStatus(USART1_DMA_RX_STREAM,USART1_DMA_RC_FLAG)!=RESET)
	{
		rece_flag =1;
		DMA_ClearFlag(USART1_DMA_RX_STREAM,USART1_DMA_RC_FLAG);    //清除DMA接收完成标志    
		DMA_Cmd(USART1_DMA_RX_STREAM,DISABLE);//DISABLE DMA  Rx
		DMA_SetCurrDataCounter(USART1_DMA_RX_STREAM,8);
		DMA_Cmd(USART1_DMA_RX_STREAM,ENABLE);//ENABLE DMA  Rx
	}	
}

/**
 * @brief       USART1串口DMA发送数据
 * 
 */
void Uart1_DMASendConfig(u8 *buffer, u32 size)
{
	while(USART1_TX_FLAG);						//等待上一次发送完成（USART1_TX_FLAG为1即还在发送数据）
	USART1_TX_FLAG=1;							//USART2发送标志（启动发送）	
	DMA_MemoryTargetConfig(USART1_DMA_TX_STREAM,(uint32_t)buffer,DMA_Memory_0);
	DMA_SetCurrDataCounter(USART1_DMA_TX_STREAM,size);
	DMA_Cmd(USART1_DMA_TX_STREAM,ENABLE);//ENABLE DMA  Tx
}

/**
 * @brief       串口3中断服务程序
 * 
 */
void USART3_IRQHandler(void)
{
    static uint8_t rx_len = 0;
    /*空闲中断触发*/
	if(USART_GetITStatus(USART3,USART_IT_IDLE)!=RESET)                                              
	{
		DMA_Cmd(DMA1_Stream1, DISABLE);                                                          /* 暂时关闭DMA数据尚未处理 */
		rx_len = USART3_MAX_RX_LEN - DMA_GetCurrDataCounter(DMA1_Stream1);                       /* 获取接收到的数据长度*/
		ABS_Encoder_DMA_DataProcess(USART3_Tx_Buf,USART3_Rx_Buf,rx_len);                         /* 报文数据处理*/
		DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF1);                                              /* 清DMA标志位 */
		DMA_SetCurrDataCounter(DMA1_Stream1,USART3_MAX_RX_LEN);                                      /* 重新赋值计数值，必须大于等于最大可能接收到的数据帧数目 */
		DMA_Cmd(DMA1_Stream1, ENABLE);                                                              /*打开DMA*/
		USART_ReceiveData(USART3);                                                                  /*清除空闲中断标志位,接收函数有清标志位的作用*/ 
        //GPIO_SetBits(TX_EN_GPIO_Port,TX_EN_Pin);                                                   //485切换为发送          
	}
    /*串口发送完成*/
    if(USART_GetFlagStatus(USART3,USART_IT_TXE)==RESET)                                             
    {
        USART3_Flag_Tx_Busy=0;                                                                                                                    
        USART_ITConfig(USART3,USART_IT_TC,DISABLE);
        GPIO_ResetBits(TX_EN_GPIO_Port,TX_EN_Pin);                                               //485切换为接收
    }
}
/**
 * @brief       DMA1_Stream3中断处理函数
 * 
 */
void DMA1_Stream3_IRQHandler(void)
{
    if(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)!=RESET)                                       /*等待DMA1_Steam3传输完成*/
    {
        DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);                                                 /*清除DMA1_Steam3传输完成标志*/
        DMA_Cmd(DMA1_Stream3,DISABLE);                                                              /*关闭使能*/
        USART_ITConfig(USART3,USART_IT_TC,ENABLE);                                                  /*打开串口发送完成中断*/
        USART3_Flag_Tx_Busy =1;
    }	
}
/**
 * @brief       USART3串口DMA发送数据
 * 
 */
void Uart3_DMASendConfig(u8 *buffer, u32 size)
{
	while(USART3_Flag_Tx_Busy);						//等待上一次发送完成（USART3_TX_FLAG为1即还在发送数据）	
    memcpy(USART3_Tx_Buf,buffer,size);
	DMA_MemoryTargetConfig(DMA1_Stream3,(uint32_t)USART3_Tx_Buf,DMA_Memory_0);
	DMA_SetCurrDataCounter(DMA1_Stream3,size);
	DMA_Cmd(DMA1_Stream3,ENABLE);//ENABLE DMA  Tx    
}
    


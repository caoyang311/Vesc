#include "abs_encoder.h"
#include "debug.h"
#include "uart.h"
#include "gpio.h"
#include "motor_pmsm.h"
#include "motor.h"
#include "foc.h"
#include "pid.h"
#include "at24c02.h"

ABS_ENCODER ABS_Encoder;

/******************************************************************************
  * 函数功能:   CRC8校验函数
  * 输入参数:   *message：接收的数据指针
                len：数据长度
  * 返 回 值:    CRC8校验值
  * 说    明： 多项式：G(X)=X^8+1 LSB first  Poly: 0000 0001
                        LSB first  : 1000 0000 =0X80
*****************************************************************************/  
uint8_t CRC8_ChkValue(uint8_t *message, uint8_t len)
{
    uint8_t crc_val;
    uint8_t i;
    crc_val = 0;
    while(len--)
    {
        crc_val ^= *message++;
        for(i = 0;i < 8;i++)
        {
            if(crc_val & 0x01)
                crc_val = (crc_val >> 1) ^ 0X80;
            else
                crc_val >>= 1;
        }
    }
    return crc_val;
}


/******************************************************************************
    * 函数功能:     DMA_USART3数据处理函数
    * 输入参数:     *data：接收的数据指针
                    size：数据长度
    * 返 回 值:    无
    * 说    明：   外部调用-多摩川编码器2
*****************************************************************************/  
void ABS_Encoder_DMA_DataProcess(u8 *txdata,u8 *rxdata,u16 size)
{
    uint8_t     crc=0;
    uint8_t     buf[USART3_MAX_RX_LEN]={0}; 

    for(int i=0;i<size;i++)
    {
        buf[i]=*rxdata++; 
    }

    if(buf[0]==*txdata)
    {
        crc=CRC8_ChkValue(buf,size-1);
        if(crc==buf[size-1])
        {
            ABS_Encoder.Encoder_Value=buf[4]<<16|buf[3]<<8|buf[2];
        }
        else
        {
            // Encoder_ReadFaultNum++;
            // if(Encoder_ReadFaultNum > ED_READ_FAULT_MAXNUM)
            // {
            //     Encoder_ReadStatus=1;
            // }
        }

    }          
}
/******************************************************************************
    * 函数功能:     DMA_USART3发送数据
    * 输入参数:     *id：请求数据指令
                    size：数据长度
    * 返 回 值:    无
    * 说    明：   外部调用-多摩川编码器2
*****************************************************************************/ 
void ABS_Encoder_DMA_SendData(u8 id,u16 size)
{
    GPIO_SetBits(TX_EN_GPIO_Port,TX_EN_Pin); 
    Uart3_DMASendConfig(&id,size);
}
/******************************************************************************
    * 函数功能:     计算电机电角度
    * 输入参数:     void
    * 返 回 值:    float:电机实时电角度（0~2PI）
    * 说    明：   外部调用
*****************************************************************************/ 
float ABS_Encoder_CalcAngle( void )
{
	uint32_t encoder_value =0;
	
    if(ABS_Encoder.Encoder_Value >= ABS_Encoder.InitPhaseShift)
    {
        encoder_value =  (ABS_Encoder.Encoder_Value - ABS_Encoder.InitPhaseShift)%TWOPI_CODER;
    }
    else
    {
        encoder_value = (ENCODER_MAX_VALUE - ABS_Encoder.InitPhaseShift + ABS_Encoder.Encoder_Value)%TWOPI_CODER;
    }
    
		ABS_Encoder.EAngle = (float)encoder_value * FACTOR;
		
    if(ABS_Encoder.EAngle > TWOPI)
    {
       ABS_Encoder.EAngle -= TWOPI;
    }
    else if (ABS_Encoder.EAngle < -TWOPI)
    {
        ABS_Encoder.EAngle += TWOPI;
    }
    return ABS_Encoder.EAngle;
}

/**
 * @brief       编码器零位校准
 * @param       void 
 * @return      void
 */
void ABS_Encoder_Calibration(void)
{
    static uint32_t wTimebase=0;
	uint32_t calibration_data=0;
    wTimebase++;

    if(wTimebase <= T_ALIGNMENT_PWM_STEPS)//开始对齐
    {
        FOC_Input.Id_ref = I_ALIGNMENT * wTimebase / T_ALIGNMENT_PWM_STEPS;               
        FOC_Input.Iq_ref = 0;       
        FOC_Input.theta = ALIGNMENT_ANGLE;//对齐电角度
        Get_Vbus_Voltage(&(FOC_Input.Udc));   //获取母线电压
        Get_PhaseCurrentValues(&Current_Iabc);//相电流采样
        Clarke_Transf(Current_Iabc,&Current_Ialpha_beta);        //CLARK 变换
        Angle_To_Cos_Sin(FOC_Input.theta,&Transf_Cos_Sin);     //由角度计算 park变换和 反park变换的 COS SIN值
        Park_Transf(Current_Ialpha_beta,Transf_Cos_Sin,&Current_Idq);  //Park变换，由Ialpha Ibeta 与角度信息，去计算Id Iq  // 由交流信息转化为直流信息，方便PID控制 
        Current_PID_Calc(FOC_Input.Id_ref,Current_Idq.Id,&ID_InitStructure,&Voltage_DQ.Vd);     //D轴电流环PID  根据电流参考与电流反馈去计算 输出电压
        Current_PID_Calc(FOC_Input.Iq_ref,Current_Idq.Iq,&IQ_InitStructure,&Voltage_DQ.Vq);     //Q轴电流环PID  根据电流参考与电流反馈去计算 输出电压
        Rev_Park_Transf(Voltage_DQ,Transf_Cos_Sin,&Voltage_Alpha_Beta);                //反park变换  通过电流环得到的dq轴电压信息结合角度信息，去把直流信息转化为交流信息用于SVPWM的输入
        SVPWM_Calc(Voltage_Alpha_Beta,FOC_Input.Udc,TS);       //SVPWM 计算模块  
        ABS_Encoder_DMA_SendData(CF_DATA_ID_0,1); //读取偏移角                           
    }
    else//对齐结束
    {
        ABS_Encoder.InitPhaseShift = ABS_Encoder.Encoder_Value;
        ABS_Encoder.Valid_Flag  = VALID_VALUE;
        calibration_data =  ((uint32_t)ABS_Encoder.Valid_Flag <<24)|(ABS_Encoder.InitPhaseShift);
        __disable_irq(); // 关闭全局中断
        IIC_AT24C02_BufferWrite((uint8_t*)(&calibration_data),0,sizeof(calibration_data));//将偏移角和校验标志位写入eeprom
        IIC_Waite_AT24C02_Standby();
        __enable_irq(); // 打开全局中断
        BLDC_Motor.Motor_Init_State = MOTOR_INIT_ENCODER_ALIGNMENT_END;  	
    }
}

/**
 * @brief       获取电机旋转方向
 * @param       void
 * @return      uint8_t  旋转方向
 */
void ABS_Encoder_GetDirection(void)
{
    if(ABS_Encoder.Direction==UNKOWN)//方向未知
    {
        ABS_Encoder.Current_Capture = ABS_Encoder.Encoder_Value;
        if(ABS_Encoder.Previous_Capture == 0)
        {
            ABS_Encoder.Previous_Capture = ABS_Encoder.Current_Capture; 
        }
        else
        {
            if(ABS_Encoder.Current_Capture > ABS_Encoder.Previous_Capture)//
            {
                if((ABS_Encoder.Current_Capture - ABS_Encoder.Previous_Capture)>100000)//反转溢出
                {
                    ABS_Encoder.Direction = CCW; //反转
                }
                else if((ABS_Encoder.Current_Capture - ABS_Encoder.Previous_Capture)>1000)
                {
                    ABS_Encoder.Direction = CW;//正转
                }
            }
            else if(ABS_Encoder.Current_Capture < ABS_Encoder.Previous_Capture)
            {
                if((ABS_Encoder.Previous_Capture - ABS_Encoder.Current_Capture)>100000)//正转溢出
                {
                    ABS_Encoder.Direction = CW; //正转
                }
                else if((ABS_Encoder.Previous_Capture - ABS_Encoder.Current_Capture)>1000)
                {
                    ABS_Encoder.Direction = CCW;//反转
                }  
            }
            ABS_Encoder.Previous_Capture = ABS_Encoder.Current_Capture; 
        }
         
    }
}

/**
 * @brief       计算平均机械转速
 * @param       void 
 * @return      void
 */
void ABS_Encoder_CalcAvrgMecSpeed(void)
{
    uint8_t bBufferIndex = 0u;
    uint32_t OverflowCntSample;

    //ABS_Encoder_GetDirection();
    ABS_Encoder.Current_Capture = ABS_Encoder.Encoder_Value;
    if(ABS_Encoder.Direction != UNKOWN)
    {
        if(ABS_Encoder.Direction == CW)//正转
        {
            //看当前捕获是否小于前一次捕获的脉冲数，当前捕获值大于前一次捕获，那么应该是发生了溢出，当前捕获应该是开始了新的一圈
            OverflowCntSample = (ABS_Encoder.Current_Capture < ABS_Encoder.Previous_Capture) ? 1 : 0; 
                //计算两次捕获的脉冲的差，求两次捕获的间隔
                //间隔脉冲数 = 当前捕获脉冲数 - 前一次的捕获的脉冲数 - 溢出次数*4000
            ABS_Encoder.DeltaCapturesBuffer[ENC_AVERAGING_FIFO_DEPTH] =
            (int32_t)(ABS_Encoder.Current_Capture)-(int32_t)(ABS_Encoder.Previous_Capture)+((int32_t)(OverflowCntSample))*(int32_t)(ENCODER_MAX_VALUE); 
        }
        else if(ABS_Encoder.Direction == CCW)//反转
        {
            //如果当前捕获大于前一次捕获，说明发生了溢出
            OverflowCntSample = ( ABS_Encoder.Current_Capture > ABS_Encoder.Previous_Capture ) ? 1 : 0; 
                //计算两次捕获的脉冲计数的差，求间隔
                //间隔脉冲数 = 当前捕获脉冲数 - 前一次的捕获脉冲数 + 溢出次数*4000
                ABS_Encoder.DeltaCapturesBuffer[ENC_AVERAGING_FIFO_DEPTH] =
            (int32_t)(ABS_Encoder.Current_Capture)-(int32_t)(ABS_Encoder.Previous_Capture)-((int32_t)(OverflowCntSample))*(int32_t)(ENCODER_MAX_VALUE);      
        }
        //更新脉冲计数
        ABS_Encoder.Previous_Capture = ABS_Encoder.Current_Capture;  
        //滑动平均滤波
        for ( bBufferIndex = 0u; bBufferIndex < ENC_AVERAGING_FIFO_DEPTH; bBufferIndex++ )
        {
            ABS_Encoder.DeltaCapturesBuffer[bBufferIndex] = ABS_Encoder.DeltaCapturesBuffer[bBufferIndex+1];
            ABS_Encoder.OverallAngleVariation += ABS_Encoder.DeltaCapturesBuffer[bBufferIndex];
        }
        //刚开始缓存：计算平均值
        if(ABS_Encoder.Buff_Full_Flag == 0)
        {
            ABS_Encoder.Buff_Counter++;
            ABS_Encoder.ENC_Period_Avg = ABS_Encoder.OverallAngleVariation/ABS_Encoder.Buff_Counter;
            ABS_Encoder.OverallAngleVariation = 0;
        }
        //buff缓存满：计算平均值
        else if(ABS_Encoder.Buff_Full_Flag == 1)
        {
            ABS_Encoder.ENC_Period_Avg = ABS_Encoder.OverallAngleVariation/ENC_AVERAGING_FIFO_DEPTH;// 2K采样率下，脉冲计数
            ABS_Encoder.OverallAngleVariation = 0;      
        }
        if(ABS_Encoder.Buff_Counter >= ENC_AVERAGING_FIFO_DEPTH )//buff缓存满标志位
        {
            ABS_Encoder.Buff_Full_Flag = 1;
        } 
        //rpm
        ABS_Encoder.AvrMecSpeed =  (float)(60.0f*ENCODER_CYCLE*ABS_Encoder.ENC_Period_Avg)/(ENCODER_MAX_VALUE);//（60*f*脉冲计数）/总的脉冲数        
    }
    else
    {
        ABS_Encoder.AvrMecSpeed = 0;
    }
 
}
/**
 * @brief       编码器初始化
 * @param       void 
 * @return      void
 */
void ABS_Encoder_Init(void)
{
    uint32_t calibration_data=0;

    IIC_AT24C02_PageRead((uint8_t*)(&calibration_data),0,sizeof(calibration_data));//读取校验数据
    ABS_Encoder.Valid_Flag = (uint8_t)(calibration_data >>24);
    ABS_Encoder.InitPhaseShift = (calibration_data & 0x0FFFFFF);
    if(ABS_Encoder.Valid_Flag != VALID_VALUE)//初始化校验数据到eeprom
    {
        ABS_Encoder.Valid_Flag = INVALID_VALUE;
        ABS_Encoder.InitPhaseShift =0;
        calibration_data =  ((uint32_t)ABS_Encoder.Valid_Flag <<24)|(ABS_Encoder.InitPhaseShift);
        IIC_AT24C02_BufferWrite((uint8_t*)(&calibration_data),0,sizeof(calibration_data));
        IIC_Waite_AT24C02_Standby();        
    }
}
/**
 * @brief       编码器复位
 * @param       void 
 * @return      void
 */
void ABS_Encoder_Reset(void)
{
    uint8_t BufferSize;
    uint8_t Index;
  
    BufferSize =ENC_SPEED_ARRAY_SIZE;
     /* Erase speed buffer */
    for ( Index = 0u; Index < BufferSize; Index++ )
    {
        ABS_Encoder.DeltaCapturesBuffer[Index] = 0;
    }
    ABS_Encoder.Buff_Counter= 0;
    ABS_Encoder.Buff_Full_Flag =0;
		ABS_Encoder.Previous_Capture = 0;
		ABS_Encoder.Current_Capture = 0;
    ABS_Encoder.Encoder_Value =0;
}





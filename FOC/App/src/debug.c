#include "debug.h"
#include "string.h"
#include "uart.h"
#include "foc.h"

Union_Typedef rece_arry[2];
Union_Typedef send_wave[WAVE_BUFF_MAX];
uint8_t tx_index;
uint8_t rece_flag;
/**
 * @brief       UART接收数据解析 
 * 帧头(2 byte) + 空数据（1 byte）+ 指令类型（1 byte）+ 数据（4 byte）  
 * 0xAA 0xFF  
 * @param       void     
 * @return      void
 */
void Uart_ReceiveDataParse(void)
{
    if(rece_flag == 1)
    {
      rece_flag = 0;
      if((rece_arry[0].char_table[0] ==0xAA) && (rece_arry[0].char_table[1] == 0xFF))//帧头
      {
          if(rece_arry[0].char_table[3] == 0x01)  //启停命令
          {
              if(rece_arry[1].char_table[0] == 0x01)
              {
                  //启动电机
              }
              else if(rece_arry[1].char_table[0] == 0x02)
              {
                  //停止电机
              }
          }
          else if(rece_arry[0].char_table[3] == 0x02) //转速参考
          {
            if((rece_arry[1].float_data <= 3000) &&(rece_arry[1].float_data >= -3000))
            {
              FOC_Input.speed_ref = (u16)(rece_arry[1].float_data);
            } 
          }
          else if(rece_arry[0].char_table[3] == 0x03) //转速KP
          {
            //Speed_InitStructure.hKp_Gain = Q10(rece_arry[1].float_data);
          }
          else if(rece_arry[0].char_table[3] == 0x04) //转速KI
          {
            //Speed_InitStructure.hKi_Gain = Q10(rece_arry[1].float_data);
          }
          else if(rece_arry[0].char_table[3] == 0x05) //力矩KP
          {
            //Torque_InitStructure.hKp_Gain = Q15(rece_arry[1].float_data);
          }
          else if(rece_arry[0].char_table[3] == 0x06) //力矩KI
          {
            //Torque_InitStructure.hKi_Gain = Q15(rece_arry[1].float_data);
          }                        
          else if(rece_arry[0].char_table[3] == 0x07) //力矩参考
          {
            //FOC_Structure.Torque_Reference = Q15(rece_arry[1].float_data);
          }
          else if(rece_arry[0].char_table[3] == 0x08) //磁链KP
          {
            //Flux_InitStructure.hKp_Gain = Q15(rece_arry[1].float_data);
          } 
          else if(rece_arry[0].char_table[3] == 0x09) //磁链KI
          {
            //Flux_InitStructure.hKi_Gain = Q15(rece_arry[1].float_data);
          }
          else if(rece_arry[0].char_table[3] == 0x0a) //磁链参考
          {
            //FOC_Structure.Flux_Reference = Q15(rece_arry[1].float_data);
          }                                                
      }
    }

}

/**
 * @brief       Uart接收波形数据到缓存区
 * 
 * @param       data1 : 发送波形数据1
 * @param       data2 ：发送波形数据2  
 * @param       data3 ：发送波形数据3   
 * @return      void
 */
void Uart_SendWave(float data1,float data2,float data3)
{
    send_wave[0].float_data = data1;
    send_wave[1].float_data = data2;
    send_wave[2].float_data = data3;
    send_wave[3].char_table[0] = 0x00;
    send_wave[3].char_table[1] = 0x00;
    send_wave[3].char_table[2] = 0x80;
    send_wave[3].char_table[3] = 0x7f;
		Uart1_DMASendConfig(&(send_wave[0].char_table[0]),16);
}
/*******************************************************************
 * Function  : 系统滴答计时器-微秒延迟
 * Parameter : u32 us--多少微秒
 * Return    : void
 * Comment   :
********************************************************************/
void Delay_Us(u32 us) 
{
    /*第一步	    【选择时钟】*/
	//因为位2是选择时钟的，0=21HMZ=外部时钟，1=168HMZ=内部时钟，
	 SysTick->CTRL &=~(1<<2);
	/*第二步 	【填入计数值】*/
		SysTick->LOAD=us*21-1;
	/*第三步 	【清零计数器】*/
		SysTick->VAL=0;
	/*第四步 	【使能计时器】*/
	//因为位0是计时器使能，1使能，0关闭，位0写1，用|=
		SysTick->CTRL|=1<<0;   
	/*第五步     【判断计数是否结束】*/
	//因为位16,倒计时结束时，该位会变成1，所以用while判断真假，若SysTick->CTRL1的16位为1，则&后，
	//SysTick->CTRL1除16位外全为0，如果他不是1位0的话SysTick->CTRL&(1<<16)的结果为0（假）
	 while(!(SysTick->CTRL&(1<<16)));
	/*第六步	     【关闭定时器】*/
	 SysTick->CTRL&=~(1<<0);
}


/**
 * @brief       usb 任务
 * 
 * @param       void
 * @return      void
 */
void Debug_Task(void)
{
  Uart_ReceiveDataParse();
}

#include "parameter_identify.h"
#include "foc.h"
#include "gpio.h"
#include "timer.h"
#include "motor_pmsm.h"
#include "adc.h"
#include "motor.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "debug.h"

uint8_t state = RESISTOR; //参数辨识状态机
Resistor_Struct Resistor; //电阻辨识结构体
D_Inductor_Struct D_Inductor;//D轴电感辨识结构体
float Freq_Hz;//高频注入
float FFT_InputBuff[FFT_LENGTH*2] = {0.0};
float FFT_OutputBuff[FFT_LENGTH] = {0.0};

float sin_func[16]={0,0.40674,	0.74314,	0.95106,	0.99452,	0.86603,	0.58779,	0.20791,	-0.20791,	-0.58779,	-0.86603,	-0.99452,	-0.95106,	-0.74314,	-0.40674,	-0.00000000000000001};
/**
 * @brief       相电阻参数识别
 * 
 * @param       void     
 * @return      void
 */
void Resistor_Identify(void)
{
    uint8_t i =0;
    static float res_curr=0;
    static uint16_t res_paraidcnt = 0;
    static float res_voltRcd[2];
    static float res_currRcd[2];

    switch (Resistor.Resistor_Identify_Status)
    {
    case 0://设置电压矢量角度为0(或其他恒定值)。令Vd = 0，Vq = 0，通过svpwm输出
        Voltage_DQ.Vd   = 0.0f;//D轴电压
        Voltage_DQ.Vq   = 0.0f;//Q轴电压 
        for(i=0;i<2;i++)
        {
            res_voltRcd[i] =0;
            res_currRcd[i] =0;
        }
        res_paraidcnt ++;
        if(res_paraidcnt > 1600)
        {
            res_paraidcnt = 0;
            Resistor.Resistor_Identify_Status =1;
        }
        
        break;
    case 1://将采样到的相电流进行clark变换，计算Is = sqrt(Iα2 + Iβ2)
    res_curr = sqrtf(Current_Ialpha_beta.Ialpha * Current_Ialpha_beta.Ialpha + Current_Ialpha_beta.Ibeta * Current_Ialpha_beta.Ibeta);
        if((res_curr < 1.5f)&&(Voltage_DQ.Vd<3.0f)) //电流小于1.5A,且电压小于1.6V
        {
            Voltage_DQ.Vd += 0.001f;   //增大Vd
        }
        else if((res_curr > 2.0f)&&(Voltage_DQ.Vd>0.0f))//大于2A
        {
            Voltage_DQ.Vd -= 0.001f;   //减小Vd
        }
        else if((res_curr > 1.5f)&&(res_curr < 2.0f))
        {
            res_paraidcnt ++;
            //低通滤波 ：截至频率：1.6hz
            res_voltRcd[0] = res_voltRcd[0] * 0.999f +  Voltage_DQ.Vd * 0.001f;  //记录此时Vd值
            res_currRcd[0] = res_currRcd[0] * 0.999f +  res_curr * 0.001f;           //记录此时curr值
            if(res_paraidcnt > 3200) //等待电流稳定:200ms
            {
                res_paraidcnt = 0;
                Voltage_DQ.Vd = 0.0f;    //Vd清0
                Resistor.Resistor_Identify_Status = 2;               
            }
        }
        break;
    case 2:
    res_curr = sqrtf(Current_Ialpha_beta.Ialpha * Current_Ialpha_beta.Ialpha + Current_Ialpha_beta.Ibeta * Current_Ialpha_beta.Ibeta);
        if((res_curr < 3.0f)&&(Voltage_DQ.Vd<3.0f)) //电流小于1.5A,且电压小于1.6V
        {
            Voltage_DQ.Vd += 0.001f;   //增大Vd
        }
        else if((res_curr > 3.5f)&&(Voltage_DQ.Vd>0.0f))//大于2A
        {
            Voltage_DQ.Vd -= 0.001f;   //减小Vd
        }
        else if((res_curr > 3.0f)&&(res_curr < 3.5f))
        {
            res_paraidcnt ++;
            //低通滤波 ：截至频率：
            res_voltRcd[1] = res_voltRcd[1] * 0.999f +  Voltage_DQ.Vd * 0.001f;  //记录此时Vd值
            res_currRcd[1] = res_currRcd[1] * 0.999f +  res_curr * 0.001f;           //记录此时curr值
            if(res_paraidcnt > 3200) //等待电流稳定:200ms
            {
                res_paraidcnt = 0;
                Voltage_DQ.Vd = 0.0f;    //Vd清0
                Resistor.Resistor_Value = (res_voltRcd[1] - res_voltRcd[0])/(res_currRcd[1] - res_currRcd[0]);
                Resistor.Resistor_Identify_Status = 0;
                state = D_INDUCTOR;
            }
        }        
        break;    
    default:
        break;
    }
} 
/**
 * @brief       D轴电感参数识别
 * 
 * @param       void     
 * @return      void
 */
void D_Inductor_Identify(void)
{
    #ifdef TI_IDENTIFY

    static float id_curr=0;
    static uint16_t ld_paraidcnt = 0;
    static float ld_voltRcd;
    static float ld_currRcd;
    switch (D_Inductor.Ld_Identify_Status)
    {
    case 0:
        Voltage_DQ.Vd   = 0.0f;//D轴电压
        Voltage_DQ.Vq   = 0.0f;//Q轴电压   
        ld_voltRcd =0;
        ld_currRcd =0;
        Freq_Hz = 2 * MOTOR_MAX_FREQ; //高频注入频率：400HZ
        ld_paraidcnt ++;
        if(ld_paraidcnt > 1600)
        {
            ld_paraidcnt = 0;
            D_Inductor.Ld_Identify_Status =1; 
        }     
        break;
    case 1:
       
        id_curr = sqrtf(Current_Ialpha_beta.Ialpha * Current_Ialpha_beta.Ialpha + Current_Ialpha_beta.Ibeta * Current_Ialpha_beta.Ibeta);
        if((id_curr < 2.0f)&&(Voltage_DQ.Vd<3.0f))
        {
                Voltage_DQ.Vd += 0.001f;
        }
        else if((id_curr > 2.5f)&&(Voltage_DQ.Vd > 0.0f))
        {
                Voltage_DQ.Vd -= 0.001f;
        }
        else if ((id_curr > 2.0f) && (id_curr < 2.5f))
        {
            ld_paraidcnt ++;
            ld_voltRcd = ld_voltRcd * 0.999f +  Voltage_DQ.Vd * 0.001f;
            ld_currRcd = ld_currRcd * 0.999f +  id_curr * 0.001f;
            if(ld_paraidcnt > 3200)
            {
                ld_paraidcnt = 0;
                Voltage_DQ.Vd = 0.0f;
                D_Inductor.Ld_Value = (ld_voltRcd - ld_currRcd * Resistor.Resistor_Value)  / (ld_currRcd * TWOPI * Freq_Hz);
                D_Inductor.Ld_Identify_Status = 0;
                state = Q_INDUCTOR;
            }
        }
        break;
    }

    #endif 

    #ifdef ST_IDENTIFY
    static uint8_t sin_index= 0;
    static uint8_t Rcd_index= 0;
    static uint16_t ld_paraidcnt = 0;
    static float ld_currRcd[128];
    int i = 0;
    switch (D_Inductor.Ld_Identify_Status)
    {
    case 0:
        Voltage_DQ.Vd   = 0.0f;//D轴电压
        Voltage_DQ.Vq   = 0.0f;//Q轴电压
        sin_index = 0;
        Rcd_index = 0;
        ld_paraidcnt ++;
        if(ld_paraidcnt > 1600)
        {
            ld_paraidcnt = 0;
            D_Inductor.Ld_Identify_Status =1; 
        }        
        break;
    case 1:
        Voltage_DQ.Vd = 3.0f * sin_func[sin_index];
        sin_index++;
        if(sin_index >= 15)
        {
            sin_index =0;
        }
        ld_paraidcnt ++;
        if(ld_paraidcnt>=3200)//等待响应稳定
        {
            ld_paraidcnt = 3200;
            ld_currRcd[Rcd_index] = Current_Idq.Id; 
            Rcd_index++;
            if(Rcd_index>=128)
            {
                for (i = 0; i < FFT_LENGTH; i++)
                {
                    FFT_InputBuff[i * 2] = ld_currRcd[i];//实部赋值
                    FFT_InputBuff[i * 2 + 1] = 0;//虚部赋值，固定为0.
                }
                arm_cfft_f32(&arm_cfft_sR_f32_len128, FFT_InputBuff, 0, 1);
                arm_cmplx_mag_f32(FFT_InputBuff, FFT_OutputBuff, FFT_LENGTH); 

                FFT_OutputBuff[0] /= FFT_LENGTH;
                for (i = 1; i < FFT_LENGTH; i++)//输出各次谐波幅值
                {
                    FFT_OutputBuff[i] /= (FFT_LENGTH/2);
                }
                D_Inductor.Ld_Identify_Status =2;  
            }
						
        }
        Uart_SendWave(Voltage_DQ.Vd,Current_Idq.Id,Current_Iabc.Ia);
        break;
    case 2:
        //printf("FFT Result:\r\n");

//        for (int i = 0; i < FFT_LENGTH; i++)//输出各次谐波幅值
//        {
//            printf("%d:\t%.2f\r\n", i, FFT_OutputBuff[i]);
//        }
        D_Inductor.Ld_Value = (3.0f / (FFT_OutputBuff[8]  *TWOPI * 1000));//|uq|/(|iq|*2pi*f)
        D_Inductor.Ld_Identify_Status = 0;
        state = Q_INDUCTOR;
        break;    
    }   

    #endif
}

/**
 * @brief       电机参数辨识处理,在ADC中断中调用，执行频率与FOC执行频率一致
 * 
 * @param       void     
 * @return      void
 */
void Parameter_Identify_Handle(void)
{
    Get_Vbus_Voltage(&(FOC_Input.Udc));   //获取母线电压
    Get_PhaseCurrentValues(&Current_Iabc);//相电流采样
    Clarke_Transf(Current_Iabc,&Current_Ialpha_beta); //CLARK 变换

    #ifdef ST_IDENTIFY
    FOC_Input.theta = 0;//电角度
    Angle_To_Cos_Sin(FOC_Input.theta,&Transf_Cos_Sin);     //由角度计算 park变换和 反park变换的 COS SIN值
    Park_Transf(Current_Ialpha_beta,Transf_Cos_Sin,&Current_Idq);  //Park变换，由Ialpha Ibeta 与角度信息，去计算Id Iq
    #endif
    switch (state)
    {
    case RESISTOR:
        FOC_Input.theta = 0;//电角度
        Resistor_Identify(); 
        break;
    case D_INDUCTOR:
        D_Inductor_Identify();
        break;
    default:
        break;
    }
    #ifdef TI_IDENTIFY
    FreqGenerator(Freq_Hz,&(FOC_Input.theta));
    Angle_To_Cos_Sin(FOC_Input.theta,&Transf_Cos_Sin);     //由角度计算 park变换和 反park变换的 COS SIN值
    Park_Transf(Current_Ialpha_beta,Transf_Cos_Sin,&Current_Idq);  //Park变换，由Ialpha Ibeta 与角度信息，去计算Id Iq 
    Rev_Park_Transf(Voltage_DQ,Transf_Cos_Sin,&Voltage_Alpha_Beta);                //反park变换
    SVPWM_Calc(Voltage_Alpha_Beta,FOC_Input.Udc,TS);       //SVPWM 计算模块
    #endif

    #ifdef ST_IDENTIFY
    Rev_Park_Transf(Voltage_DQ,Transf_Cos_Sin,&Voltage_Alpha_Beta);                //反park变换
    SVPWM_Calc(Voltage_Alpha_Beta,FOC_Input.Udc,TS);       //SVPWM 计算模块
    #endif
}


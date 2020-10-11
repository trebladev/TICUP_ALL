/*
****************************************************************************
// Created by TerrorBlade on 2020/7/28.

	*文件：ad7606.c
	*作者：xzw
	*描述：修改自 康威科技AD7606 stm32f103驱动
						1、将f103改成f407驱动
						2、将标准库改为Hal库驱动
						3、需要FPU支持，在keil中添加FPU开启
						4、增加三个FFT计算函数，增加对正弦波采样幅值、频率计算功能。
	*最后修改时间：2020/8/7

****************************************************************************
*/

#include "stm32f4xx.h"
#include <stdio.h>
#include "ad7606.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "arm_common_tables.h"

FIFO_T	g_tAD;            //定义一个数据交换缓冲区



/*
************************************************************
*函数名 ad7606_init
*功能 初始化ad7606
*形参 无
*返回值 无
************************************************************
*/
void ad7606_init(void)
{
    ad7606_SetOS(6);    //设置过采样模式

    ad7606_Reset();     //复位ad7606

    AD_CONVST_HIGH();
}





/*
************************************************************
*函数名 ad7606_Reset
*功能 复位ad7606
*形参 无
*返回值 无
************************************************************
*/
void ad7606_Reset(void)
{
    /* AD7606高电平复位，最小脉冲要求50ns */

    AD_RESET_LOW();

    AD_RESET_HIGH();
    AD_RESET_HIGH();
    AD_RESET_HIGH();
    AD_RESET_HIGH();

    AD_RESET_LOW();
}

/*
************************************************************
*函数名 ad7606_SetOS
*功能 设置过采样模式
*形参 ucMode: 0-6 0表示无过采样 1表示2倍 2表示4倍 3表示8倍 4表示16倍
 5表示32倍 6表示64倍
*返回值 无
************************************************************
*/
void ad7606_SetOS(uint8_t _ucMode)
{
    if (_ucMode == 1)
    {
        AD_OS2_0();
        AD_OS1_0();
        AD_OS0_1();
    }
    else if (_ucMode == 2)
    {
        AD_OS2_0();
        AD_OS1_1();
        AD_OS0_0();
    }
    else if (_ucMode == 3)
    {
        AD_OS2_0();
        AD_OS1_1();
        AD_OS0_1();
    }
    else if (_ucMode == 4)
    {
        AD_OS2_1();
        AD_OS1_0();
        AD_OS0_0();
    }
    else if (_ucMode == 5)
    {
        AD_OS2_1();
        AD_OS1_0();
        AD_OS0_1();
    }
    else if (_ucMode == 6)
    {
        AD_OS2_1();
        AD_OS1_1();
        AD_OS0_0();
    }
    else
    {
        AD_OS2_0();
        AD_OS1_0();
        AD_OS0_0();
    }
}

/*
************************************************************
*函数名 ad7606_StartConv
*功能 启动adc转换
*形参 无
*返回值 无
************************************************************
*/
void ad7606_StartConv(void)
{
    /* 上升沿开始转换，低电平至少持续25ns  */
    AD_CONVST_LOW();
    AD_CONVST_LOW();
    AD_CONVST_LOW();	/* 连续执行2次，低电平持续时间大约为50ns */
    AD_CONVST_LOW();
    AD_CONVST_LOW();
    AD_CONVST_LOW();
    AD_CONVST_LOW();
    AD_CONVST_LOW();
    AD_CONVST_LOW();

    AD_CONVST_HIGH();
}


//spi写数据


void SPI_SendData(u16 data)
{
    u8 count=0;
    AD_SCK_LOW();	//???μ??óDD§
    for(count=0;count<16;count++)
    {
        if(data&0x8000)
            AD_MISO_LOW();
        else
            AD_MISO_HIGH();
        data<<=1;
        AD_SCK_LOW();
        AD_CSK_HIGH();		//é?éy??óDD§
    }
}

//spi读数据
u16 SPI_ReceiveData(void)
{
    u8 count=0;
    u16 Num=0;
    AD_CSK_HIGH();
    for(count=0;count<16;count++)//?á3?16??êy?Y
    {
        Num<<=1;
        AD_SCK_LOW();	//???μ??óDD§
        if(AD_MISO_IN)Num++;
        AD_CSK_HIGH();
    }
    return(Num);
}

/*
************************************************************
*函数名 ad7606_ReadBytes
*功能 读取ADC采样结果
*形参 无
*返回值 usData
************************************************************
*/
uint16_t ad7606_ReadBytes(void)
{
    uint16_t usData = 0;

    usData = SPI_ReceiveData();

    /* Return the shifted data */
    return usData;
}

/*
************************************************************
*函数名 ad7606_IRQSrc
*功能 定时调用函数，用于读取ADC转换数据
*形参 无
*返回值 无
************************************************************
*/
void ad7606_IRQSrc(void)
{
    uint8_t i;
    uint16_t usReadValue;
		
		
    /* 读取数据
    示波器监视，CS低电平持续时间 35us
    */
    AD_CS_LOW();
    for (i = 0; i < CH_NUM; i++)
    {
        usReadValue = ad7606_ReadBytes();
        if (g_tAD.usWrite < FIFO_SIZE)
        {
            g_tAD.usBuf[g_tAD.usWrite] = usReadValue;
            ++g_tAD.usWrite;
        }
    }
		
		g_tAD.usWrite = 0;
    AD_CS_HIGH();
    ad7606_StartConv();
}

/*
************************************************************
*函数名 ad7606_StartRecord
*功能 开始采集
*形参 _ulFreq
*返回值 无
************************************************************
*/
void ad7606_StartRecord(uint32_t _ulFreq)
{
    //ad7606_Reset();

    ad7606_StartConv();				/* ???ˉ2é?ù￡?±ü?aμú1×éêy?Yè?0μ??êìa */

    g_tAD.usRead = 0;				/* ±?D??ú?a??TIM2???°??0 */
    g_tAD.usWrite = 0;


    MX_TIM4_Init(_ulFreq);                 //设置定时器4频率
    HAL_TIM_Base_Start_IT(&htim4);         //使能定时器4中断

}

/*
************************************************************
*函数名 ad7606_StopRecord
*功能 停止采集
*形参 无
*返回值 无
************************************************************
*/
void ad7606_StopRecord(void)
{
    HAL_TIM_Base_Stop_IT(&htim4);
}








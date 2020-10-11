/*
****************************************************************************
// Created by TerrorBlade on 2020/7/28.

	*�ļ���ad7606.c
	*���ߣ�xzw
	*�������޸��� �����Ƽ�AD7606 stm32f103����
						1����f103�ĳ�f407����
						2������׼���ΪHal������
						3����ҪFPU֧�֣���keil�����FPU����
						4����������FFT���㺯�������Ӷ����Ҳ�������ֵ��Ƶ�ʼ��㹦�ܡ�
	*����޸�ʱ�䣺2020/8/7

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

FIFO_T	g_tAD;            //����һ�����ݽ���������



/*
************************************************************
*������ ad7606_init
*���� ��ʼ��ad7606
*�β� ��
*����ֵ ��
************************************************************
*/
void ad7606_init(void)
{
    ad7606_SetOS(6);    //���ù�����ģʽ

    ad7606_Reset();     //��λad7606

    AD_CONVST_HIGH();
}





/*
************************************************************
*������ ad7606_Reset
*���� ��λad7606
*�β� ��
*����ֵ ��
************************************************************
*/
void ad7606_Reset(void)
{
    /* AD7606�ߵ�ƽ��λ����С����Ҫ��50ns */

    AD_RESET_LOW();

    AD_RESET_HIGH();
    AD_RESET_HIGH();
    AD_RESET_HIGH();
    AD_RESET_HIGH();

    AD_RESET_LOW();
}

/*
************************************************************
*������ ad7606_SetOS
*���� ���ù�����ģʽ
*�β� ucMode: 0-6 0��ʾ�޹����� 1��ʾ2�� 2��ʾ4�� 3��ʾ8�� 4��ʾ16��
 5��ʾ32�� 6��ʾ64��
*����ֵ ��
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
*������ ad7606_StartConv
*���� ����adcת��
*�β� ��
*����ֵ ��
************************************************************
*/
void ad7606_StartConv(void)
{
    /* �����ؿ�ʼת�����͵�ƽ���ٳ���25ns  */
    AD_CONVST_LOW();
    AD_CONVST_LOW();
    AD_CONVST_LOW();	/* ����ִ��2�Σ��͵�ƽ����ʱ���ԼΪ50ns */
    AD_CONVST_LOW();
    AD_CONVST_LOW();
    AD_CONVST_LOW();
    AD_CONVST_LOW();
    AD_CONVST_LOW();
    AD_CONVST_LOW();

    AD_CONVST_HIGH();
}


//spiд����


void SPI_SendData(u16 data)
{
    u8 count=0;
    AD_SCK_LOW();	//???��??��DD��
    for(count=0;count<16;count++)
    {
        if(data&0x8000)
            AD_MISO_LOW();
        else
            AD_MISO_HIGH();
        data<<=1;
        AD_SCK_LOW();
        AD_CSK_HIGH();		//��?��y??��DD��
    }
}

//spi������
u16 SPI_ReceiveData(void)
{
    u8 count=0;
    u16 Num=0;
    AD_CSK_HIGH();
    for(count=0;count<16;count++)//?��3?16??��y?Y
    {
        Num<<=1;
        AD_SCK_LOW();	//???��??��DD��
        if(AD_MISO_IN)Num++;
        AD_CSK_HIGH();
    }
    return(Num);
}

/*
************************************************************
*������ ad7606_ReadBytes
*���� ��ȡADC�������
*�β� ��
*����ֵ usData
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
*������ ad7606_IRQSrc
*���� ��ʱ���ú��������ڶ�ȡADCת������
*�β� ��
*����ֵ ��
************************************************************
*/
void ad7606_IRQSrc(void)
{
    uint8_t i;
    uint16_t usReadValue;
		
		
    /* ��ȡ����
    ʾ�������ӣ�CS�͵�ƽ����ʱ�� 35us
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
*������ ad7606_StartRecord
*���� ��ʼ�ɼ�
*�β� _ulFreq
*����ֵ ��
************************************************************
*/
void ad7606_StartRecord(uint32_t _ulFreq)
{
    //ad7606_Reset();

    ad7606_StartConv();				/* ???��2��?����?����?a�̨�1������y?Y��?0��??����a */

    g_tAD.usRead = 0;				/* ��?D??��?a??TIM2???��??0 */
    g_tAD.usWrite = 0;


    MX_TIM4_Init(_ulFreq);                 //���ö�ʱ��4Ƶ��
    HAL_TIM_Base_Start_IT(&htim4);         //ʹ�ܶ�ʱ��4�ж�

}

/*
************************************************************
*������ ad7606_StopRecord
*���� ֹͣ�ɼ�
*�β� ��
*����ֵ ��
************************************************************
*/
void ad7606_StopRecord(void)
{
    HAL_TIM_Base_Stop_IT(&htim4);
}








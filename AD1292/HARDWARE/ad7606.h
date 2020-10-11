/*
****************************************************************************
// Created by TerrorBlade on 2020/7/28.

	*�ļ���ad7606.h
	*���ߣ�xzw
	*�������޸��� �����Ƽ�AD7606 stm32f103����
						1����f103�ĳ�f407����
						2������׼���ΪHal������
				
				��������
				
				PC11 -> RD			OUT_PP		VERY_HIGH
				PG12 -> D7			IPD
				PC12 -> CS			OUT_PP		VERY_HIGH
				PC10 -> RST			OUT_PP		VERY_HIGH
				PG13 -> CB			OUT_PP		VERY_HIGH
				PG11 -> RANGE		OUT_PP		VERY_HIGH
				PC4 -> OS0			OUT_PP		VERY_HIGH
				PC5 -> OS1			OUT_PP		VERY_HIGH
				PC6 -> OS2			OUT_PP		VERY_HIGH
				GND -> GND			OUT_PP    VERY_HIGH
			
	*���ѣ����ڵ�Ƭ����IO�ӿڵ��ٶ����ƣ��޷��ﵽ8ͨ��ͬʱ200KHz�ɼ��������������
				32��Ӳ�������жϣ����Լ��˴���
	
	*����޸�ʱ�䣺2020/8/7

****************************************************************************
*/
#ifndef ad7606_ad7606_H
#define ad7606_ad7606_H
#include "stdint.h"
#include "sys.h"
/*����ȫ���жϵĺ�*/
#define ENABLE_INT()	__set_PRIMASK(0)	/* ʹ��ȫ���ж� */
#define DISABLE_INT()	__set_PRIMASK(1)	/* ��ֹȫ���ж� */

/*ÿ���������ֽڣ�ͨ����*/
#define CH_NUM			1
#define FIFO_SIZE		1*1024*2



/*����GPIO*/
#define AD_SPI_SCK_PIN                   GPIO_PIN_11
#define AD_SPI_SCK_GPIO_PORT             GPIOC
#define AD_SPI_SCK_GPIO_CLK              RCC_APB2Periph_GPIOC

#define AD_SPI_MISO_PIN                  GPIO_PIN_12
#define AD_SPI_MISO_GPIO_PORT            GPIOG
#define AD_SPI_MISO_GPIO_CLK             RCC_APB2Periph_GPIOG

#define AD_CS_PIN                        GPIO_PIN_12
#define AD_CS_GPIO_PORT                  GPIOC
#define AD_CS_GPIO_CLK                   RCC_APB2Periph_GPIOC

#define AD_RESET_PIN                     GPIO_PIN_10
#define AD_RESET_GPIO_PORT               GPIOC
#define AD_RESET_GPIO_CLK                RCC_APB2Periph_GPIOC

#define AD_CONVST_PIN                    GPIO_PIN_13
#define AD_CONVST_GPIO_PORT              GPIOG
#define AD_CONVST_GPIO_CLK               RCC_APB2Periph_GPIOG

#define AD_RANGE_PIN                     GPIO_PIN_11
#define AD_RANGE_GPIO_PORT               GPIOG
#define AD_RANGE_GPIO_CLK                RCC_APB2Periph_GPIOG

#define AD_OS0_PIN                     GPIO_PIN_4
#define AD_OS0_GPIO_PORT               GPIOC
#define AD_OS0_GPIO_CLK                RCC_APB2Periph_GPIOC

#define AD_OS1_PIN                     GPIO_PIN_5
#define AD_OS1_GPIO_PORT               GPIOC
#define AD_OS1_GPIO_CLK                RCC_APB2Periph_GPIOC

#define AD_OS2_PIN                     GPIO_PIN_6
#define AD_OS2_GPIO_PORT               GPIOC
#define AD_OS2_GPIO_CLK                RCC_APB2Periph_GPIOC

#define AD_CS_LOW()     				AD_CS_GPIO_PORT->BSRR = AD_CS_PIN << 16U
#define AD_CS_HIGH()     				AD_CS_GPIO_PORT->BSRR = AD_CS_PIN

#define AD_RESET_LOW()					AD_RESET_GPIO_PORT->BSRR = AD_RESET_PIN << 16U
#define AD_RESET_HIGH()					AD_RESET_GPIO_PORT->BSRR = AD_RESET_PIN

#define AD_CONVST_LOW()					AD_CONVST_GPIO_PORT->BSRR = AD_CONVST_PIN << 16U
#define AD_CONVST_HIGH()				AD_CONVST_GPIO_PORT->BSRR = AD_CONVST_PIN

#define AD_RANGE_5V()					  AD_RANGE_GPIO_PORT->BSRR = AD_RANGE_PIN << 16U
#define AD_RANGE_10V()					AD_RANGE_GPIO_PORT->BSRR = AD_RANGE_PIN

#define AD_OS0_0()						  AD_OS0_GPIO_PORT->BSRR = AD_OS0_PIN << 16U
#define AD_OS0_1()						  AD_OS0_GPIO_PORT->BSRR = AD_OS0_PIN

#define AD_OS1_0()						  AD_OS1_GPIO_PORT->BSRR = AD_OS1_PIN << 16U
#define AD_OS1_1()						  AD_OS1_GPIO_PORT->BSRR = AD_OS1_PIN

#define AD_OS2_0()						  AD_OS2_GPIO_PORT->BSRR = AD_OS2_PIN << 16U
#define AD_OS2_1()						  AD_OS2_GPIO_PORT->BSRR = AD_OS2_PIN

#define AD_MISO_LOW()					  AD_SPI_MISO_GPIO_PORT->BSRR  = AD_SPI_MISO_PIN << 16U
#define AD_MISO_HIGH()				  AD_SPI_MISO_GPIO_PORT->BSRR = AD_SPI_MISO_PIN

#define AD_SCK_LOW()					  AD_SPI_SCK_GPIO_PORT->BSRR  = AD_SPI_SCK_PIN << 16U
#define AD_CSK_HIGH()				    AD_SPI_SCK_GPIO_PORT->BSRR = AD_SPI_SCK_PIN

#define AD_MISO_IN					PGin(12)

/*AD���ݲɼ�������*/
typedef struct
{
    uint16_t usRead;
    uint16_t usWrite;
    uint16_t usCount;
    uint16_t usBuf[FIFO_SIZE];
}FIFO_T;

void ad7606_Reset(void);
void ad7606_init(void);
void ad7606_SetOS(uint8_t _ucMode);
void ad7606_StartRecord(uint32_t _ulFreq);
void ad7606_StopRecord(void);
void ad7606_IRQSrc(void);
void ad7606_StartConv(void);

extern FIFO_T  g_tAD;

#endif //ad7606_ad7606_H


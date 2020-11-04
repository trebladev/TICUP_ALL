/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "AD1292.h"
#include "sys.h"
#include <stdio.h>
#include "USART_HMI.h"
#include "findpeaks.h"
#include "FIR_48.h"
#include "delay.h"
#include "medfilt1.h"
#include "bpm.h"
#include "mpu6050.h"
#include "ad7606.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define g_num 50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static int mid_filt_start_flag;
static int temp_ad_send_flag;
static int warning_flag;
static int debug_flag;


static int half_send_flag;

uint8_t wifi_bpm_flag = 0x01;
uint8_t wifi_temp_flag = 0x02;
uint8_t wifi_step_flag = 0x03;

static int32_t value;
float32_t temp_ad;
int real_temp;
uint8_t wifi_real_temp;
static float32_t sum;
float32_t a=-1.809628E-09;
float32_t b=-3.325395E-06;
float32_t c=-1.814103E-01;
float32_t d=2.055894E+02;

short aacx,aacy,aacz;

s32 get_volt(u32 num);                 //�Ѳɵ���3���ֽڲ���ת���з���32λ��
float32_t val1,val2;
//u32 val1_last;
float32_t calculate_cache[36];             //���㲿�ֻ���
float32_t calculate_cache1[18];            //���㲿�ֻ���
float32_t fir_put[36];                     //�˲��������
float32_t fir_put1[36];                     //�˲��������
float32_t breath_cache[36];                //�����˲�����
int32_t breath_calculate_cache[250];    //�������㻺��
int32_t val1_int;                          //��������int32��ʽ
int32_t bpm_cache[1300];                   //�������ʵ����ݻ���
float32_t mid_filt_cache[midfilt_num];             //��ֵ�˲�����
float32_t mid_filt_cache1[midfilt_num];             //��ֵ�˲�����



int32_t pn_npks;                           //���ʷ�ֵ��⺯����ֵ����
int32_t pn_locs[15];                       //���ʷ�ֵ��⺯�������ֵ��
int32_t pn_npks_b;                         //������ֵ��⺯����ֵ����
int32_t pn_locs_b[15];                     //������ֵ��⺯�������ֵ��

float32_t a1,a2;                           //������ʾϵ��
float32_t b1,b2;                           //������ʾϵ��
float32_t mean;                            //��ֵ�˲����ֵ
float32_t val_init_data[Val_Init_Num];     //���ʳ�ʼ������
float32_t breath_init_cache[Val_Init_Num]; //������ʼ������
float32_t mid_val;

static float bpm;                          //������ֵ

float32_t min;
uint32_t min_index;

static float32_t G;
static float32_t faacx;
static float32_t faacy;
static float32_t faacz;
static int32_t G_cal[g_num];
static int32_t max_G;
static uint32_t max_G_loc;
static int8_t step;
static uint8_t wifi_step;
static uint8_t bpm_val;
static int8_t int_wifi_bpm_val;
static uint8_t wifi_bpm_val;
static int i;

int32_t pn_npks_s;                           
int32_t pn_locs_s[15];                       
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
 static uint16_t j,n,mid_filt_num;
	
	
	//uint8_t data_to_send[60];//���ڷ��ͻ���
	u32 cannle[2];	//�洢����ͨ��������
	s32	p_Temp[2];	//���ݻ���
	
	
	//data_to_send[0]=0xAA;
	//data_to_send[1]=0xAA;
	//data_to_send[2]=0xF1;	
	//data_to_send[3]=8;
	
	int z,p;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
	MX_TIM5_Init();
	MX_I2C1_Init();
  MX_USART1_UART_Init();
	ad7606_init();
	AD_RANGE_5V();                                        //���������ѹ���ֵ
  ad7606_StartRecord(200);                               //���ò���Ƶ��Ϊ200hz
  /* USER CODE BEGIN 2 */
  ADS1292_Init();

	HAL_Delay(10);
	while(Set_ADS1292_Collect(0))//0 �����ɼ�  //1 1mV1Hz�ڲ������ź� //2 �ڲ��̽���������
		{	
				//printf("1292�Ĵ�������ʧ��\r\n");
		}	
		//printf("�Ĵ������óɹ�\r\n");
	
	MPU_Init();
		
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);                       //�����ⲿ�ж�
		
	HAL_TIM_Base_Start_IT(&htim3);                        //������ʱ��3�ж�
	HAL_TIM_Base_Start_IT(&htim5);                        //������ʱ��2�ж�
	Get_val_init_data(val_init_data,breath_init_cache);
	
	arm_min_f32(breath_init_cache,Val_Init_Num,&min,&min_index);
	
	for(p=0;p<Val_Init_Num;p++)
		{
			breath_init_cache[p] = breath_init_cache[p] - (float)0.95*min;
		}
		
		
  ADS1292_val_init(val_init_data,&a1,&b1);
	//ADS1292_val_init(breath_init_cache,&a2,&b2);
  /* USER CODE END 2 */
	//arm_fir5_init();
	arm_fir48_init();
	debug_flag = HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_4);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		//printf("%f\n\r",G*10);
		if(temp_ad_send_flag == 1)
		{
			//printf("x0.val=%d",real_temp);
			//send_ending_flag();
			temp_ad_send_flag=0;
		}
		if(i == g_num)
		{
			i = 0;
			arm_max_q31(G_cal,g_num,&max_G,&max_G_loc);
			if(max_G > 1500)
			{
				//maxim_peaks_above_min_height(pn_locs,&pn_npks,G_cal,g_num,0.7*max_G);
				maxim_find_peaks(pn_locs_s,&pn_npks_s,G_cal,g_num,0.4*max_G,35,100);
				step = step + pn_npks_s;
			}
			else
			{
				step = step;
			}
			
		}
		wifi_step = step;
		//HAL_UART_Transmit(&huart1,&wifi_step_flag,1,0xFFFF);
		//HAL_UART_Transmit(&huart1,&wifi_step,1,0xFFFF);
    /* USER CODE END WHILE */
		if(ads1292_recive_flag)
		{										
							
					cannle[0]=ads1292_Cache[3]<<16 | ads1292_Cache[4]<<8 | ads1292_Cache[5];//��ȡԭʼ����		
					cannle[1]=ads1292_Cache[6]<<16 | ads1292_Cache[7]<<8 | ads1292_Cache[8];
						
					p_Temp[0] = get_volt(cannle[0]);	//�Ѳɵ���3���ֽ�ת���з���32λ��
					p_Temp[1] = get_volt(cannle[1]);	//�Ѳɵ���3���ֽ�ת���з���32λ��
					
					//�з�����Ϊ��תΪ�޷��ţ��޷�����Ϊ�߼�����
					cannle[0] = p_Temp[0];
					cannle[1]	= p_Temp[1];
					
					val1 = cannle[1]*(a1)+b1;                                //�����ݸ�Ϊ���ڴ�������ʾ����ֵ
						
          calculate_cache[j] = val1;                                //�����ݴ����˲����㻺��������
							
          j++;
							
				  z++;
							
          if(j == 19)
          {
								
            j=18;
                
						arm_fir_f32_lp_48(calculate_cache,fir_put);              //�����ݽ���FIR 48Hz��ͨ�˲�
								
						if(mid_filt_start_flag == 0)
						{
							mid_filt_cache[mid_filt_num] = fir_put[0];
							mid_filt_num++;
							if(mid_filt_num == midfilt_num)
							{
								mid_filt_start_flag = 1;
								}
							}
						else if(mid_filt_start_flag == 1)
						{
							arm_copy_f32(mid_filt_cache+1,mid_filt_cache1,midfilt_num-1);
							mid_filt_cache1[midfilt_num-1] = fir_put[0];
							mid_val=midfilt1(mid_filt_cache1,midfilt_num,midfilt_num);
										
							bpm_cache[n] = (fir_put[0]-mid_val+100);
							n++;
							if(n>1300)
							{
								n=0;
								maxim_peaks_above_min_height(pn_locs,&pn_npks,bpm_cache,1300,135);                   //Ѱ��175���ϵķ�
								bpm = bpm_calculate(pn_locs,pn_npks);
								warning_flag = find_bad_bpm(pn_locs,pn_npks);
								int_wifi_bpm_val = (int)bpm;
								wifi_bpm_val = int_wifi_bpm_val;
								//bpm = 60.0/(pn_locs[pn_npks-1]-pn_locs[pn_npks-2])*204;                              //�������� �㷨:����֮�����*������
								if(half_send_flag == 0)
								{
									if(debug_flag == 1)
									{
										printf("n0.val=%d",(int)bpm);
										send_ending_flag();
										half_send_flag = 1;
									}
								else{
										HAL_UART_Transmit(&huart1,&wifi_bpm_flag,1,0xFFFF);
										HAL_UART_Transmit(&huart1,&wifi_bpm_val,1,0xFFFF);
										half_send_flag = 1;
									}
									
								}
								else
									half_send_flag = 0;
								//printf("n0.val=%d",(int)bpm); 
								//�����������
								//send_ending_flag();
								
								}
								//printf("add 2,0,%0.f",fir_put[0]-mid_val+100);
								bpm_val = fir_put[0]-mid_val+100;
								if(debug_flag == 1)
								{
									printf("add 9,0,%0.f",fir_put[0]-mid_val+100);
									send_ending_flag();
								}
								else{
									HAL_UART_Transmit(&huart1,&bpm_val,1,0xFFFF);
								}
								//send_ending_flag();
								arm_copy_f32(mid_filt_cache1,mid_filt_cache,midfilt_num);
							}
						   
						arm_copy_f32(calculate_cache+1,calculate_cache1,18);     //��ǰһ����ĺ�18λ���������������У���ΪFIR�˲�����Ⱥ��ʱ
								
						arm_copy_f32(calculate_cache1,calculate_cache,18);       //�����������18λ��������һ������
								
           }
						
						ads1292_recive_flag=0;
							
				}
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if((GPIO_Pin == GPIO_PIN_8) && (ADS_DRDY == 0))
  {
    PA8_IRQHandler();
  }
    

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	
	static u8 j;
	
  if (htim == (&htim3))
  {
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);
		faacx = aacx /16384.0;
		faacy = aacy /16384.0;
		faacz = aacz /16384.0;
		G = sqrt((faacx*faacx+faacy*faacy+faacz*faacz));
	  G_cal[i] = (int)(G*1000);
		i++;
	}
	if(htim == (&htim5))
		{
			value = (int32_t)((10000)*((float)((short)g_tAD.usBuf[0])/32768/2));	//�̣�??1mv
			temp_ad = value*value*value*a+b*value*value+c*value+d;

			if(j<3)
			{
				sum = sum +temp_ad;
				j++;
				}
			else if(j==3)
			{
				j = 0;
				real_temp = sum / 3;
				wifi_real_temp = real_temp;
				if(debug_flag == 1)
				{
					printf("x0.val=%d",(int)(sum*10/3));
					send_ending_flag();
				}
				else
				{
					HAL_UART_Transmit(&huart1,&wifi_temp_flag,1,0xFFFF);
					HAL_UART_Transmit(&huart1,&wifi_real_temp,1,0xFFFF);
				}
				
				delay_us(2);
				if(debug_flag == 1)
				{
					printf("n2.val=%d",step);
					send_ending_flag();
				}
				else
				{
					HAL_UART_Transmit(&huart1,&wifi_step_flag,1,0xFFFF);
					HAL_UART_Transmit(&huart1,&wifi_step,1,0xFFFF);
				}
				
				//printf("x0.val=%d",(int)(sum*10));
				//send_ending_flag();
				temp_ad_send_flag = 1;
				sum = 0;
			}
		}
	if (htim == (&htim4))                    //TIM4�жϺ��ȡadc��ֵ
  {
    ad7606_IRQSrc();
    }
		
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

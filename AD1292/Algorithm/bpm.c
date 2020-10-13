/*
 * @Autor: xzw
 * @Date: 2020-09-16 09:42:06
 * @LastEditors: xzw
 * @LastEditTime: 2020-09-16 09:58:49
 * @Description: 
 */

#include "bpm.h"
#include "math.h"
static int signal_bad_bpm_flag;
static int bad_bpm_flag;
static int signal_loc;
static int32_t signal_time;

int32_t bpm_calculate(int32_t *loc,int32_t size)
{
    int32_t num,bpm;
	  num = size;
		
    switch (num)
    {
    case 2:
        bpm = 60.0/(loc[num-1]-loc[num-2])*FPS;                              //计算心率 算法:两峰之间点数*采样率
        break;
    
    case 3:
        bpm = 60.0/(((loc[num-1]-loc[num-2])+(loc[num-2]-loc[num-3]))/2)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
        break;

    case 4:
        bpm = 60.0/(((loc[num-1]-loc[num-2])+(loc[num-2]-loc[num-3])+(loc[num-3]-loc[num-4]))/3)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
        break;

    case 5:
        bpm = 60.0/(((loc[num-1]-loc[num-2])+(loc[num-2]-loc[num-3])+(loc[num-3]-loc[num-4])+(loc[num-4]-loc[num-5]))/4)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
        break;

    case 6:
        bpm = 60.0/(((loc[num-1]-loc[num-2])+(loc[num-2]-loc[num-3])+(loc[num-3]-loc[num-4])+(loc[num-4]-loc[num-5])+(loc[num-5]-loc[num-6]))/5)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
			 break;
    case 7:
        bpm = 60.0/(((loc[num-1]-loc[num-2])+(loc[num-2]-loc[num-3])+(loc[num-3]-loc[num-4])+(loc[num-4]-loc[num-5])+(loc[num-5]-loc[num-6])+(loc[num-6]-loc[num-7]))/6)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
        break;
		case 8:
			  bpm = 60.0/(((loc[num-1]-loc[num-2])+(loc[num-2]-loc[num-3])+(loc[num-3]-loc[num-4])+(loc[num-4]-loc[num-5])+(loc[num-5]-loc[num-6])+(loc[num-6]-loc[num-7])+(loc[num-7]-loc[num-8]))/7)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
        break;

    default: bpm = 60.0/(loc[num-1]-loc[num-2])*340;
        break;
    }
		
		//bpm = 60.0/(location[num-1]-location[num-2])*204;
    return bpm;
}


int8_t find_bad_bpm(int32_t *loc,int32_t size)
{
	int32_t num,bpm;
	num = size;
	int i;

	if(signal_bad_bpm_flag == 1)
	{
		bad_bpm_flag = two_num(signal_time,loc[0]+(1300-signal_loc));
		signal_bad_bpm_flag = 0;
	}
	else
	{
  switch(num)
	{
		case 2:
        signal_time = loc[num-1]-loc[num-2];                              //计算心率 算法:两峰之间点数*采样率
				signal_bad_bpm_flag = 1;
				signal_loc = loc[num-1];
				bad_bpm_flag = 0;
        break;
		
		case 3:
				bad_bpm_flag = two_num((loc[num-1]-loc[num-2]),(loc[num-2]-loc[num-3]));
				break;
		
		case 4:
			  for(i=1;i<3;i++)
				{
					bad_bpm_flag = two_num((loc[num-i]-loc[num-i-1]),(loc[num-i-1]-loc[num-i-2]));
					if(bad_bpm_flag == 1)
					{
						break;
					}
				}
				break;
				
		case 5:
			for(i=1;i<4;i++)
				{
					bad_bpm_flag = two_num((loc[num-i]-loc[num-i-1]),(loc[num-i-1]-loc[num-i-2]));
					if(bad_bpm_flag == 1)
					{
						break;
					}
				}
				break;
				
		case 6:
			for(i=1;i<5;i++)
				{
					bad_bpm_flag = two_num((loc[num-i]-loc[num-i-1]),(loc[num-i-1]-loc[num-i-2]));
					if(bad_bpm_flag == 1)
					{
						break;
					}
				}
				break;
				
		case 7:
			for(i=1;i<6;i++)
				{
					bad_bpm_flag = two_num((loc[num-i]-loc[num-i-1]),(loc[num-i-1]-loc[num-i-2]));
					if(bad_bpm_flag == 1)
					{
						break;
					}
				}
				break;
		
		case 8:
			for(i=1;i<7;i++)
				{
					bad_bpm_flag = two_num((loc[num-i]-loc[num-i-1]),(loc[num-i-1]-loc[num-i-2]));
					if(bad_bpm_flag == 1)
					{
						break;
					}
				}
				break;
		
		default: bad_bpm_flag=0;
        break;
		
		
		
		
	}
	
}
	return bad_bpm_flag;
	
	
}

int8_t two_num(int32_t num1,int32_t num2)
{
	float bili;
	if(num1 > num2)
	{
		bili = num1 -num2;
		if(bili>(num2*0.2))
		{
			return 1;
		}
		else
			return 0;
	}
	else if(num1 < num2)
	{
		bili = num2 -num1;
		if(bili>(num1*0.2))
		{
			return 1;
		}
		else
			return 0;
	}
	
}











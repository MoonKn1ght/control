/**
  ******************************************************************************
  * @FileName			    CCD.h
  * @Description
  * @author                 Yu Xinyi
  * @note
  ******************************************************************************
  *
  * Copyright (c) 2023 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
**/

#include "CCD.h"
#include "hal.h"

float ccd_norm[128] = {2122.7,2156.54,2253.56,2302.95,2364.85,2384.49,2396.97,2428.91,
						2480.69,2519.74,2546.13,2574.88,2585.78,2596.07,2601.78,2609.21,2613.86,
						2617.39,2624.16,2627.02,2630.75,2632.91,2636.34,2635.21,2639.37,2637.85,
						2640.23,2640.35,2642.84,2640.99,2644.47,2646.65,2647.22,2647.11,
						2650.54,2648.92,2649.7,2648.93,2653.38,2653.39,2654.39,2652.66,
						2655.27,2655.2,2657.64,2655.99,2656.98,2656.18,2656.85,2656.03,2658.39,
						2657.44,2659.74,2658.54,2660.06,2658.44,2659.89,2659.08,2661.04,
						2659.3,2661.93,2659.94,2661.27,2659.89,2654.96,2658.26,2662.76,
						2661.22,2663.08,2660.28,2661.75,2659.91,2661.96,2660.47,2661.43,2659.87,
						2661.34,2659.94,2660.41,2659.75,2661.03,2658.01,2659.25,2657.01,2656.67,
						2655.27,2657.45,2655.02,2655.54,2654.36,2655.18,2653.17,2653.52,
						2650.31,2651.69,2650.36,2650.25,2646.86,2647.2,2644.43,2645.1,
						2641.63,2640.21,2637.08,2639.83,2630.44,2632.93,2631.94,2632.31,
						2629.85,2629.91,2624.66,2625.5,2620.12,2615.18,2612.95,2604.39,2589.61,2587.5,2584.95,
						2582.28,2553.78,2483.84,2428.47,2454.0,2465.4,2417.91,2374.55};

void CCD::SI_send() {
    if(state == 0){ //刚开中断会直接触发中断，而不是在下降沿触发
        state++;
    }else if(state == 1){
        HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_SET);
        state++;
    }else{
        __HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC4);
        state = 0;
        HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_RESET);
        //开ADC DMA
        HAL_ADC_Start_DMA(&hadc3, (uint32_t *)data, 128);
    }
}

void CCD::Handler() {
    if(HAL_GetTick() % integrate_time == 0){
        __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC4);
        sample_t = 0;
        sample_complete = false;
    }
    sample_t++;
}

void CCD::init() {
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    last_dir = 64;
}

//确定二值化阈值
void CCD::GetThreshold()
{
	this->max_data = 0;
	this->min_data = 3000;
	for(int i = 0; i < 128; ++i)
	{
		if(this->data_norm[i] > max_data)
		{
			this->max_data = this->data_norm[i];
		}
		else if(this->data_norm[i] < min_data)
		{
			this->min_data = this->data_norm[i];
		}

	}
	if(this->max_data - this->min_data > 700)
	{
		this->threshold = (this->max_data + this->min_data)/2;
	}
	else this->threshold = 0;
}

//归一化
void CCD::Normalization()
{
	for(int i = 0; i < 128; ++i)
	{
		this->data_norm[i] = this->data[i]/(ccd_norm[i]/2663.08);
	}
}

//CCD图像二值化
void CCD::Binarization()
{
	for(int i = 0; i < 128; ++i)
	{
		if(this->data_norm[i] > this->threshold)
		{
			this->bin_ccd[i] = 1;
		}
		else
		{
			this->bin_ccd[i] = 0;
		}
	}
}

//CCD采集图像滤波
void CCD::Filter()
{
	for(int i = 1; i < 127; ++i)
	{
		if(this->bin_ccd[i] == 1 && this->bin_ccd[i-1] == 0 && this->bin_ccd[i+1] == 0)	//消除噪点
		{
			this->bin_ccd[i] = 0;
		}
		if(this->bin_ccd[i] == 0 && this->bin_ccd[i-1] == 1 && this->bin_ccd[i+1] == 1)
		{
			this->bin_ccd[i] = 1;
		}
	}
}

//计算黑线中值（单线）
void CCD::GetDirection()
{
	int i;
	//从两边向中间找线
	for(i = 3; i < 125; ++i)	//从左找线
	{
		if(this->bin_ccd[i-1] + this->bin_ccd[i-2] + this->bin_ccd[i-3] == 3
			&& this->bin_ccd[i+1] + this->bin_ccd[i+2] + this->bin_ccd[i+3] == 0)
		{
			this->Left = i;
			break;
		}
		if(i == 124)
			this->Left = 500;
	}

	for(i = 125; i > 3; --i)
	{
		if(this->bin_ccd[i-1] + this->bin_ccd[i-2] + this->bin_ccd[i-3] == 0
			&& this->bin_ccd[i+1] + this->bin_ccd[i+2] + this->bin_ccd[i+3] == 3)
		{
			this->Right = i;
			break;
		}
		if(i == 4)
			this->Right = 500;
	}

	if(this->Left == 500 || this->Right == 500)	//未检测到线
	{
		this->dir = 500;
	}
	else this->dir = (this->Right + this->Left)/2;	//返回黑线中值坐标
}

////计算赛道偏差值（双线）
//uint16_t CCD::GetError(uint8_t *bin)
//{
//	int i,j;
//	int error_L,error_R;
//
//	if(bin[64] == 0)
//	{
//		for(i = 0; i < 128/2-3; i++)
//		{
//			if(bin[64-i]+bin[64-i-1]+bin[64-i-2] == 3)	//左丢线
//			{
//				return -64;
//			}
//			else if(bin[64+i]+bin[64+i+1]+bin[64+i+2] == 3)	//右丢线
//			{
//				return 64;
//			}
//		}
//		return 200;	//左右均丢线
//	}
//	else	//从中间向两边找线
//	{
//		for(i = 64; i > 3; i--)
//		{
//			if(bin[i-1] + bin[i-2] + bin[i-3] == 0 && bin[i+1] + bin[i+2] + bin[i+3] == 3)
//			{
//				error_L = i;
//				break;
//			}
//			if(i == 3)
//				error_L = 3;
//		}
//
//		for(i = 64; i < 125; i++)
//		{
//			if(bin[i-1] + bin[i-2] + bin[i-3] == 3 && bin[i+1] + bin[i+2] + bin[i+3] == 0)
//			{
//				error_R = i;
//				break;
//			}
//			if(i == 125)
//				error_R = 125;
//		}
//	}
//
//	return (error_L + error_R - 128);	//赛道中线偏差值
//
//}




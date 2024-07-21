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
	uint16_t max_data = 0;
	uint16_t min_data = 3000;
	for(int i = 0; i < 128; ++i)
	{
		if(this->data[i] > max_data)
		{
			max_data = this->data[i];
		}
		else if(this->data[i] < min_data)
		{
			min_data = this->data[i];
		}

	}
	this->threshold = (max_data + min_data)/2;
}

//CCD图像二值化
void CCD::Binarization()
{
	for(int i = 0; i < 128; ++i)
	{
		if(this->data[i] > this->threshold)
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
uint16_t CCD::GetDirection()
{
	int i;
	int Left,Right;
	//从两边向中间找线
	for(i = 3; i < 125; i++)	//从左找线
	{
		if(this->bin_ccd[i-1] + this->bin_ccd[i-2] + this->bin_ccd[i-3] == 3
			&& this->bin_ccd[i+1] + this->bin_ccd[i+2] + this->bin_ccd[i+3] == 0)
		{
			Left = i;
			break;
		}
		if(i == 125)
			Left = 125;
	}

	for(i = 125; i > 3; i--)
	{
		if(this->bin_ccd[i-1] + this->bin_ccd[i-2] + this->bin_ccd[i-3] == 0
			&& this->bin_ccd[i+1] + this->bin_ccd[i+2] + this->bin_ccd[i+3] == 3)
		{
			Right = i;
			break;
		}
		if(i == 3)
			Right = 3;
	}

	if(Left == 125 && Right == 3)	//未检测到线
	{
		return 200;
	}

	return (Right + Left)/2;	//返回黑线中值坐标
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

//小车巡线
void CCD::Line_tracing()
{
	int Mid_error = 15;	//直线行驶范围，待调整


	GetThreshold();
	Binarization();	//二值化
	Filter();	//数据滤波
	this->dir = GetDirection();	//计算偏差值

	if(this->dir == 200)
		this->dir = this->last_dir;

	if(this->dir > 64 + Mid_error && this->dir < 64 - Mid_error)	//直道巡迹
	{

	}
	else	//弯道巡迹
	{

	}

	//

	this->last_dir = this->dir;

}



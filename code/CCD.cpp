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
    if(HAL_GetTick() % integrate_time == 0) __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC4);
}

void CCD::init() {
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

//CCD图像二值化
void CCD::Binarization(uint16_t threshold, uint16_t *data, uint8_t *bin_ccd)
{
	for(int i = 0; i < 128; ++i)
	{
		if(*data++ > threshold)
		{
			*bin_ccd++ = 1;
		}
		else
		{
			*bin_ccd = 0;
		}
	}
}

//CCD采集图像滤波
void CCD::Filter(uint8_t *bin)
{
	for(int i = 1; i < 127; ++i)
	{
		if(bin[i] == 1 && bin[i-1] == 0 && bin[i+1] == 0)	//消除噪点
		{
			bin[i] = 0;
		}
		if(bin[i] == 0 && bin[i-1] == 1 && bin[i+1] == 1)
		{
			bin[i] = 1;
		}
	}
}

//计算赛道偏差值（单线）
uint16_t CCD::GetError(uint8_t *bin)
{
	int i,j;
	int error_L,error_R;
	//从两边向中间找线
	for(i = 3; i < 125; i++)	//从左找线
	{
		if(bin[i-1] + bin[i-2] + bin[i-3] == 3 && bin[i+1] + bin[i+2] + bin[i+3] == 0)
		{
			error_L = i;
			break;
		}
		if(i == 125)
			error_L = 125;
	}

	for(i = 125; i > 3; i--)
	{
		if(bin[i-1] + bin[i-2] + bin[i-3] == 0 && bin[i+1] + bin[i+2] + bin[i+3] == 3)
		{
			error_R = i;
			break;
		}
		if(i == 3)
			error_R = 3;
	}

	if(error_L == 125 && error_R == 3)	//未检测到线
	{
		return 200;
	}

	return (error_R + error_L - 128);
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
	uint16_t threshold_bin = 500;	//二值化阈值，待调整
		int threshold_line = 30;	//直线行驶范围，待调整
	uint16_t CCD_error,CCD_last_error;

	Binarization(threshold_bin, this->data, this->bin_ccd);	//二值化
	Filter(this->bin_ccd);	//数据滤波
	CCD_error = GetError(this->bin_ccd);	//计算偏差值

	if(CCD_error == 200)
		CCD_error = CCD_last_error;

	//分段PID
	if(CCD_error > -threshold_line && CCD_error < threshold_line)	//直道
	{

	}
	else	//弯道
	{

	}

	//控制电机

	CCD_last_error = CCD_error;

}



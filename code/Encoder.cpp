/**
  ******************************************************************************
  * @FileName			    Encoder.h
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

#include "Encoder.h"
#define trans_ratio_N20 298
#define ADD_overflow 65536
void Encoder::Handler() {
	static uint8_t enconder_init_flag = 0;
	if(enconder_init_flag == 0)
	{
		enconder_init_flag = 1;
		return;
	}
	uint32_t temp_CNT = (htim->Instance)->CNT;
	if(temp_CNT >= 0 && temp_CNT < ADD_overflow/2)
		this->cir_direction = 0;
	else  if(temp_CNT <= ADD_overflow && temp_CNT > ADD_overflow/2)
		this->cir_direction = 1;

	if(this->cir_direction == 0)
	{
		this->pulse = this->temp_pulse + ADD_overflow;
		this->temp_pulse = this->pulse;
	}
	if(this->cir_direction == 1)
	{
		this->pulse = this->temp_pulse - ADD_overflow;
		this->temp_pulse = this->pulse;
	}
}

void Encoder::circle_get(){
		this->pulse = this->temp_pulse + ((int32_t)(htim->Instance)->CNT);

	this->circle_CNT = (float)pulse/((float)(trans_ratio_N20*7*2));
}

Encoder::Encoder(TIM_HandleTypeDef *htim) {
    this->htim = htim;
    this->pulse = 0;
    this->temp_pulse = this->pulse;
    this->circle_CNT = 0;
    this->cir_direction = 0;
}

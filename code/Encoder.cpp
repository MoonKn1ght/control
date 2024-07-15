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

#define CNT_OVERFLOW 65536

void Encoder::Handler() {
    pulse += (htim->Instance)->CNT - prev_cnt + carry_flag * CNT_OVERFLOW;
    carry_flag = 0;
    prev_cnt = (htim->Instance)->CNT;

}

Encoder::Encoder(TIM_HandleTypeDef *htim) {
    this->htim = htim;
}

void Encoder::init() {
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
    HAL_TIM_Base_Start_IT(htim);
}

void Encoder::overflow_update() {
    if((htim->Instance)->CNT < CNT_OVERFLOW / 2) carry_flag++;
    else carry_flag--;
}

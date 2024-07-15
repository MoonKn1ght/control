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

#ifndef CONTROL_ENCODER_H
#define CONTROL_ENCODER_H

#include "hal.h"

class Encoder{
public:
    int32_t pulse; //几个脉冲
    float circle_CNT;
    Encoder(TIM_HandleTypeDef* htim);
    void Handler(); //更新角度
    void circle_get();
private:
    int32_t temp_pulse;
    uint8_t cir_direction;
    TIM_HandleTypeDef* htim;
};


#endif //CONTROL_ENCODER_H

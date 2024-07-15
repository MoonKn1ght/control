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
    int32_t pulse = 0; //几个脉冲
    Encoder(TIM_HandleTypeDef* htim);

    void init();
    void Handler(); //更新角度，放在主程序任务轮询中断里
    void overflow_update(); //放在定时器update中断里
private:
    TIM_HandleTypeDef* htim;

    int carry_flag = 0;
    uint32_t prev_cnt = 0;
};


#endif //CONTROL_ENCODER_H

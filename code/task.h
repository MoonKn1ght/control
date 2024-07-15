/**
  ******************************************************************************
  * @FileName			    task.h
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

#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H
#include "hal.h"
#ifdef __cplusplus
extern "C" {
#endif

void setup();

void loop();

void task_handler();

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);
#ifdef __cplusplus
}
#endif


#endif //CONTROL_TASK_H

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

#include "task.h"
#include "hal.h"
#include "module.h"
#include "device.h"
Encoder left_encoder(&htim2);
Encoder right_encoder(&htim3);
N20_Motor left_motor(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2,
                     &left_encoder,
                     298 * 4,
                     1,
                     0
                     );
N20_Motor right_motor(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4,
                     &right_encoder,
                     298 * 4,
                     1,
                     0
);

void setup(){

}

void loop(){
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(500);
}

void task_handler(){

}


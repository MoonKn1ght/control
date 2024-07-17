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
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim2);

	CCD_init();

}

void loop(){
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    CCD_scale_get();
    HAL_Delay(500);
}

void task_handler(){
	left_encoder.circle_get();
}

void TIM2_handler(){
	left_encoder.Handler();
}

void TIM4_handler(){
	CCD_Handler();
}


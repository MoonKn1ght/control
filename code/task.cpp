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

Encoder left_encoder(&htim5);
Encoder right_encoder(&htim2);
N20_Motor left_motor(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2,
                     &left_encoder,
                     298 * 4 * 7,
                     1,
                     0
                     );
N20_Motor right_motor(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4,
                     &right_encoder,
                     298 * 4 * 7,
                     1,
                     0
);

uint16_t ccd_data[128];
int ccd_state = 0;

void ccd_start_sampling(){
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC4);

}

void setup(){
    left_encoder.init();
    right_encoder.init();
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

void loop(){
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(500);
}

void task_handler(){
	left_encoder.Handler();
	right_encoder.Handler();
	left_motor.Handler();
	right_motor.Handler();
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
    if(htim == &htim2){
        right_encoder.overflow_update();
    }else if(htim == &htim5){
        left_encoder.overflow_update();
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef * htim){
    if(htim == &htim4){
        if(ccd_state == 0){
            HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_SET);
            ccd_state++;
            //开ADC DMA
            HAL_ADC_Start_DMA(&hadc3, (uint32_t *)ccd_data, 128);
        }else{
            ccd_state = 0;
            HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_RESET);
            //关闭中断
            __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC4);
        }
    }
}

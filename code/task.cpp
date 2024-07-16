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
N20_Motor right_motor(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2,
                     &left_encoder,
                     298 * 4 * 7,
                     1,
                     0
                     );
N20_Motor left_motor(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4,
                     &right_encoder,
                     298 * 4 * 7,
                     1,
                     0
);
CCD ccd;

float vpwr = 12, vpwr_th = 9;

void get_vpwr(){
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 1000);
	int val = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);
	vpwr = (float)val / 4096 * 3.3 * 8;
}

void setup(){
    left_encoder.init();
    right_encoder.init();
	left_motor.init();
	right_motor.init();
    ccd.init();
}


void loop(){
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(500);

	HAL_UART_Transmit(&huart1, (uint8_t *)ccd.data, 128 * 2, 0xff);
	HAL_UART_Transmit(&huart1, (uint8_t *)"\0\0", 2, 0xff);
	
	get_vpwr();
	if(vpwr < vpwr_th && vpwr > 3) HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
}

void task_handler(){
	left_encoder.Handler();
	right_encoder.Handler();
	left_motor.Handler();
	right_motor.Handler();
	ccd.Handler();
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
    if(htim == &htim2){
        right_encoder.overflow_update();
    }else if(htim == &htim5){
        left_encoder.overflow_update();
    }
}

void CCD_Handler(){ //放在TIM中断回调函数里，注意前置，以免延时
    ccd.SI_send();
}


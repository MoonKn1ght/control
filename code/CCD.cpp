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
    if(HAL_GetTick() % integrate_time == 0){
        __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC4);
        sample_t = 0;
        sample_complete = false;
    }
    sample_t++;
}

void CCD::init() {
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}



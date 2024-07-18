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

#define UART1_BUF_LEN 100
uint8_t uart1_rx_buf[UART1_BUF_LEN];
uint16_t uart1_rx_len = 0;
int uart1_state = 0;

/**************************   device    ***************************/
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
Remote remote(&huart1);

float vpwr = 12, vpwr_th = 9;
int pwr_cnt = 0;

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

	HAL_UART_Receive_IT(&huart1,(uint8_t *)(uart1_rx_buf), 1);//接收一个字节
}

uint8_t start_flag[] = {0xfe, 0xfe};
uint8_t end_flag[] = {0xff, 0xff};
void loop(){
	HAL_UART_Transmit(&huart1, start_flag, 2, 0xffff);
	HAL_UART_Transmit(&huart1, (uint8_t *)ccd.data, 128 * 2, 0xffff);
	HAL_UART_Transmit(&huart1, end_flag, 2, 0xffff);
	HAL_Delay(50);

//    ccd.GetThreshold();
//    ccd.Binarization();	//二值化
//	ccd.Filter();	//数据滤波
//	HAL_UART_Transmit(&huart1, start_flag, 1, 0xffff);
//	HAL_UART_Transmit(&huart1, ccd.bin_ccd, 128, 0xffff);
//	HAL_UART_Transmit(&huart1, end_flag, 1, 0xffff);
//	HAL_Delay(50);
	get_vpwr();
	if(vpwr < vpwr_th && vpwr > 3){
	    if(pwr_cnt >= 10){
            HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
	    }else pwr_cnt++;
	}
	else{
        HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
        pwr_cnt = 0;
	}
}

void task_handler(){
	left_encoder.Handler();
	right_encoder.Handler();
	left_motor.Handler();
	right_motor.Handler();
	ccd.Handler();
	remote.Handler();
	
	if(HAL_GetTick() % 500 == 0) HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

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


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1){
        uint8_t val = uart1_rx_buf[uart1_rx_len++];
        if(uart1_rx_len >= UART1_BUF_LEN) uart1_rx_len = 0;

        if(val == '\r'){
            uart1_state = 1;
        }else if(val == '\n'){
            if(uart1_state == 1){
                remote.process_data((uint16_t*)uart1_rx_buf);
                uart1_state = 0;
                uart1_rx_len = 0;
            }else uart1_state = 0;
        }

        HAL_UART_Receive_IT(&huart1,(uint8_t *)(uart1_rx_buf + uart1_rx_len), 1);//接收一个字节
    }

}

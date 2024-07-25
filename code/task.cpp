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

#define USE_REMOTE


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
                     33.4 / 2 / 1000 / 435 * 432,
                     0
                     );
N20_Motor left_motor(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4,
                     &right_encoder,
                     298 * 4 * 7,
                     33.4 / 2 / 1000 / 435 * 432,
                     0
);
CCD ccd;
Remote remote(&huart1);
IMU imu;
Chassis chassis(&left_motor, &right_motor, &imu, 112.5 / 1000);
Controller controller(&chassis);



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
    imu.init();

	HAL_UART_Receive_IT(&huart1,(uint8_t *)(uart1_rx_buf), 1);//接收一个字节
}

uint8_t start_flag[] = {0xfe, 0xfe};
uint8_t end_flag[] = {0xff, 0xff};
void loop(){
    if(ccd.sample_complete == true){
		HAL_UART_Transmit(&huart1, start_flag, 2, 0xffff);
		HAL_UART_Transmit(&huart1, (uint8_t *)ccd.data, 128 * 2, 0xffff);
		HAL_UART_Transmit(&huart1, end_flag, 2, 0xffff);
		HAL_UART_Transmit(&huart1, (uint8_t *)&chassis.x, 4, 0xffff);
		HAL_UART_Transmit(&huart1, (uint8_t *)&chassis.y, 4, 0xffff);
		HAL_UART_Transmit(&huart1, (uint8_t *)&chassis.ang, 4, 0xffff);
		HAL_UART_Transmit(&huart1, (uint8_t *)&chassis.ang1, 4, 0xffff);
		HAL_UART_Transmit(&huart1, (uint8_t *)&chassis.ang2, 4, 0xffff);
		HAL_UART_Transmit(&huart1, end_flag, 2, 0xffff);
	}
	//HAL_Delay(50);

//    测量速度与电流关系
//    chassis.state = CHASSIS_RUN;
//    chassis.v_set = intensity;
//    intensity += 100;
//    if(intensity >= 10000) intensity = 0;
//    HAL_Delay(500);
//    printf("%d,%f,%f;\r\n", intensity, chassis.v, chassis.w);



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
	imu.Handler();
	chassis.Handler();
	
	if(HAL_GetTick() % 500 == 0) HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

	//遥控器
#ifdef USE_REMOTE
    switch (remote.mode) {
        case 0: { //停止
            chassis.state = CHASSIS_STOP;
        }break;
        case 1: { //遥控
			float thresh = 0;
			if(remote.vertical < thresh && remote.vertical > -thresh){
				chassis.v_set = 0;
			}else chassis.v_set = remote.vertical * 0.1;
            if(remote.vertical < thresh && remote.vertical > -thresh){
				chassis.w_set = 0;
			}else chassis.w_set = remote.horizontal * (-0.1);
			chassis.state = CHASSIS_RUN;
        }break;
        case 2: { //位置控制器测试矩形
            const float l = 0.3;
			static float x_array[] = {0, l, l, 0};
			static float y_array[] = {0, 0, -l, -l};
			static int array_len = 4, index = 0;

			if(controller.reached == true){
				if(++index >= array_len) index = 0;
				controller.x_set = x_array[index];
				controller.y_set = y_array[index];
				controller.reached = false;
			}
			chassis.state = CHASSIS_RUN;
			controller.Handler();
        } break;
        case 3: { //上位机按照一定频率发送轨迹坐标
            controller.x_set = remote.x;
            controller.y_set = remote.y;
            chassis.state = CHASSIS_RUN;
            controller.Handler();
        } break;
    }

#else
    const float l = 0.3;
    static float x_array[] = {0, l, l, 0};
    static float y_array[] = {0, 0, -l, -l};
    static int array_len = 4, index = 0;

    if(controller.reached == true){
        if(++index >= array_len) index = 0;
        controller.x_set = x_array[index];
        controller.y_set = y_array[index];
		controller.reached = false;
    }
	controller.Handler();


#endif

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
				memcpy(remote.data, uart1_rx_buf, uart1_rx_len);
				remote.len = uart1_rx_len;
				//remote.process_data((uint16_t*)uart1_rx_buf, uart1_rx_len);
                uart1_state = 0;
                uart1_rx_len = 0;
            }else uart1_state = 0;
        }

		while(HAL_UART_Receive_IT(&huart1, (uint8_t *)(uart1_rx_buf + uart1_rx_len), 1) != HAL_OK){
			huart1.RxState = HAL_UART_STATE_READY;
			__HAL_UNLOCK(&huart1);
		}
    }

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if(hadc == &hadc3){
        ccd.sample_complete = true;
    }
}

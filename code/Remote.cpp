/**
  ******************************************************************************
  * @FileName			    Remote.h
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

#include "Remote.h"

extern uint8_t* uart1_rx_buf;

void Remote::process_data(uint16_t* data) {
    //解析数据
    mode = data[0];
    vertical = (float)(data[1] - 2048) / 4096 * 1;
    horizontal = (float)(data[2] - 2048) / 4096 * 1;

}

Remote::Remote(UART_HandleTypeDef *huart) {
    this->huart = huart;
}


void Remote::Handler() {
    if(HAL_GPIO_ReadPin(REMOTE_LOCK_GPIO_Port, REMOTE_LOCK_Pin) == GPIO_PIN_SET){
        state = REMOTE_CONNECTED;
    }else{
        state = REMOTE_DISCONNECTED;
    }
}
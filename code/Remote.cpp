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
#define REMOTE_LEN 3

uint32_t remote_cnt = 0;
uint16_t crc;
uint16_t crc1 = 0;
int crc_err = 0;

void Remote::process_data() {
	if(len == REMOTE_LEN * 2 + 2 + 2){
		//解析数据
		uint16_t mode = data[0];
		float vertical = (float)(data[2] - 2048) / 2048 * 1 - 0.07;
		float horizontal = (float)(data[1] - 2048) / 2048 * 1 - 0.07;
		crc = data[3];
		crc1 = 0;
		for(int i = 0; i < REMOTE_LEN; i++) crc1 += data[i];
		remote_cnt++;
		crc_err = crc1 - crc;
		if(mode <= 2 && data[1] < 4096 && data[2] < 4096 && crc1 == crc){
			this->mode = mode;
			this->vertical = -vertical * 0.1 + this->vertical * 0.9;
			this->horizontal = -horizontal * 0.1 + this->horizontal * 0.9;
		}
	}else if(len == 2 + 2 * 4 + 2 + 2){ //发送位置
        uint16_t mode = data[0];
		remote_cnt++;
        if(mode == 3){
            crc = data[5];
            crc1 = 0;
            for(int i = 0; i < 2 + 2 * 4; i++) crc1 += *((uint8_t*)data + i);
            crc_err = crc1 - crc;
            if(crc1 == crc){
                float x0 = *(float*)(data + 1);
                float y0 = *(float*)(data + 3);
                if(-2 < x0 && x0 < 2 && -2 < y0 && y0 < 2){
                    this->mode = mode;
                    this->x = x0 * 0.1 + x * 0.9;
                    this->y = y0 * 0.1 + y * 0.9;
                }
            }
        }
	}
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
    process_data();
}
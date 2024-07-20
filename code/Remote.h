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

#ifndef CONTROL_REMOTE_H
#define CONTROL_REMOTE_H

#include "hal.h"


typedef enum{
    REMOTE_DISCONNECTED=0,
    REMOTE_CONNECTED
} remote_state;

class Remote {
public:
    remote_state state = REMOTE_DISCONNECTED;

    uint16_t data[100];
    uint16_t len = 0;

    int mode = 0;
    float horizontal = 0; //-1 - 1
    float vertical = 0; //-1 - 1

    Remote(UART_HandleTypeDef *huart);

    void Handler();
    void process_data(); //接收中断

private:
    UART_HandleTypeDef *huart;
};


#endif //CONTROL_REMOTE_H

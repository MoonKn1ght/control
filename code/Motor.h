/**
  ******************************************************************************
  * @FileName			    Motor.h
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

#ifndef CONTROL_MOTOR_H
#define CONTROL_MOTOR_H

#include "hal.h"

typedef enum{
    MOTOR_STOP = 0,
    MOTOR_RUN,
    MOTOR_RELEASE
} motor_state;

class Motor{
public:
    motor_state state = MOTOR_STOP;
    int16_t intensity = 0;
    float ang = 0;  // (degree)
    float v = 0;    // m/s

};


#endif //CONTROL_MOTOR_H

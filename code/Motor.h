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

#include "module.h"

typedef enum{
    MOTOR_STOP = 0,
    MOTOR_RUN,
    MOTOR_RELEASE
} motor_state;

class Motor{
public:
    motor_state state;
    int16_t intensity;
    float ang;  // (degree)
    float v;    // m/s

};


#endif //CONTROL_MOTOR_H

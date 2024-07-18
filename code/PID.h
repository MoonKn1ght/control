/**
  ******************************************************************************
  * @FileName			    PID.h
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

#ifndef CONTROL_PID_H
#define CONTROL_PID_H

#include "config.h"

class PID {
public:
    float target = 0;
    float *feedback;
    float err = 0;

    float component_p = 0;
    float component_i = 0;
    float component_d = 0;
    float output = 0;

    float kp;
    float ki;
    float kd;

    float component_p_max;
    float component_i_max;
    float component_d_max;
    float output_max;

    PID(float kp, float ki, float kd,
        float component_p_max, float component_i_max, float component_d_max, float output_max);

    float calc();
};


#endif //CONTROL_PID_H

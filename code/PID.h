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
    float target;
    float *feedback;
    float err;

    float component_p;
    float component_i;
    float component_d;
    float output;

    float kp;
    float ki;
    float kd;

    float component_p_max;
    float component_i_max;
    float component_d_max;
    float output_max;

    float calc();
};


#endif //CONTROL_PID_H

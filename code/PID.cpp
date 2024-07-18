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

#include "PID.h"

float PID::calc(){
    float err = this->target - *(this->feedback);
    this->component_d = (err - this->err) * this->kd;
    this->component_i +=  err * this->ki;
    this->component_p = err * this->kp;
    INRANGE(this->component_p, this->component_p_max);
    INRANGE(this->component_i, this->component_i_max);
    INRANGE(this->component_d, this->component_d_max);
    this->err = err;
    this->output = this->component_p + this->component_i + this->component_d;
    INRANGE(this->output, this->output_max);

    return this->output;
}

PID::PID(float kp, float ki, float kd, float component_p_max, float component_i_max,
         float component_d_max, float output_max) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->component_p_max = component_p_max;
    this->component_i_max = component_i_max;
    this->component_d_max = component_d_max;
    this->output_max = output_max;
}

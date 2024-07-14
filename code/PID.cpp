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
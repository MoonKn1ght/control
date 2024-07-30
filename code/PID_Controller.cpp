/**
  ******************************************************************************
  * @FileName			    PID_Controller.h
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

#include "PID_Controller.h"

void PID_Controller::Handler() {
    if(state){
        err = target - (*feedback);
        w_out = kv * v_set * dir + kp * err;
        INRANGE(w_out, w_out_max);
        chassis->v_set = v_set;
        chassis->w_set = w_out;
    }
}

PID_Controller::PID_Controller(Chassis *chassis) {
    this->chassis = chassis;
    feedback = &(chassis->mid_point);
}

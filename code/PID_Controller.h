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

#ifndef CONTROL_PID_CONTROLLER_H
#define CONTROL_PID_CONTROLLER_H

#include "Chassis.h"

class PID_Controller {
public:
    Chassis* chassis;

    bool state = false;

    float target = 0;
    float *feedback;
    float err = 0;

    float kp = 0;
    float kv = 0.1375;
    float dir = -1;

    float v_set = 0.1;

    float w_out_max = 0.1;
    float w_out;

    PID_Controller(Chassis* chassis);
    void Handler();



};


#endif //CONTROL_PID_CONTROLLER_H

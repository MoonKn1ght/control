/**
  ******************************************************************************
  * @FileName			    Controller.h
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

#ifndef CONTROL_CONTROLLER_H
#define CONTROL_CONTROLLER_H

#include "Chassis.h"

class Controller {
public:
    Chassis* chassis;

    bool state = false;

    float x, y, ang;
    float x_set = 0, y_set = 0;
    float dist, ang_err;
    float x_err, y_err;

    float dist_th = 0.005, ang_th = 1;

    float kp1 = 1, kp2 = 0.1;

    float v_out = 0, w_out = 0;
    float v_out_max = 0.1, w_out_max = 0.1;

    bool reached = true;

    Controller(Chassis* chassis);

    void Handler();
};


#endif //CONTROL_CONTROLLER_H

/**
  ******************************************************************************
  * @FileName			    Tracking.h
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

#include "Tracking.h"

Tracking::Tracking(Chassis *chassis, Controller *controller, PID_Controller *pid_controller) {
    this->chassis = chassis;
    this->controller = controller;
    this->pid_controller = pid_controller;
}

void Tracking::Handler() {
    switch (state) {
        case 0:{
            chassis->state = CHASSIS_STOP;
            chassis->v_set = 0;
            chassis->w_set = 0;
        } break;
    }
}

void Tracking::calibrate_pos(Point r1, Point r2, ref_point_e ref_r, int dir) {
    Point ref_point = ref_points[ref_r];
    float x_diff = r2.x - r1.x;
    float y_diff = r2.y - r1.y;
    float dist;
    arm_sqrt_f32(x_diff * x_diff + y_diff * y_diff, &dist);
    //根据距离算参考向量

    //比较两个向量校正小车

    if(dir == 1){

    }

}

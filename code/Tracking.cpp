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
        case 1:{
            chassis->state = CHASSIS_RUN;
            chassis->v_set = 0.05;
//            if(chassis->inrange){
//                r2_set = true;
//                r2.x = chassis->x_line;
//                r2.y = chassis->y_line;
//                if(!r1_set){ //看到第一个点
//                    r1_set = true;
//                    r1.x = chassis->x_line;
//                    r1.y = chassis->y_line;
//                }else if(calc_dist(r1, r2) >= ref_dist){
//                    //先测试能否校准零位
//                    calibrate_pos(r1, r2, POINT_B);
//                    chassis->v_set = 0;
//                    state = 2;
//                }
//            }
            if(chassis->inrange){
                if(!r1_set){ //看到第一个点
                    r1_set = true;
                    r1.x = chassis->x_line;
                    r1.y = chassis->y_line;

                    pid_controller->dir = -1;
                    pid_controller->state = 1;
                }else{
                    r2.x = chassis->x_line;
                    r2.y = chassis->y_line;
                }
            } else{
                if(r1_set){ //看不到第二个点
                    r2_set = true;
                    pid_controller->state = 0;
                    chassis->v_set = 0;
                    chassis->w_set = 0;
                    calibrate_pos2(r1, r2, POINT_A, POINT_B);
                    state = 2;
                }
            }
        } break;
    }
}

void Tracking::calibrate_pos(Point r1, Point r2, ref_point_e ref_r) {
    Point ref_r1 = ref_points[ref_r];
    //根据距离算向量
    float temp_cos = dist / 2 / TRACK_R;
    float temp_sin;
    arm_sqrt_f32(1 - temp_cos * temp_cos, &temp_sin);
    float dx = dist * temp_sin;
    float dy = dist * temp_cos;
    switch (ref_r) {
        case POINT_A:{
            dy = -dy;
        } break;
        case POINT_B:{

        } break;
        case POINT_C:{
            dx = -dx;
        } break;
        case POINT_D:{
            dx = -dx;
            dy = -dy;
        } break;

    }
    Point ref_r21 = {dx, dy};
    Point r21 = {r2.x - r1.x, r2.y - r1.y};
    //计算两向量夹角
    float ang = acosf((ref_r21.x * r21.x + ref_r21.y + r21.y) / (dist * dist)) / M_PI * 180;
    //计算向量旋转方向
    float dir = r21.x * ref_r21.y - r21.y * ref_r21.x;
    if(dir < 0) ang = -ang;
    //比较两个向量校正小车
    chassis->x += ref_r1.x - r1.x;
    chassis->y += ref_r1.y - r1.y;
    chassis->ang += ang;
    MOD(chassis->ang, 360);



}

float Tracking::calc_dist(Point r1, Point r2) {
    float x_diff = r2.x - r1.x;
    float y_diff = r2.y - r1.y;
    arm_sqrt_f32(x_diff * x_diff + y_diff * y_diff, &dist);
    return dist;
}

void Tracking::calibrate_pos2(Point r1, Point r2, ref_point_e ref1, ref_point_e ref2) {
    Point ref_r1 = ref_points[ref1];
    Point ref_r2 = ref_points[ref2];

    //calc_dist(r1, r2);


}


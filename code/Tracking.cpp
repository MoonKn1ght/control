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

//#define QUESTION_2
#define QUESTION_3

Tracking::Tracking(Chassis *chassis, Controller *controller, PID_Controller *pid_controller) {
    this->chassis = chassis;
    this->controller = controller;
    this->pid_controller = pid_controller;
}

void Tracking::Handler() {
#ifdef QUESTION_2
    switch (state) {
        case 0:{
            chassis->state = CHASSIS_STOP;
            chassis->v_set = 0;
            chassis->w_set = 0;
        } break;
        case 1:{
            chassis->state = CHASSIS_RUN;
            controller->x_set = 1.0;
            controller->y_set = 0;
            controller->state = 1;
            if(chassis->inrange){
                if(!r1_set){ //看到第一个点
                    r1_set = true;
                    r1.x = chassis->x_line;
                    r1.y = chassis->y_line;

                    controller->state = 0;
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
                    calibrate_pos2(r1, r2, POINT_B, POINT_C);
                    state++;
                }
            }
        } break;
        case 2:{
            controller->y_set = -0.8;
            controller->x_set = -0.0;
            controller->state = 1;
            if(controller->reached || chassis->inrange){
                controller->state = 0;
                chassis->v_set = 0;
                chassis->w_set = 0;
                state++;
            }
        } break;
        case 3:{
            if(!chassis->inrange){
                chassis->w_set = 0.05;
            }else{
                chassis->w_set = 0;
                r1_set = false;
                r2_set = false;
                pid_controller->state = 1;
                state++;
            }
        } break;
        case 4:{
            if(!chassis->inrange){
                pid_controller->state = 0;
                chassis->w_set = 0;
                chassis->v_set = 0;
                state++;
            }
        } break;
    }
#endif
#ifdef QUESTION_3
    switch (state) {
        case 0:{
            chassis->state = CHASSIS_STOP;
            chassis->v_set = 0;
            chassis->w_set = 0;
        } break;
        case 1:{ // A->C->B
            chassis->state = CHASSIS_RUN;
            if(!init_pos){
                chassis->x = init_x;
                chassis->y = init_y;
                init_pos = 1;
            }
            controller->x_set = 1;
            controller->y_set = -0.8;
            controller->state = 1;

            if(chassis->inrange){
                if(!r1_set){ //看到第一个点
                    r1_set = true;
                    r1.x = chassis->x_line;
                    r1.y = chassis->y_line;

                    controller->state = 0;
                    pid_controller->dir = 1;
                    pid_controller->state = 1;
                }else{
                    pid_controller->state = 1;
                    r2.x = chassis->x_line;
                    r2.y = chassis->y_line;
                }
            } else{
                if(r1_set && ABS(chassis->y) < 0.2){ //看不到第二个点
                    r2_set = true;
                    pid_controller->state = 0;
                    chassis->v_set = 0;
                    chassis->w_set = 0;
                    calibrate_pos2(r1, r2, POINT_C, POINT_B);
                    state++;
                }else if(controller->reached){
                    pid_controller->state = 0;
                    controller->state = 0;
                    chassis->w_set = 0.05;
                }
            }
        } break;
        case 2:{ // B->D
            controller->y_set = -0.8;
            controller->x_set = -0.0;
            controller->state = 1;
            if(chassis->inrange && ABS(chassis->x - controller->x_set) < 0.1){
                controller->state = 0;
                chassis->v_set = 0;
                chassis->w_set = 0;
                state++;
            }
        } break;
        case 3: {
            if(!chassis->inrange){
                chassis->w_set = -0.05;
            }else{
                chassis->w_set = 0;
                r1_set = true;
                r2_set = false;
                r1.x = chassis->x_line;
                r1.y = chassis->y_line;
                pid_controller->dir = -1;
                pid_controller->state = 1;
                state++;
            }
        } break;
        case 4:{ //D->A
            if(!chassis->inrange){
                r2_set = true;
                pid_controller->state = 0;
                chassis->v_set = 0;
                chassis->w_set = 0;
                calibrate_pos2(r1, r2, POINT_D, POINT_A);
                r1_set = false;
                r2_set = false;
                state = 1;
            }else{
                r2.x = chassis->x_line;
                r2.y = chassis->y_line;
            }

        } break;
    }
#endif
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
    chassis->x += ref_r2.x - r2.x;
    chassis->y += ref_r2.y - r2.y;

    Point ref_r21 = {ref_r2.x - ref_r1.x, ref_r2.y - ref_r1.y};
    Point r21 = {r2.x - r1.x, r2.y - r1.y};
    //计算两向量夹角
    float ang = acosf((ref_r21.x * r21.x + ref_r21.y + r21.y) / (dist * dist)) / M_PI * 180;
    //计算向量旋转方向
    float dir = r21.x * ref_r21.y - r21.y * ref_r21.x;
    if(dir < 0) ang = -ang;
    chassis->ang += ang;
    MOD(chassis->ang, 360);

    /*if(!init_pos){
        init_x = ref_r2.x - r2.x;
        init_y = ref_r2.y - r2.y;
        init_ang = ang;
        MOD(init_ang, 360);
        init_pos = true;
    }*/



}


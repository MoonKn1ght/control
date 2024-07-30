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

#ifndef CONTROL_TRACKING_H
#define CONTROL_TRACKING_H

#include "device.h"

#define TRACK_L 1.0
#define TRACK_R 0.4


typedef struct{
    float x;
    float y;
} Point;

typedef enum{
    POINT_A = 0,
    POINT_B,
    POINT_C,
    POINT_D
} ref_point_e;

class Tracking {
public:
    Chassis* chassis;
    Controller* controller;
    PID_Controller* pid_controller;

    int state = 0; //0: stop; 1: task1; 2: task2; 3: task3; 4. task4
    float v_set = 0.05;

    Tracking(Chassis* chassis, Controller* controller, PID_Controller* pid_controller);
    void Handler();

private:
    Point ref_points[4] = {{0.0, 0.0},
                           {TRACK_L, 0.0},
                           {TRACK_L, -TRACK_R * 2},
                           {0, -TRACK_R * 2}};

    Point r1 = {0.0, 0.0}, r2 = {0.0, 0.0};
    bool r1_set = false, r2_set = false;

    float ref_dist = 0.03;

    float dist = 0;
    float calc_dist(Point r1, Point r2);

    //r1: 位置参考点，对应ref_r
    void calibrate_pos(Point r1, Point r2, ref_point_e ref_r);

};


#endif //CONTROL_TRACKING_H

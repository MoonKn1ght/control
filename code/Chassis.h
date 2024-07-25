/**
  ******************************************************************************
  * @FileName			    Chassis.h
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

#ifndef CONTROL_CHASSIS_H
#define CONTROL_CHASSIS_H

#include "module.h"
#include "algorithm.h"

typedef enum{
    CHASSIS_STOP = 0,
    CHASSIS_RUN,
    CHASSIS_RELEASE
} chassis_state;

class Chassis {
public:
    chassis_state state;

    N20_Motor* motorL;
    N20_Motor* motorR;

    IMU* imu;

    float wheel_seperation;

    Chassis(N20_Motor* left_motor, N20_Motor* right_motor, IMU* imu,
            float wheel_seperation);


    float v = 0;
    float w = 0;
    float ang = 0;
    float ang1 = 0, ang2 = 0, ang3, ang4;
    //float R = 0;

    float x = 0, y = 0;

    float v_set = 0, w_set = 0; //0-10000


    void Handler();

private:
    float scale1 = 1.00699306;
    float scale2 = 1.01982999;

};


#endif //CONTROL_CHASSIS_H

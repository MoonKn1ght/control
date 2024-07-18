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

#include "Chassis.h"

Chassis::Chassis(N20_Motor *left_motor, N20_Motor *right_motor, IMU *imu, float wheel_seperation) {
    this->motorL = left_motor;
    this->motorR = right_motor;
    this->imu = imu;
    this->wheel_seperation = wheel_seperation;
}

void Chassis::Handler() {
    v = (motorL->v + motorR->v) / 2;
    w = (motorR->v - motorL->v) / wheel_seperation; //TODO 改成IMU


    switch (this->state){
        case CHASSIS_STOP:{
            this->motorL->state = MOTOR_STOP;
            this->motorR->state = MOTOR_STOP;
            ang = 0; ang1 = 0; ang2 = 0;
            x = 0; y = 0;
        }  break;
        case CHASSIS_RUN:{
            float output1 = v_set - w_set;
            float output2 = v_set + w_set;
            INRANGE(output1, 10000);
            INRANGE(output2, 10000);
            this->motorL->intensity = this->motorL->intensity * 0.9 + (int)(output1 * 0.1);
            this->motorR->intensity = this->motorR->intensity * 0.9 + (int)(output2 * 0.1);

            this->motorL->state = MOTOR_RUN;
            this->motorR->state = MOTOR_RUN;

            ang1 += w / MAIN_LOOP_FREQ;
            ang2 += imu->wz / MAIN_LOOP_FREQ;
            ang = 0.33 * ang1 + (1 - 0.33) * ang2;


            float temp_cos, temp_sin;
            arm_sin_cos_f32(ang, &temp_sin, &temp_cos);
            x += v * temp_cos / MAIN_LOOP_FREQ;
            y += v * temp_sin / MAIN_LOOP_FREQ;
        } break;
        case CHASSIS_RELEASE:{
            this->motorL->state = MOTOR_RELEASE;
            this->motorR->state = MOTOR_RELEASE;
        } break;
        default:
            break;
    }
}



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

#define dt (0.00100)
float Kalman_Filter_x(float angle, float Accel,float Gyro)
{

    static float angle_dot;
    //static float angle;
    float Q_angle=0.001; // 过程噪声的协方差
    float Q_gyro=0.003;	//0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
    float R_angle=0.5;		// 测量噪声的协方差 既测量偏差
    char  C_0 = 1;
    static float Q_bias, Angle_err;
    static float PCt_0, PCt_1, E;
    static float K_0, K_1, t_0, t_1;
    static float Pdot[4] ={0,0,0,0};
    static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
    angle+=(Gyro - Q_bias) * dt; //先验估计
    Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

    Pdot[1]=-PP[1][1];
    Pdot[2]=-PP[1][1];
    Pdot[3]=Q_gyro;
    PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
    PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;

    Angle_err = Accel - angle;	//zk-先验估计

    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    angle	+= K_0 * Angle_err;	 //后验估计
    Q_bias	+= K_1 * Angle_err;	 //后验估计
    angle_dot   = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
    return angle;
}

void Chassis::Handler() {
    v = v * 0.9 + (motorL->v + motorR->v) / 2 * 0.1;
    w = w * 0.9 + (motorR->v - motorL->v) / 2 * 0.1; //差速


    switch (this->state){
        case CHASSIS_STOP:{
            this->motorL->state = MOTOR_STOP;
            this->motorR->state = MOTOR_STOP;
            ang = 0; ang1 = 0; ang2 = 0;
            x = 0; y = 0;
        }  break;
        case CHASSIS_RUN:{
            float output1 = (v_set - w_set) * 58327;
            float output2 = (v_set + w_set) * 58327;
            float thresh = 404;
            if(output1 > 0) output1 += thresh;
            else if(output1 < 0) output1 -= thresh;
            if(output2 > 0) output2 += thresh;
            else if(output2 < 0) output2 -= thresh;

            INRANGE(output1, 10000);
            INRANGE(output2, 10000);
            this->motorL->intensity = this->motorL->intensity * 0.95 + (int)(output1 * 0.05);
            this->motorR->intensity = this->motorR->intensity * 0.95 + (int)(output2 * 0.05);

            this->motorL->state = MOTOR_RUN;
            this->motorR->state = MOTOR_RUN;

            ang1 += (motorR->v - motorL->v) / wheel_seperation / M_PI * 180 / MAIN_LOOP_FREQ;
            ang2 += imu->wz / MAIN_LOOP_FREQ;
            ang3 = ang1 * scale1;
            ang4 = ang2 * scale2;
            //ang = (ang1 + ang4) / 2;
            //ang = ang1 / 3525 * 3600;
            ang = ang2 / 3540 * 3600;
           // ang = ang4;
            MOD(ang, 360);

//            TODO 速度用加速度计修正
//            v1 = v;
//            v2 = Kalman_Filter_x(v2, v, imu->ax * 9.8);
//            v3 += imu->ax_correct * 9.8 / MAIN_LOOP_FREQ;
            float temp_cos, temp_sin;
            arm_sin_cos_f32(ang, &temp_sin, &temp_cos);
            x += (motorL->v + motorR->v) / 2 * temp_cos / MAIN_LOOP_FREQ;
            y += (motorL->v + motorR->v) / 2 * temp_sin / MAIN_LOOP_FREQ;
        } break;
        case CHASSIS_RELEASE:{
            this->motorL->state = MOTOR_RELEASE;
            this->motorR->state = MOTOR_RELEASE;
        } break;
        default:
            break;
    }
}



/**
  ******************************************************************************
  * @FileName			    IMU.h
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

#ifndef CONTROL_IMU_H
#define CONTROL_IMU_H

//#define USE_IMU660RA
#define USE_IMU963RA

#include "hal.h"

typedef enum{
    IMU_NOT_INIT,
    IMU_RUN,
    IMU_ERR,
    IMU_CALIBRATE
} IMU_state;


class IMU {
public:
    IMU_state state;

    float ax;
    float ay;
    float az;
    float wx;
    float wy;
    float wz;

    float gx = 0;
    float gy = 0;
    float gz = 0;

    float correct_ang = 0;
    float ax_correct;
    float ay_correct;

    void init();

    void Handler();

private:
    float k_imu = 0;

    float ax_offset = 0;
    float ay_offset = 0;
    float az_offset = 0;
    float wx_offset = 0;
    float wy_offset = 0;
    float wz_offset = 0;

    void get_data();

};


#endif //CONTROL_IMU_H

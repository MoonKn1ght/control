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

#include "hal.h"

typedef enum{
    IMU_NOT_INIT,
    IMU_RUN,
    IMU_ERR
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

    void init();

    void Handler();

private:
    float k_imu = 0;

};


#endif //CONTROL_IMU_H

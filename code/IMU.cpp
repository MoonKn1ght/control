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

#include "IMU.h"
#include "imu660.h"

void IMU::get_data(){
    imu660ra_get_acc();                                                         // 获取 IMU660RA 的加速度测量数值
    imu660ra_get_gyro();

    this->ax = this->ax * k_imu + (1 - k_imu) * (imu660ra_acc_transition(imu660ra_acc_x) - ax_offset);
    this->ay = this->ay * k_imu + (1 - k_imu) * (imu660ra_acc_transition(imu660ra_acc_y) - ay_offset);
    this->az = this->az * k_imu + (1 - k_imu) * (imu660ra_acc_transition(imu660ra_acc_z) - az_offset);
    this->wx = this->wx * k_imu + (1 - k_imu) * (imu660ra_gyro_transition(imu660ra_gyro_x) - wx_offset);
    this->wy = this->wy * k_imu + (1 - k_imu) * (imu660ra_gyro_transition(imu660ra_gyro_y) - wy_offset);
    this->wz = this->wz * k_imu + (1 - k_imu) * (imu660ra_gyro_transition(imu660ra_gyro_z) - wz_offset);
}
#define IMU_CALIBRATE_N 1000
void IMU::init() {
    if(imu660ra_init()) this->state = IMU_ERR;
    else this->state = IMU_CALIBRATE;

    HAL_Delay(10);
    if(this->state == IMU_CALIBRATE){
        //calibrate
        float ax_mean = 0, ay_mean = 0, az_mean = 0, wx_mean = 0, wy_mean = 0, wz_mean = 0;
        for(int i = 0; i < IMU_CALIBRATE_N; i++){
           get_data();
           ax_mean += ax;
           ay_mean += ay;
           az_mean += az;
           wx_mean += wx;
           wy_mean += wy;
           wz_mean += wz;
           HAL_Delay(1);
        }
        ax_offset = ax_mean / IMU_CALIBRATE_N;
        ay_offset = ay_mean / IMU_CALIBRATE_N;
        az_offset = az_mean / IMU_CALIBRATE_N;
        wx_offset = wx_mean / IMU_CALIBRATE_N;
        wy_offset = wy_mean / IMU_CALIBRATE_N;
        wz_offset = wz_mean / IMU_CALIBRATE_N;
        state = IMU_RUN;
    }
}

void IMU::Handler() {
    if(this->state == IMU_RUN){
        get_data();
    }
}

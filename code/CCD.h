/**
  ******************************************************************************
  * @FileName			    CCD.h
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

#ifndef CONTROL_CCD_H
#define CONTROL_CCD_H

#include "hal.h"

class CCD {
public:
    int integrate_time = 10;
    uint16_t data[128];
    uint16_t data_norm[128];
    int sample_t = 0;
    bool sample_complete = false;

    uint8_t bin_ccd[128];
    uint16_t dir;	//黑线位置
    uint16_t last_dir;
    uint16_t threshold;
//    float kp = -0.001;
    int Left;
	int Right;
	int max_data = 0;
	int min_data = 3000;

    void init();
    void Handler(); //放在主任务循环里
    void SI_send(); //放在定时器中断里
    void GetThreshold();
    void Binarization();
    void Filter();
    void GetDirection();
    void Normalization();



private:
    int state = 0;
};



#endif //CONTROL_CCD_H

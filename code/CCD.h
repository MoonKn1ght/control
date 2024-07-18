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
    int integrate_time = 50;
    uint16_t data[128];
    uint8_t bin_ccd[128];
    uint16_t dir;	//黑线位置
    uint16_t last_dir;
    uint16_t threshold;

    void init();
    void Handler(); //放在主任务循环里
    void SI_send(); //放在定时器中断里
    void GetThreshold();
    void Binarization();
    void Filter();
    uint16_t GetDirection();
    void Line_tracing();



private:
    int state = 0;
};



#endif //CONTROL_CCD_H

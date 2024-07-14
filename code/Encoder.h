/**
  ******************************************************************************
  * @FileName			    Encoder.h
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

#ifndef CONTROL_ENCODER_H
#define CONTROL_ENCODER_H


class Encoder{
public:
    float ang;

    void Handler(); //更新角度
};


#endif //CONTROL_ENCODER_H

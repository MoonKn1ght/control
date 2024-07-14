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

#include "Encoder.h"

void Encoder::Handler() {

}

Encoder::Encoder(TIM_HandleTypeDef *htim) {
    this->htim = htim;
}

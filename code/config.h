/**
  ******************************************************************************
  * @FileName			    config.h
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

#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

//#include "math.h"

#define MAIN_LOOP_FREQ 5000

#ifndef M_PI
#define M_PI		3.14159265358979323846
#endif

#define INRANGE(num, max)   if(num > max) num = max; else if(num < -max) num = -max;
#define GETBIT(num, b)  ((num>>b) & 1)
#define MAP(num, frommin, frommax, tomin, tomax) (((float)(num) - frommin) / (frommax - frommin) * (tomax - tomin) + tomin)
#define ABS(n) ((n>=0)? n:-n)


	
#endif //CONTROL_CONFIG_H

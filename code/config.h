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

#ifdef __cplusplus
extern "C" {
#endif
	
//#include "math.h"
#include "arm_math.h"
#include "string.h"


#define MAIN_LOOP_FREQ 1000

#ifndef M_PI
#define M_PI		3.14159265358979323846
#endif

#define INRANGE(num, max)   if(num > max) num = max; else if(num < -max) num = -max;
#define GETBIT(num, b)  ((num>>b) & 1)
#define MAP(num, frommin, frommax, tomin, tomax) (((float)(num) - frommin) / (frommax - frommin) * (tomax - tomin) + tomin)
#define ABS(n) ((n>=0)? n:-n)
#define MOD(ang, n)     if(ang >= n){ang -= (int)(ang / n) * n;}else if(ang < 0){ang -= (int)(ang / n) * n - n;}
#define NORMALIZE_ANGLE180(angle) {\
    angle=angle>180?angle-360:(angle<-180?angle+360:angle);\
}

#ifdef __cplusplus
}
#endif
	
#endif //CONTROL_CONFIG_H

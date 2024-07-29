/**
  ******************************************************************************
  * @FileName			    Controller.h
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

#include "Controller.h"

Controller::Controller(Chassis *chassis) {
    this->chassis = chassis;

}

void Controller::Handler(){
    if(state){
        x = chassis->x;
        y = chassis->y;
        ang = chassis->ang;
        x_err = x_set - x;
        y_err = y_set - y;

        if((x_err > -dist_th && x_err < dist_th) && (y_err > -dist_th && y_err < dist_th)){
            v_out = 0;
            w_out = 0;
            chassis->v_set = v_out;
            chassis->w_set = w_out;
            reached = true;
            return;
        }
        reached = false;
        ang_err = atan2(y_err, x_err) / M_PI * 180 - ang;
        if(ang_err < -180) ang_err += 360;
        else if(ang_err > 180) ang_err -= 360;
        arm_sqrt_f32(x_err * x_err + y_err * y_err, &dist);
        //判断目标点在前方还是后方，先计算矢径的方向
        if(ang_err > 90 || ang_err < -90) dist = -dist;

        if(ang_err > ang_th || ang_err < -ang_th){ //优先转向
            v_out = 0;
        }else v_out = kp1 * dist;
        w_out = kp2 * chassis->wheel_seperation * ang_err;
        INRANGE(v_out, v_out_max);
        INRANGE(w_out, w_out_max);

        chassis->v_set = v_out;
        chassis->w_set = w_out;
    }


}
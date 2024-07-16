/**
  ******************************************************************************
  * @FileName			    N20_Motor.h
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

#ifndef CONTROL_N20_MOTOR_H
#define CONTROL_N20_MOTOR_H

#include "Motor.h"
#include "Encoder.h"

class N20_Motor: public Motor {
public:
    Encoder* encoder;

    float reduction_rate;
    float radius;
    float deadzone;

    N20_Motor(TIM_HandleTypeDef* htim, uint32_t pwm_channel1, uint32_t pwm_channel2,
              Encoder* encoder,
              float reduction_rate, float radius, float deadzone);
    void init();
    void Handler();

private:
    TIM_HandleTypeDef* htim;
    uint32_t pwm_channel1;
    uint32_t pwm_channel2;


    void pwm_set_duty(uint32_t channel, uint16_t duty);

    void measure_parameter();
    void output_intensity();
};


#endif //CONTROL_N20_MOTOR_H

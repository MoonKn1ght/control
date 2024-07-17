/**
  ******************************************************************************
  * @FileName			    N20_h
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

#include "N20_Motor.h"

void N20_Motor::Handler() {
    measure_parameter();
    output_intensity();
}

void N20_Motor::measure_parameter() {
    float inc = encoder->pulse / this->reduction_rate * 360;
	inc = -inc;
    encoder->pulse = 0;
    this->v = this->v * 0.98 + inc / 180 * M_PI * MAIN_LOOP_FREQ * 0.02;
    this->ang += inc;
    if(this->ang >= 360){
        this->ang -= (int)(this->ang / 360) * 360;
    }else if(this->ang < 0){
        this->ang -= (int)(this->ang / 360) * 360 - 360;
    }
}

#define PWM_DUTY_MAX 10000
void N20_Motor::pwm_set_duty(uint32_t channel, uint16_t duty){ //0-10000 TODO cube配置
    switch (channel) {
        case TIM_CHANNEL_1: htim->Instance->CCR1 = duty; break;
        case TIM_CHANNEL_2: htim->Instance->CCR2 = duty; break;
        case TIM_CHANNEL_3: htim->Instance->CCR3 = duty; break;
        case TIM_CHANNEL_4: htim->Instance->CCR4 = duty; break;
        default: break;
    }
}

void N20_Motor::output_intensity() {
    INRANGE(this->intensity, PWM_DUTY_MAX);
    switch(this->state){
        case MOTOR_STOP: {
            this->intensity = 0;
            //gpio_set_level(this->IN1_pin, 0);
            //gpio_set_level(this->IN2_pin, 0);
            pwm_set_duty(pwm_channel1, PWM_DUTY_MAX - 1);
            pwm_set_duty(pwm_channel2, PWM_DUTY_MAX - 1);

        }break;
        case MOTOR_RUN: {
            if(this->intensity >= 0){
                //gpio_set_level(this->IN1_pin, 1);
                //gpio_set_level(this->IN2_pin, 0);
                pwm_set_duty(pwm_channel1, PWM_DUTY_MAX - (int)MAP(ABS(this->intensity), 0, PWM_DUTY_MAX, this->deadzone, PWM_DUTY_MAX));
                pwm_set_duty(pwm_channel2, PWM_DUTY_MAX - 1);
            }else{
                //gpio_set_level(this->IN1_pin, 0);
                //gpio_set_level(this->IN2_pin, 1);
                pwm_set_duty(pwm_channel1, PWM_DUTY_MAX - 1);
                pwm_set_duty(pwm_channel2, PWM_DUTY_MAX - (int)MAP(ABS(this->intensity), 0, PWM_DUTY_MAX, this->deadzone, PWM_DUTY_MAX));
            }

        }break;
        case MOTOR_RELEASE: {
            this->intensity = 0;
            //gpio_set_level(this->IN1_pin, 1);
            //gpio_set_level(this->IN2_pin, 1);
            pwm_set_duty(pwm_channel1, 0);
            pwm_set_duty(pwm_channel2, 0);
        }break;
    }
}

N20_Motor::N20_Motor(TIM_HandleTypeDef *htim, uint32_t pwm_channel1, uint32_t pwm_channel2, Encoder *encoder,
                     float reduction_rate, float radius, float deadzone) {
    this->htim = htim;
    this->pwm_channel1 = pwm_channel1;
    this->pwm_channel2 = pwm_channel2;
    this->encoder = encoder;
    this->reduction_rate = reduction_rate;
    this->radius = radius;
    this->deadzone = deadzone;

}

void N20_Motor::init() {
    HAL_TIM_PWM_Start(htim, pwm_channel1);
    HAL_TIM_PWM_Start(htim, pwm_channel2);
}



/**
  ******************************************************************************
  * @FileName			    Chassis.h
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

#include "Chassis.h"

float ccd_norm[128] = {
        1.2870592260559972, 1.2648513515347348, 1.2233098296859939, 1.1887824932296176, 1.158897621206234, 1.160736277983108, 1.1506209789308148, 1.1422216875537974, 1.1241699662532234, 1.1080059781492437, 1.0845954838043605, 1.0755111689342371, 1.0625902369880764, 1.0532066864988638, 1.0430060219036525, 1.033213535397254, 1.0258783767959903, 1.024650788336391, 1.0205240889733236, 1.0203520421617636, 1.0174206713786276, 1.0148192429801175, 1.0128817312646912, 1.0120848994195895, 1.013205441320869, 1.013105223242896, 1.0116120131676096, 1.0116927795661554, 1.008348685455276, 1.0093078680843117, 1.0075399676296595, 1.0079670245162877, 1.0068314467514126, 1.0075628397448004, 1.0066830726833582, 1.0060973741238588, 1.0047688324804755, 1.0050077218127478, 1.00318297362638, 1.0034664813793481, 1.0027373055133346, 1.0032964174998906, 1.002582466679666, 1.0028958491394346, 1.0023221528386286, 1.0023824574275244, 1.001556909334388, 1.0023937765752367, 1.0033454984742491, 1.0036328805208894, 1.0027787797599925, 1.003046946799558, 1.000443237697232, 1.0013723704167004, 1.0005108938710567, 1.0005635813381462, 1.0001464104404474, 1.000969633966161, 1.0001089095460023, 1.0005522438155252, 1.0, 1.0008229824224413, 1.0013310124223251, 1.0020430616742786, 1.0020430616742786, 1.0027373055133346, 1.0022240760669738, 1.0011464969901247, 1.000386924192374, 1.0010712284901788, 1.000386924192374, 1.001266950135495, 1.0024315688088086, 1.002797660069299, 1.0024654704476363, 1.0029299020982767, 1.0025032680875057, 1.0030016726841808, 1.0029373964577761, 1.003595117722459, 1.0030658372084027, 1.0017038956746318, 1.0014854051364313, 1.0017792593461272, 1.0017792593461272, 1.0023937765752367, 1.0042913943565739, 1.005049384089441, 1.004302756657947, 1.005489637842602, 1.0054022671927747, 1.0029638374527234, 1.0029827247312675, 1.0040642623975085, 1.0040339778868808, 1.005630065248529, 1.0076657771149118, 1.008784131633661, 1.0085244071846986, 1.0089867040033649, 1.0098054050527299, 1.0109132272590082, 1.0112586593596429, 1.0133634566042669, 1.0130898068366891, 1.0121310583350878, 1.0108940401221613, 1.0124466514106634, 1.0133596004936931, 1.0153571318554713, 1.0156706821172705, 1.0162094741945356, 1.017976887240655, 1.0198479663756619, 1.0193172005391005, 1.0215615082381784, 1.0246311387605207, 1.02724349122853, 1.0283064726943312, 1.0299569531789083, 1.0357612382845565, 1.054069480736888, 1.0807792358550483, 1.1079828552476714, 1.0907299871795204, 1.08226822747299, 1.1012161268160363, 1.1127512877358572};

void Chassis::process()
{
    // Normalization
    int i;
    for(i = 0; i < 128; ++i)
    {
        data_norm[i] = (float)ccd->data[i] * ccd_norm[i];
    }

    // Get the threshold
    float min_data = 3000;
    float max_data = 0;
    for(i = 0; i < 128; ++i)
    {
        if(data_norm[i] > max_data)
        {
            max_data = data_norm[i];
        }
        else if(data_norm[i] < min_data)
        {
            min_data = data_norm[i];
        }

    }
    if(max_data - min_data > 700)
    {
        threshold = (max_data + min_data)/2;
    }
    else threshold = 0;

    // Binarization
    for(i = 0; i < 128; ++i)
    {
        if(data_norm[i] > threshold)
        {
            bin_ccd[i] = 1;
        }
        else
        {
            bin_ccd[i] = 0;
        }
    }


    //TODO 滤波
    //求轨道中点
    uint32_t mid = 0;
    uint32_t n = 0;
    for(i = 3; i < 125; i++) {
        if(bin_ccd[i] == 0) {
            n++;
            mid += i;
        }
    }
    if(n < 3) inrange = false;
    else {
        inrange = true;
        mid_point = mid_point * 0.9 + ((float) mid / (float) n - 64) * 0.1;
    }

    //计算轨道坐标
    if(inrange) {
        float temp_cos = 0, temp_sin = 0;
        arm_sin_cos_f32(ang, &temp_sin, &temp_cos);
        x_line = x + D * temp_cos + mid_point * dl * temp_sin;
        y_line = y + D * temp_sin - mid_point * dl * temp_cos;
    }


}

Chassis::Chassis(N20_Motor *left_motor, N20_Motor *right_motor, IMU *imu, CCD* ccd, float wheel_seperation) {
    this->motorL = left_motor;
    this->motorR = right_motor;
    this->imu = imu;
    this->ccd = ccd;
    this->wheel_seperation = wheel_seperation;
}

#define dt (0.00100)
float Kalman_Filter_x(float angle, float Accel,float Gyro)
{

    static float angle_dot;
    //static float angle;
    float Q_angle=0.001; // 过程噪声的协方差
    float Q_gyro=0.003;	//0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
    float R_angle=0.5;		// 测量噪声的协方差 既测量偏差
    char  C_0 = 1;
    static float Q_bias, Angle_err;
    static float PCt_0, PCt_1, E;
    static float K_0, K_1, t_0, t_1;
    static float Pdot[4] ={0,0,0,0};
    static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
    angle+=(Gyro - Q_bias) * dt; //先验估计
    Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

    Pdot[1]=-PP[1][1];
    Pdot[2]=-PP[1][1];
    Pdot[3]=Q_gyro;
    PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
    PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;

    Angle_err = Accel - angle;	//zk-先验估计

    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    angle	+= K_0 * Angle_err;	 //后验估计
    Q_bias	+= K_1 * Angle_err;	 //后验估计
    angle_dot   = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
    return angle;
}

void Chassis::Handler() {
    v = v * 0.9 + (motorL->v + motorR->v) / 2 * 0.1;
    w = w * 0.9 + (motorR->v - motorL->v) / 2 * 0.1; //差速


    switch (this->state){
        case CHASSIS_STOP:{
            this->motorL->state = MOTOR_STOP;
            this->motorR->state = MOTOR_STOP;
            ang = 0; ang1 = 0; ang2 = 0;
            x = 0; y = 0;
        }  break;
        case CHASSIS_RUN:{
            float output1 = (v_set - w_set) * 58327;
            float output2 = (v_set + w_set) * 58327;
            float thresh = 404;
            if(output1 > 0) output1 += thresh;
            else if(output1 < 0) output1 -= thresh;
            if(output2 > 0) output2 += thresh;
            else if(output2 < 0) output2 -= thresh;

            INRANGE(output1, 10000);
            INRANGE(output2, 10000);
            this->motorL->intensity = this->motorL->intensity * 0.95 + (int)(output1 * 0.05);
            this->motorR->intensity = this->motorR->intensity * 0.95 + (int)(output2 * 0.05);

            this->motorL->state = MOTOR_RUN;
            this->motorR->state = MOTOR_RUN;

            ang1 += (motorR->v - motorL->v) / wheel_seperation / M_PI * 180 / MAIN_LOOP_FREQ;
            ang2 += imu->wz / MAIN_LOOP_FREQ;
            //ang = (ang1 + ang4) / 2;
            //ang = ang1 / 3525 * 3600;
            ang = ang2 / 3540 * 3600; //TODO 待修正
           // ang = ang4;
            MOD(ang, 360);

//            TODO 速度用加速度计修正
//            v1 = v;
//            v2 = Kalman_Filter_x(v2, v, imu->ax * 9.8);
//            v3 += imu->ax_correct * 9.8 / MAIN_LOOP_FREQ;
            float temp_cos = 0, temp_sin = 0;
            arm_sin_cos_f32(ang, &temp_sin, &temp_cos);
            x += (motorL->v + motorR->v) / 2 * temp_cos / MAIN_LOOP_FREQ;
            y += (motorL->v + motorR->v) / 2 * temp_sin / MAIN_LOOP_FREQ;
        } break;
        case CHASSIS_RELEASE:{
            this->motorL->state = MOTOR_RELEASE;
            this->motorR->state = MOTOR_RELEASE;
        } break;
        default:
            break;
    }
}



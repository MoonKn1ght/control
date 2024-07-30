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
        1.6583531955235222, 1.6299747496582606, 1.5733657561731795, 1.5488711628496536, 1.501297388087582, 1.4763559495269993, 1.4650593050152725, 1.4440509130489756, 1.4201576839767018, 1.3985009549812193, 1.3692879769705981, 1.3553188856169858, 1.33900373403005, 1.3248439753146455, 1.3133885387286497, 1.2869058080225428, 1.2693655576689098, 1.2636751511652973, 1.2415136636894748, 1.2262687848334937, 1.2166839252211334, 1.2061219046660507, 1.2021815979515669, 1.1904490277320603, 1.1820226650650114, 1.181006711314908, 1.158745302953053, 1.1519358961454278, 1.1507451213651116, 1.1822132364744131, 1.139936417955089, 1.146305571053974, 1.1377195245876015, 1.1365139824916641, 1.1295175829698652, 1.1265562973095347, 1.1086813582291883, 1.107016143277202, 1.0936719924554097, 1.0954428804768828, 1.0944250210114896, 1.0790635235881072, 1.0708258880074917, 1.0619259355281208, 1.0612468227683693, 1.0517872379107462, 1.0409894639720958, 1.0225203287064635, 1.033296515266687, 1.0296712992993207, 1.0210601519441125, 1.0199865422117123, 1.0150110438579918, 1.0131829241772456, 1.009666361229022, 1.008504704391162, 1.0057291715372898, 1.0057521423887388, 1.00426301978487, 1.003518220078157, 1.0013966967286736, 1.00130925901025, 1.000128822230389, 1.0004360191924773, 1.003575426961203, 1.0039000028661973, 1.002324907408464, 1.0010281632727096, 1.0, 1.002884938293213, 1.0006939813180349, 1.0008383262752978, 1.003430411498661, 1.0046224492771596, 1.0050434235230927, 1.0069458986463813, 1.0038121875243253, 1.0111677236669747, 1.0150383095343583, 1.0170220024788417, 1.0210285228256, 1.0238495845977706, 1.0211589667248402, 1.0263508219785393, 1.0285513901235326, 1.0357545881571875, 1.0310433653333477, 1.0322010337750631, 1.037599815204844, 1.0478981408901118, 1.0551691585390588, 1.0610674832634366, 1.0644708787497814, 1.075948120177311, 1.0854784687633119, 1.0923227050971105, 1.106862843929928, 1.1160818626891724, 1.1143001382612345, 1.1222724021136472, 1.132090912673371, 1.1430923790302991, 1.1482011826244638, 1.177564749961449, 1.1820702188721701, 1.1949727033770774, 1.1944479567136552, 1.1985771856766976, 1.1991545915300479, 1.2129085000750783, 1.2078668361416578, 1.2177901483591473, 1.2277009438913427, 1.2392046295149377, 1.2498413041568888, 1.2638505943983747, 1.278053939778892,
        1.2840748450550246, 1.3016156055937484, 1.297684957366528, 1.3103294603991076, 1.3284058559005814, 1.386055471742289, 1.4276732043051745, 1.4037689334212773, 1.388075380532694, 1.4184398698132963, 1.4340048718100598};
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

            INRANGE(output1, PWM_DUTY_MAX);
            INRANGE(output2, PWM_DUTY_MAX);
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



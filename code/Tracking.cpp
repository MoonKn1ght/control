/*
 * Tracking.cpp
 *
 *  Created on: Jul 25, 2024
 *      Author: pc
 */

#include "Tracking.h"


float ccd_norm[128] = {
        1.2870592260559972, 1.2648513515347348, 1.2233098296859939, 1.1887824932296176, 1.158897621206234, 1.160736277983108, 1.1506209789308148, 1.1422216875537974, 1.1241699662532234, 1.1080059781492437, 1.0845954838043605, 1.0755111689342371, 1.0625902369880764, 1.0532066864988638, 1.0430060219036525, 1.033213535397254, 1.0258783767959903, 1.024650788336391, 1.0205240889733236, 1.0203520421617636, 1.0174206713786276, 1.0148192429801175, 1.0128817312646912, 1.0120848994195895, 1.013205441320869, 1.013105223242896, 1.0116120131676096, 1.0116927795661554, 1.008348685455276, 1.0093078680843117, 1.0075399676296595, 1.0079670245162877, 1.0068314467514126, 1.0075628397448004, 1.0066830726833582, 1.0060973741238588, 1.0047688324804755, 1.0050077218127478, 1.00318297362638, 1.0034664813793481, 1.0027373055133346, 1.0032964174998906, 1.002582466679666, 1.0028958491394346, 1.0023221528386286, 1.0023824574275244, 1.001556909334388, 1.0023937765752367, 1.0033454984742491, 1.0036328805208894, 1.0027787797599925, 1.003046946799558, 1.000443237697232, 1.0013723704167004, 1.0005108938710567, 1.0005635813381462, 1.0001464104404474, 1.000969633966161, 1.0001089095460023, 1.0005522438155252, 1.0, 1.0008229824224413, 1.0013310124223251, 1.0020430616742786, 1.0020430616742786, 1.0027373055133346, 1.0022240760669738, 1.0011464969901247, 1.000386924192374, 1.0010712284901788, 1.000386924192374, 1.001266950135495, 1.0024315688088086, 1.002797660069299, 1.0024654704476363, 1.0029299020982767, 1.0025032680875057, 1.0030016726841808, 1.0029373964577761, 1.003595117722459, 1.0030658372084027, 1.0017038956746318, 1.0014854051364313, 1.0017792593461272, 1.0017792593461272, 1.0023937765752367, 1.0042913943565739, 1.005049384089441, 1.004302756657947, 1.005489637842602, 1.0054022671927747, 1.0029638374527234, 1.0029827247312675, 1.0040642623975085, 1.0040339778868808, 1.005630065248529, 1.0076657771149118, 1.008784131633661, 1.0085244071846986, 1.0089867040033649, 1.0098054050527299, 1.0109132272590082, 1.0112586593596429, 1.0133634566042669, 1.0130898068366891, 1.0121310583350878, 1.0108940401221613, 1.0124466514106634, 1.0133596004936931, 1.0153571318554713, 1.0156706821172705, 1.0162094741945356, 1.017976887240655, 1.0198479663756619, 1.0193172005391005, 1.0215615082381784, 1.0246311387605207, 1.02724349122853, 1.0283064726943312, 1.0299569531789083, 1.0357612382845565, 1.054069480736888, 1.0807792358550483, 1.1079828552476714, 1.0907299871795204, 1.08226822747299, 1.1012161268160363, 1.1127512877358572};

Tracking::Tracking(Chassis* chassis, CCD* ccd,PID* tracking_PID,Controller* controller)
{
	this->chassis = chassis;
	this->ccd = ccd;
	this->tracking_PID = tracking_PID;
	this->controller = controller;
}

void Tracking::init()
{
	tracking_PID->target = 0;
	tracking_PID->feedback = &input;
	last_x_line = 15.4/100;
	last_y_line = 0;
	last_dir = dir_filtered = 64;
	v = 0.08;
	state = 0;

}
void Tracking::process()
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
	//Get the direction
	//从两边向中间找线
	for(i = 3; i < 125; ++i)	//从左找线
	{
		if(bin_ccd[i-1] + bin_ccd[i-2] + bin_ccd[i-3] == 3
			&& bin_ccd[i+1] + bin_ccd[i+2] + bin_ccd[i+3] == 0)
		{
			Left = i;
			break;
		}
		if(i == 124)
			Left = 500;
	}

	for(i = 125; i > 3; --i)
	{
		if(bin_ccd[i-1] + bin_ccd[i-2] + bin_ccd[i-3] == 0
			&& bin_ccd[i+1] + bin_ccd[i+2] + bin_ccd[i+3] == 3)
		{
			Right = i;
			break;
		}
		if(i == 4)
			Right = 500;
	}

	last_dir = dir;


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
        arm_sin_cos_f32(chassis->ang, &temp_sin, &temp_cos);
        x_line = chassis->x + D * temp_cos + mid_point * dl * temp_sin;
        y_line = chassis->y + D * temp_sin - mid_point * dl * temp_cos;
    }


}

//#define U_turn
#define line_memory0
//#define line_memory1
//#define line_memory2

void Tracking::Handler()
{

    if(state)
    {
#ifdef U_turn

	if(Left == 500 || Right == 500)	//未检测到线
	{
		chassis->v_set = 0 ;
		chassis->w_set = 0.05;
		chassis->state = CHASSIS_RUN;
	}
	else
	{
		dir = (Right + Left)/2;
		dir_filtered = (1 - k_filter) * dir + k_filter * dir_filtered;
		input = dir_filtered-64;
		if(Right - Left < 35)
		{
			chassis->v_set = v ;
			chassis->w_set = tracking_PID->calc()*0.5;
			chassis->state = CHASSIS_RUN;
		}
		else
		{
			chassis->state = CHASSIS_STOP;
			chassis->v_set = 0.05 ;
			chassis->w_set = 0.05;
			chassis->state = CHASSIS_RUN;
		}
	}
#endif

#ifdef line_memory0

#endif

#ifdef line_memory1
        static int t1 = 0;
        static int t2 = 0;


		if(Left == 500)	//左丢线
		{
			chassis->v_set = 0 ;
			chassis->w_set = 0.05;
			chassis->state = CHASSIS_RUN;
		}
		else if(Right == 500)	//右丢线
		{
			chassis->v_set = 0 ;
			chassis->w_set = -0.05;
			chassis->state = CHASSIS_RUN;
		}
		else if(Right - Left < 35)
		{
			dir = (Right + Left)/2;
			dir_filtered = (1 - k_filter) * dir_filtered + k_filter * ((float)dir - 64);

			//计算轨道坐标，单位为m
			float theta = atan((64-dir_filtered)*0.093/120/0.154);
			float distance = sqrt(((64-dir_filtered)*0.093/120)*((64-dir_filtered)*0.093/120) + (0.154)*(0.154));
			x_line = chassis->x + distance*cos(theta + chassis->ang*PI/180);
			y_line = chassis->y + distance*sin(theta + chassis->ang*PI/180);
			controller->x_set = x_line;
			controller->y_set = y_line;
//			if(abs(x_line - last_x_line) + abs(y_line - last_y_line) > 0.03)
//			{
//				x_line_array[t1] = x_line;
//				y_line_array[t1] = y_line;
//				++t1;
//				last_x_line = x_line;
//				last_y_line = y_line;
//				controller->x_set = x_line;
//				controller->y_set = y_line;
//			}
		}
//		else	//回到起点
//		{
//			if(t2 == 0)
//			{
//				controller->x_set = x_line_array[0];
//				controller->y_set = y_line_array[0];
//			}
//			if(t2 < t1)
//			{
//				if(abs(chassis->x-x_line_array[t2]) + abs(chassis->y-y_line_array[t2]) < 0.13)
//				{
//					t2++;
//					controller->x_set = x_line_array[t2];
//					controller->y_set = y_line_array[t2];
//				}
//			}
//			else
//			{
//				chassis->state = CHASSIS_STOP;
//			}
//		}
	}

#endif

#ifdef line_memory2

	static int t1 = 0;
	static int t2 = 0;
	if(!start_flag)
	{
		if(Left == 500)
		{
			chassis->v_set = 0 ;
			chassis->w_set = 0.05;
			chassis->state = CHASSIS_RUN;
		}
		else if(Right == 500)
		{
			chassis->v_set = 0 ;
			chassis->w_set = -0.05;
			chassis->state = CHASSIS_RUN;
		}
		else
		{
			dir = (Right + Left)/2;
			dir_filtered = (1 - k_filter) * dir + k_filter * dir_filtered;
			input = dir_filtered-64;
			if(Right - Left < 35)
			{
				chassis->v_set = v ;
				chassis->w_set = tracking_PID->calc()*0.5;
				chassis->state = CHASSIS_RUN;
				x_line = chassis->x;
				y_line = chassis->y;
				if(abs(x_line - last_x_line) + abs(y_line - last_y_line) > 0.03)
				{
					x_line_array[t1] = x_line;
					y_line_array[t1] = y_line;
					++t1;
					last_x_line = x_line;
					last_y_line = y_line;
					controller->x_set = x_line;
					controller->y_set = y_line;
				}
			}
//			else
//			{
//				chassis->state = CHASSIS_STOP;
//				start_flag = 1;
//			}
		}
		if(Right == 500 && Left == 500)
		{
			chassis->state = CHASSIS_STOP;
			start_flag = 1;
		}
	}
	else
	{
		if(t2 == 0)
		{
			controller->x_set = x_line_array[0];
			controller->y_set = y_line_array[0];
			controller->Handler();
		}
		if(t2 < t1)
		{
			if(abs(chassis->x-x_line_array[t2]) + abs(chassis->y-y_line_array[t2]) < 0.03)
			{
				t2++;
				controller->x_set = x_line_array[t2];
				controller->y_set = y_line_array[t2];
				controller->Handler();
			}
		}
		else
		{
			chassis->state = CHASSIS_STOP;
		}
	}


#endif
    }
}

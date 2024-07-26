/*
 * Tracking.cpp
 *
 *  Created on: Jul 25, 2024
 *      Author: pc
 */

#include "Tracking.h"

//#define U_turn
//#define line_memory

float ccd_norm[128] = {2122.7,2156.54,2253.56,2302.95,2364.85,2384.49,2396.97,2428.91,
						2480.69,2519.74,2546.13,2574.88,2585.78,2596.07,2601.78,2609.21,2613.86,
						2617.39,2624.16,2627.02,2630.75,2632.91,2636.34,2635.21,2639.37,2637.85,
						2640.23,2640.35,2642.84,2640.99,2644.47,2646.65,2647.22,2647.11,
						2650.54,2648.92,2649.7,2648.93,2653.38,2653.39,2654.39,2652.66,
						2655.27,2655.2,2657.64,2655.99,2656.98,2656.18,2656.85,2656.03,2658.39,
						2657.44,2659.74,2658.54,2660.06,2658.44,2659.89,2659.08,2661.04,
						2659.3,2661.93,2659.94,2661.27,2659.89,2654.96,2658.26,2662.76,
						2661.22,2663.08,2660.28,2661.75,2659.91,2661.96,2660.47,2661.43,2659.87,
						2661.34,2659.94,2660.41,2659.75,2661.03,2658.01,2659.25,2657.01,2656.67,
						2655.27,2657.45,2655.02,2655.54,2654.36,2655.18,2653.17,2653.52,
						2650.31,2651.69,2650.36,2650.25,2646.86,2647.2,2644.43,2645.1,
						2641.63,2640.21,2637.08,2639.83,2630.44,2632.93,2631.94,2632.31,
						2629.85,2629.91,2624.66,2625.5,2620.12,2615.18,2612.95,2604.39,2589.61,2587.5,2584.95,
						2582.28,2553.78,2483.84,2428.47,2454.0,2465.4,2417.91,2374.55};
float max_ccd_norm = 2663.08;

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
	tracking_PID->feedback = &dir_filtered;
	last_x_line = 15.4/100;
	last_y_line = 0;
}

void Tracking::Handler()
{
	// Normalization
	int i;
	for(i = 0; i < 128; ++i)
	{
		data_norm[i] = ccd->data[i]/ccd_norm[i]*max_ccd_norm;
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
	uint16_t Left,Right;
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
		dir_filtered = (1 - k_filter) * dir_filtered + k_filter * ((float)dir - 64);

		if(Right - Left < 35)
		{
			chassis->v_set = v ;
			chassis->w_set = tracking_PID->calc()*0.5;
			chassis->state = CHASSIS_RUN;
		}
		else
		{
			chassis->state = CHASSIS_STOP;
			chassis->v_set = 0 ;
			chassis->w_set = 0.05;
			chassis->state = CHASSIS_RUN;
		}
	}
#endif

#ifdef line_memory
	static int t1 = 0;
	static int t2 = 0;
    static float x_line_array[1000] = {0};
	static float y_line_array[1000] = {0};
	if(Left == 500 || Right == 500)	//未检测到线
	{
		chassis->v_set = 0 ;
		chassis->w_set = 0.05;
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
	else	//回到起点
	{
		if(t2 == 0)
		{
			controller->x_set = x_line_array[0];
			controller->y_set = y_line_array[0];
		}
		if(t2 < t1)
		{
			if(abs(chassis->x-x_line_array[t2]) + abs(chassis->y-y_line_array[t2]) < 0.13)
			{
				t2++;
				controller->x_set = x_line_array[t2];
				controller->y_set = y_line_array[t2];
			}
		}
		else
		{
			chassis->state = CHASSIS_STOP;
		}
	}


#endif

}

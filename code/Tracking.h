/*
 * Tracking.h
 *
 *  Created on: Jul 25, 2024
 *      Author: pc
 */

#ifndef TRACKING_H_
#define TRACKING_H_

#include "Chassis.h"
#include "CCD.h"
#include "PID.h"
#include "Controller.h"

class Tracking
{
public:
    bool state = false;

	PID* tracking_PID;
	CCD* ccd;
	Chassis* chassis;
	Controller* controller;

	//视野参数
	float D = 8.8 / 100;
	float dl = 8.0 / 100 / 88;

	//轨道识别信息
	bool inrange = false;
	float mid_point = 0;
    float x_line = 0, y_line = 0;	//轨道坐标

	float data_norm[128];
	uint8_t bin_ccd[128];
	float dir_filtered;
	float k_filter = 0.1;
    uint16_t dir;	//黑线位置
    uint16_t last_dir;
    float threshold;
	uint16_t Left,Right;
	float input;

    float v;

    float last_x_line,last_y_line;
    bool start_flag = 0;

	float x_line_array[1000] = {0};
	float y_line_array[1000] = {0};


	Tracking(Chassis* chassis, CCD* ccd,PID* tracking_PID,Controller* controller);
	void Handler();
	void init();
	void process();


};





#endif /* TRACKING_H_ */

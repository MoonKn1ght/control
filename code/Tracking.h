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
	PID* tracking_PID;
	CCD* ccd;
	Chassis* chassis;
	Controller* controller;
	float data_norm[128];
	uint8_t bin_ccd[128];
	float dir_filtered;
	float k_filter;
    uint16_t dir;	//黑线位置
    uint16_t last_dir;
    float threshold;

    float v;

    float x_line,y_line;	//轨道坐标
    float last_x_line,last_y_line;


	Tracking(Chassis* chassis, CCD* ccd,PID* tracking_PID,Controller* controller);
	void Handler();
	void init();


};





#endif /* TRACKING_H_ */

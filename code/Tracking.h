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

class Tracking
{
public:
	PID* tracking_PID;
	CCD* ccd;
	Chassis* chassis;
	uint16_t data_norm[128];
	uint8_t bin_ccd[128];
	float dir_filtered;
	float k_filter;
    uint16_t dir;	//黑线位置
    uint16_t last_dir;
    uint16_t threshold;

    float v;

    float x_path,y_path;	//轨道坐标


	Tracking(Chassis* chassis, CCD* ccd,PID* tracking_PID);
	void Handler();
	void init();


};





#endif /* TRACKING_H_ */

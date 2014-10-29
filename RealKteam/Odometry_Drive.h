/*
 * Odometry_Drive.h
 *
 *  Created on: Dec 19, 2013
 *      Author: bluebird
 */

#ifndef ODOMETRY_DRIVE_H_
#define ODOMETRY_DRIVE_H_

#include "odometry_track.h"
#include "odometry_goto.h"

// Instances of the odometry objects
struct sOdometryTrack ot;
struct sOdometryGoto og;

#endif /* ODOMETRY_DRIVE_H_ */

void Odometry_Drive_Init();
void run_goto_position(float x, float y);
void run_goto_heading(float goal_theta);
void run_goto_wait(int wait_usec);
void UMBmark_Test_Odometry();
void UMBmark_Test_Odometry_1();


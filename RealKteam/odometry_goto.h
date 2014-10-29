

#ifndef ODOMETRY_GOTO
#define ODOMETRY_GOTO

#include "odometry_track.h"

struct sOdometryGoto {
	struct {
		int acceleration_step;
		float speed_max;
		float speed_min;
	} configuration;
	struct sOdometryTrack * track;
	struct {
		float goal_x;
		float goal_y;
		int speed_left_internal;
		int speed_right_internal;
	} state;
	struct {
		int speed_left;
		int speed_right;
		int closetogoal;
		int veryclosetogoal;
		int atgoal;
	} result;

	struct {
		float K_1;
		float K_2;
		float K_3;
		float Offset;
	}SettingForOdometry_goto;
};


//! Initializes this module.
void odometry_goto_init();

//! Initializes a goto structure.
void odometry_goto_start(struct sOdometryGoto * og, struct sOdometryTrack * ot);
//! Sets a new target position.
void odometry_goto_set_goal(struct sOdometryGoto * og, float goalx, float goaly);
//! Calculates the new motor speeds.
void odometry_goto_step(struct sOdometryGoto * og);

//使用A Stable Target-Tracking Control for Unicycle Mobile Borots的算法
void odometry_goto_step_UseStableTT(struct sOdometryGoto * og);

#endif

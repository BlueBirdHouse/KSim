

#include "odometry_goto.h"
#include <stdlib.h>
#include <math.h>

#define max(x,y) ( x>y?x:y )

#define PI 3.14159265358979

#define speed_min_Number 2000
#define speed_max_Number 43000
#define W_Max 9.561 //At 43000

#define W_Min  (W_Max*speed_min_Number/speed_max_Number)//At 2000
#define Rad_S_to_Number (speed_max_Number/W_Max)

void odometry_goto_init() {
}

void odometry_goto_start(struct sOdometryGoto * og, struct sOdometryTrack * ot) {
	og->track = ot;
	og->configuration.acceleration_step = 500.;
	og->configuration.speed_min = speed_min_Number;
	og->configuration.speed_max = speed_max_Number;
	og->state.goal_x = 0.;
	og->state.goal_y = 0.;
	og->state.speed_left_internal = 0;
	og->state.speed_right_internal = 0;
	og->result.speed_left = 0;
	og->result.speed_right = 0;
	og->result.atgoal = 1;
	og->result.closetogoal = 1;
	og->result.veryclosetogoal = 1;

	ot->SettingForOdometry_goto.Radii_Left = ot->configuration.wheel_conversion_left*4198.4/(2*PI);
	ot->SettingForOdometry_goto.Radii_Right = ot->configuration.wheel_conversion_right*4198.4/(2*PI);

	ot->SettingForOdometry_goto.MaxRadii = max(ot->SettingForOdometry_goto.Radii_Left,ot->SettingForOdometry_goto.Radii_Right);

	float closetogoal_Range = 1;//在什么范围内执行减速(m)

	og->SettingForOdometry_goto.K_1 = (ot->SettingForOdometry_goto.MaxRadii*W_Max)/closetogoal_Range;

	og->SettingForOdometry_goto.Offset = (W_Min*ot->SettingForOdometry_goto.MaxRadii)/og->SettingForOdometry_goto.K_1;

	og->SettingForOdometry_goto.K_3 = W_Min*ot->SettingForOdometry_goto.MaxRadii + 0.0003;

	og->SettingForOdometry_goto.K_2 = og->SettingForOdometry_goto.K_1*2;


	//int Temp = 0;
}

void odometry_goto_set_goal(struct sOdometryGoto * og, float x, float y) {
	og->state.goal_x = x;
	og->state.goal_y = y;
	og->result.atgoal = 0;
	og->result.closetogoal = 0;
	og->result.veryclosetogoal = 0;
}

void odometry_goto_step(struct sOdometryGoto * og) {
	float dx, dy;
	float distance, goalangle, alpha;
	float speedfactor;
	long speed_left_wish, speed_right_wish;
	//int atmaxspeed = 0;

	// Do nothing if we are at goal
	if (og->result.atgoal != 0) {
		return;
	}

	// Calculate new wish speeds
	dx = og->state.goal_x - og->track->result.x;
	dy = og->state.goal_y - og->track->result.y;
	distance = sqrt(dx * dx + dy * dy);
	goalangle = atan2(dy, dx);
	alpha = goalangle - og->track->result.theta;
	while (alpha > PI) {
		alpha -= 2 * PI;
	}
	while (alpha < -PI) {
		alpha += 2 * PI;
	}

	// Calculate the speed factor
	speedfactor = (distance) * 10. * og->configuration.speed_max;

	if (speedfactor > og->configuration.speed_max) {
		speedfactor = og->configuration.speed_max;
		//atmaxspeed = 1;
	}
	//else
	//{
		//atmaxspeed = 0;
	//}

	// Calculate the theoretical speed
	//printf("dist %f - goalangle %f - alpha %f \n", distance, goalangle, alpha);
	//speed_left_wish = speedfactor * (PI - 2 * abs(alpha) + alpha) / PI ;
	//speed_right_wish = speedfactor * (PI - 2 * abs(alpha) - alpha) / PI ;

	speed_right_wish = speedfactor * (PI - 2 * abs(alpha) + alpha) / PI ;
	speed_left_wish = speedfactor * (PI - 2 * abs(alpha) - alpha) / PI ;

	/*
	// Close to termination condition: just stop
	if (og->result.veryclosetogoal) {
		speed_left_wish = 0;
		speed_right_wish = 0;
	}
	*/

/*
	// Limit acceleration
	if (speed_left_wish > og->state.speed_left_internal) {
		og->state.speed_left_internal += og->configuration.acceleration_step;
	}
	if (speed_left_wish < og->state.speed_left_internal) {
		og->state.speed_left_internal -= og->configuration.acceleration_step;
		atmaxspeed = 0;
	}
	if (speed_right_wish > og->state.speed_right_internal) {
		og->state.speed_right_internal += og->configuration.acceleration_step;
	}
	if (speed_right_wish < og->state.speed_right_internal) {
		og->state.speed_right_internal -= og->configuration.acceleration_step;
		atmaxspeed = 0;
	}
	// Don't set speeds < MIN_SPEED (for accuracy reasons)
	og->result.speed_left = og->state.speed_left_internal;
	og->result.speed_right = og->state.speed_right_internal;

*/
	og->result.speed_left = speed_left_wish;
	og->result.speed_right = speed_right_wish;

	/*
	if (abs(og->result.speed_left) < og->configuration.speed_min) {
		og->result.speed_left = 0;
		//atmaxspeed = 0;

	}
	if (abs(og->result.speed_right) < og->configuration.speed_min) {
		og->result.speed_right = 0;
		//atmaxspeed = 0;
	}

	// Termination condition
	if (atmaxspeed == 0) {
		og->result.closetogoal = 1;
		if ((og->result.speed_left == 0) || (og->result.speed_right == 0)) {
			og->result.veryclosetogoal = 1;
		}
		if ((og->result.speed_left == 0) && (og->result.speed_right == 0)) {
			og->result.atgoal = 1;
		}
	}
	*/

	if((fabs(dx) < 0.03)&&(fabs(dy) < 0.03))
	{
		og->result.speed_left = 0;
		og->result.speed_right = 0;
		og->result.veryclosetogoal = 1;
		og->result.atgoal = 1;
	}
}

void odometry_goto_step_UseStableTT(struct sOdometryGoto * og) {
	//推箱子算法
	float dx, dy;
	float distance, goalangle,Phi;
	float W_Right_float,W_Left_float;
	long speed_left_wish, speed_right_wish;
	//int atmaxspeed = 0;

	// Do nothing if we are at goal
	if (og->result.atgoal != 0) {
		return;
	}

	// Calculate new wish speeds
	dx =  og->track->result.x - og->state.goal_x;
	dy = og->track->result.y - og->state.goal_y;

	distance = sqrt(dx * dx + dy * dy);
	goalangle = atan2(dy, dx);

	Phi = PI + og->track->result.theta - goalangle;
	while (Phi > PI) {
		Phi -= 2 * PI;
	}
	while (Phi < -PI) {
		Phi += 2 * PI;
	}

	//W_Right_float = (og->SettingForOdometry_goto.K_1*(distance + og->SettingForOdometry_goto.Offset) * cosf(Phi))/og->track->SettingForOdometry_goto.Radii_Right - (og->track->configuration.wheel_distance*(og->SettingForOdometry_goto.K_2*Phi + og->SettingForOdometry_goto.K_1*cosf(Phi)*sinf(Phi)))/(2*og->track->SettingForOdometry_goto.Radii_Right);
	//W_Left_float = (og->SettingForOdometry_goto.K_1*(distance + og->SettingForOdometry_goto.Offset) *  cosf(Phi))/og->track->SettingForOdometry_goto.Radii_Left +  (og->track->configuration.wheel_distance*(og->SettingForOdometry_goto.K_2*Phi + og->SettingForOdometry_goto.K_1*cosf(Phi)*sinf(Phi)))/(2*og->track->SettingForOdometry_goto.Radii_Left);
	if(distance > og->SettingForOdometry_goto.Offset)
	{
		W_Right_float = (og->SettingForOdometry_goto.K_1*(distance) * cosf(Phi))/og->track->SettingForOdometry_goto.Radii_Right - (og->track->configuration.wheel_distance*(og->SettingForOdometry_goto.K_2*Phi + og->SettingForOdometry_goto.K_1*cosf(Phi)*sinf(Phi)))/(2*og->track->SettingForOdometry_goto.Radii_Right);
		W_Left_float = (og->SettingForOdometry_goto.K_1*(distance) *  cosf(Phi))/og->track->SettingForOdometry_goto.Radii_Left +  (og->track->configuration.wheel_distance*(og->SettingForOdometry_goto.K_2*Phi + og->SettingForOdometry_goto.K_1*cosf(Phi)*sinf(Phi)))/(2*og->track->SettingForOdometry_goto.Radii_Left);

		og->result.closetogoal = 0;

		speed_right_wish = W_Right_float * Rad_S_to_Number;
		speed_left_wish = W_Left_float * Rad_S_to_Number;
	}
	else
	{
		W_Right_float = ((og->SettingForOdometry_goto.K_3) * cosf(Phi))/og->track->SettingForOdometry_goto.Radii_Right - (og->track->configuration.wheel_distance*(og->SettingForOdometry_goto.K_2*Phi + og->SettingForOdometry_goto.K_1*cosf(Phi)*sinf(Phi)))/(2*og->track->SettingForOdometry_goto.Radii_Right);
		W_Left_float = ((og->SettingForOdometry_goto.K_3) *  cosf(Phi))/og->track->SettingForOdometry_goto.Radii_Left +  (og->track->configuration.wheel_distance*(og->SettingForOdometry_goto.K_2*Phi + og->SettingForOdometry_goto.K_1*cosf(Phi)*sinf(Phi)))/(2*og->track->SettingForOdometry_goto.Radii_Left);

		//W_Right_float = ((og->SettingForOdometry_goto.K_3) * cosf(Phi))/og->track->SettingForOdometry_goto.Radii_Right - (og->track->configuration.wheel_distance*(og->SettingForOdometry_goto.K_2*Phi + (og->SettingForOdometry_goto.K_3/distance)*cosf(Phi)*sinf(Phi)))/(2*og->track->SettingForOdometry_goto.Radii_Right);
		//W_Left_float = ((og->SettingForOdometry_goto.K_3) *  cosf(Phi))/og->track->SettingForOdometry_goto.Radii_Left +  (og->track->configuration.wheel_distance*(og->SettingForOdometry_goto.K_2*Phi + (og->SettingForOdometry_goto.K_3/distance)*cosf(Phi)*sinf(Phi)))/(2*og->track->SettingForOdometry_goto.Radii_Left);

		og->result.closetogoal = 1;

		speed_right_wish = W_Right_float * Rad_S_to_Number;
		speed_left_wish = W_Left_float * Rad_S_to_Number;

		if(fabs(speed_right_wish) < og->configuration.speed_min)
		{
			speed_right_wish = 0;
		}
		if(fabs(speed_left_wish) < og->configuration.speed_min)
		{
			speed_left_wish = 0;
		}
	}


	if (speed_right_wish > og->configuration.speed_max) {
		speed_right_wish = og->configuration.speed_max;
	}
	if (speed_left_wish > og->configuration.speed_max) {
		speed_left_wish = og->configuration.speed_max;
	}

	if (speed_right_wish < -(og->configuration.speed_max)) {
		speed_right_wish = -(og->configuration.speed_max);
	}
	if (speed_left_wish < -(og->configuration.speed_max)) {
		speed_left_wish = -(og->configuration.speed_max);
	}

	og->result.speed_left = speed_left_wish;
	og->result.speed_right = speed_right_wish;


	//if((fabs(dx) < 0.01)&&(fabs(dy) < 0.01))
	if(distance < 0.01)
	{
		og->result.speed_left = 0;
		og->result.speed_right = 0;
		og->result.veryclosetogoal = 1;
		og->result.atgoal = 1;
	}

}


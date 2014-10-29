

#include "khepera3.h"
#include "odometry_track.h"
#include "odometry_goto.h"
#include "commandline.h"
#include "nmea.h"
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define PI 3.14159265358979

// Instances of the odometry objects
struct sOdometryTrack ot;
struct sOdometryGoto og;

// Level of verbosity (0=quiet, 1=normal, 2=verbose, 3=very verbose, ...)
int verbosity;

void run_goto_position(float x, float y) {
	// Set and announce the new target position
	odometry_goto_set_goal(&og, x, y);
	if (verbosity > 0) {
		printf("$GOTO_POSITION,%f,%f\n", og.state.goal_x, og.state.goal_y);
	}

	// Move until we have reached the target position
	while (og.result.atgoal == 0) {
		// Update position and calculate new speeds
		odometry_track_step(og.track);
		odometry_goto_step(&og);

		// Set speed
		khepera3_drive_set_speed(og.result.speed_left, og.result.speed_right);
		//khepera3_drive_set_speed_using_profile(og.result.speed_left, og.result.speed_right);

		//printf("SpeedLeft = %d\n",og.result.speed_left);
		//printf("Speedright = %d\n",og.result.speed_right);
		//printf("X = %f\n",og.track->result.x);
		//printf("Y = %f\n",og.track->result.y);

		// Print the current position
		if (verbosity > 1) {
			//khepera3_drive_set_speed(0, 0);
			printf("$POSITION,%f,%f,%f\n", og.track->result.x, og.track->result.y, og.track->result.theta);

		}
	}

	// Announce when we have reached the target position
	if (verbosity > 0) {
		printf("$POSITION,%f,%f,%f\n", og.track->result.x, og.track->result.y, og.track->result.theta);
		fflush(stdout);
	}
}

void run_goto_heading(float goal_theta) {
	float diff_theta;

	// Announce the new target heading
	if (verbosity > 0) {
		printf("$GOTO_HEADING,%f\n", goal_theta);
	}

	// Move until we have reached the target position
	while (1) {
		// Update position and calculate new speeds
		odometry_track_step(og.track);

		// Calculate the current heading error
		diff_theta = goal_theta - og.track->result.theta;
		while (diff_theta > PI) {
			diff_theta -= 2 * PI;
		}
		while (diff_theta < -PI) {
			diff_theta += 2 * PI;
		}

		// Termination condition
		if (fabs(diff_theta) < 0.01) {
			break;
		}

		// Set speed
		khepera3_drive_set_speed_differential_bounded(5000, 0, 0, diff_theta * 1., 1);

		// Print the current position
		if (verbosity > 1) {
			printf("$POSITION,%f,%f,%f\n", og.track->result.x, og.track->result.y, og.track->result.theta);
		}
	}

	// Stop the motors
	khepera3_drive_set_speed(0, 0);

	// Announce when we have reached the target heading
	if (verbosity > 0) {
		printf("$POSITION,%f,%f,%f\n", og.track->result.x, og.track->result.y, og.track->result.theta);
		fflush(stdout);
	}
}

void run_goto_wait(int wait_usec) {
	// Announce the waiting time
	if (verbosity > 0) {
		printf("$GOTO_WAIT,%d\n", wait_usec);
	}

	// Wait
	usleep(wait_usec);

	// Announce when we are done
	if (verbosity > 0) {
		printf("$POSITION,%f,%f,%f\n", og.track->result.x, og.track->result.y, og.track->result.theta);
		fflush(stdout);
	}
}

void UMBmark_Test_Odometry()
{
	//标准UMBmark实验用例
	float L = 0.5;
			run_goto_position(L, 0);
			sleep(3);
			run_goto_heading(PI/2);
			sleep(3);
			run_goto_position(L, L);
			sleep(3);
			run_goto_heading(PI);
			sleep(3);
			run_goto_position(0, L);
			sleep(3);
			run_goto_heading(-PI/2);
			sleep(3);
			run_goto_position(0, 0);
}


// Main program.
int main(int argc, char *argv[]) {

	// Module initialization
	khepera3_init();
	odometry_track_init();
	odometry_goto_init();
	// Read command line arguments
	verbosity = 1;

	// Start a new track
	odometry_track_start(&ot);
	odometry_goto_start(&og, &ot);

	// Put the wheels in normal (control) mode
	khepera3_drive_init();
	khepera3_drive_start();

	//run_goto_position(0.5, 0);
	//run_goto_heading(30*PI/180);
	//run_goto_heading(PI);
	UMBmark_Test_Odometry();
	return 0;
}

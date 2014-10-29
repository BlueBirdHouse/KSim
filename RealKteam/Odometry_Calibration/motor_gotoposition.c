
#include "khepera3.h"
#include "commandline.h"
#include <stdio.h>
#include <stdlib.h>

#include <korebot/korebot.h> //基本初始化文件
#include <korebot/kb_khepera3.h>

// Prints the help text.
void help() {
	printf("Runs the motors to a certain encoder position.\n");
	printf("\n");
	printf("Usage:\n");
	printf("  motor_gotoposition [OPTIONS] POS         Runs both motors to POS\n");
	printf("  motor_gotoposition [OPTIONS] POS1 POS2   Runs the left motor until POS1 and right motor to POS2\n");
	printf("  motor_gotoposition [OPTIONS] -l POS      Runs the left motor to POS\n");
	printf("  motor_gotoposition [OPTIONS] -r POS      Runs the right motor to POS\n");
	printf("\n");
	printf("Options:\n");
	printf("  -p     Uses a trapezoidal speed profile\n");
	printf("\n");
}

// Main program.
int main(int argc, char *argv[]) {

	 kb_set_debug_level(2);

	int ResponseValue1 = 0;
	 ResponseValue1 = kb_init( argc , argv );//通常不需要主动调用这个，因为kh3_init会间接调用
	 if(ResponseValue1 != 1)
	 {
		 //Stop!
		 printf("kb_init初始化失败!\n");
		 kb_set_debug_level(2);
	 }

	// Command line parsing
	commandline_init();
	commandline_option_register("-p", "--profile", cCommandLine_Option);
	commandline_parse(argc, argv);

	// Help
	if (commandline_option_provided("-h", "--help")) {
		help();
		exit(1);
	}

	// Initialization
	khepera3_init();
	// Put the wheels in normal (control) mode
	khepera3_drive_start();

	//khepera3_drive_goto_position_using_profile(4198, 4198);
	//khepera3;
	//int Temp = 0;
	// Set the speed


	if (commandline_argument_count() == 1) {
		if (commandline_option_provided("-p", "--profile")) {
			khepera3_drive_goto_position_using_profile(commandline_argument_int(0, 0), commandline_argument_int(0, 0));
		} else {
			khepera3_drive_goto_position(commandline_argument_int(0, 0), commandline_argument_int(0, 0));
		}
	} else if (commandline_argument_count() == 2) {
		if (commandline_option_provided("-p", "--profile")) {
			khepera3_drive_goto_position_using_profile(commandline_argument_int(0, 0), commandline_argument_int(1, 0));
		} else {
			khepera3_drive_goto_position(commandline_argument_int(0, 0), commandline_argument_int(1, 0));
		}
	} else {
		if (commandline_option_provided("-l", "--left")) {
			if (commandline_option_provided("-p", "--profile")) {
				khepera3_motor_goto_position_using_profile(&(khepera3.motor_left), commandline_option_value_int("-l", "--left", 0));
			} else {
				khepera3_motor_goto_position(&(khepera3.motor_left), commandline_option_value_int("-l", "--left", 0));
			}
		}

		if (commandline_option_provided("-r", "--right")) {
			if (commandline_option_provided("-p", "--profile")) {
				khepera3_motor_goto_position_using_profile(&(khepera3.motor_right), commandline_option_value_int("-r", "--right", 0));
			} else {
				khepera3_motor_goto_position(&(khepera3.motor_right), commandline_option_value_int("-r", "--right", 0));
			}
		}
	}

	return 0;
}


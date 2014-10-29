

#include <assert.h>
#include <math.h>
#include "khepera3_drive.h"
#include "khepera3.h"

/*** Module initialization ***/
#include <korebot/korebot.h> //基本初始化文件
#include <korebot/kb_khepera3.h>

//一种机器人硬件专用数据结构
static knet_dev_t * dsPic;
static knet_dev_t * mot1;
static knet_dev_t * mot2;

#define PI 3.14159265358979

//#define PULSE_TO_MM_FIRMWARE_BE_3 0.03068 //motor position factor,固件版本>3.0
#define PULSE_TO_MM_FIRMWARE_BE_3_Left 0.032268396038840 //motor position factor,固件版本>3.0
#define PULSE_TO_MM_FIRMWARE_BE_3_Right 0.032268396038840 //motor position factor,固件版本>3.0

//#define MM_S_TO_SPEED_FIRMWARE_BE_3 218.72 //motor speed factor,固件版本>3.0
#define MM_S_TO_SPEED_FIRMWARE_BE_3_Left 218.72 //motor speed factor,固件版本>3.0
#define MM_S_TO_SPEED_FIRMWARE_BE_3_Right 218.72 //motor speed factor,固件版本>3.0

//#define WHEEL_Base 88.41 //mm
#define WHEEL_Base 89.5746 //mm


void KTeam_Init_Motor()
{
	//初始化电机逻辑
	 //打开这个总线，并将总线信息传回结构体,这些数值至少不应该是０，否则工作不正常
	 dsPic = knet_open( "Khepera3:dsPic" , KNET_BUS_I2C , 0 , NULL );
	 mot1 = knet_open( "Khepera3:mot1" , KNET_BUS_I2C , 0 , NULL );
	 mot2 = knet_open( "Khepera3:mot2" , KNET_BUS_I2C , 0 , NULL );

	 if((dsPic = 0) && (mot1 = 0) && (mot2 =0))
	 {
		 //Stop!
		 printf("硬件初始化失败!\n");
		 kb_set_debug_level(2);
	 }

	 //初始化电机控制器1
	 kmot_SetMode( mot1 , kMotModeIdle );
	 kmot_SetSampleTime( mot1 , 1550 );
	 kmot_SetMargin( mot1 , 6 );
	 kmot_SetOptions( mot1 , 0x0 , kMotSWOptWindup | kMotSWOptStopMotorBlk | kMotSWOptDirectionInv );
	 kmot_ResetError( mot1 );
	 kmot_SetBlockedTime( mot1 , 10);
	 kmot_ConfigurePID( mot1 , kMotRegSpeed , 620 , 3 , 10 );
	 kmot_ConfigurePID( mot1 ,kMotRegPos,600,20,30);
	 //kmot_SetSpeedProfile(mot1 ,15000,30);//最大速度约为68.37mm/s,0.5秒加速到极限
	 kmot_SetSpeedProfile(mot1 ,43000,30);


	 //初始化电机控制器2
	 kmot_SetMode( mot2 , kMotModeIdle );
	 kmot_SetSampleTime( mot2 , 1550 );
	 kmot_SetMargin( mot2 , 6 );
	 kmot_SetOptions( mot2 , 0x0 , kMotSWOptWindup | kMotSWOptStopMotorBlk);
	 kmot_ResetError( mot2 );
	 kmot_SetBlockedTime( mot2 , 10);
	 kmot_ConfigurePID( mot2 , kMotRegSpeed , 620 , 3 , 10 );
	 kmot_ConfigurePID( mot2 ,kMotRegPos,600,20,30);
	 //kmot_SetSpeedProfile(mot2 ,15000,30);//最大速度约为68.37mm/s,0.5秒加速到极限
	 kmot_SetSpeedProfile(mot2 ,43000,30);

}



void khepera3_drive_init() {

	int ResponseValue2 = 0;
	 ResponseValue2 = kh3_init();
	 if(ResponseValue2 != 0)
	 {
		 //Stop!
		 printf("kh3_init初始化失败!\n");
		 kb_set_debug_level(2);
	 }

	KTeam_Init_Motor();
	khepera3_motor_init(&(khepera3.motor_left));
	khepera3_motor_init(&(khepera3.motor_right));
	khepera3.motor_left.direction = 1;
	//khepera3.motor_left.direction = -1;
	khepera3.motor_right.direction = 1;
}

/*** Functions to set/get values on both motors ***/

void khepera3_drive_stop() {
	 dsPic = knet_open( "Khepera3:dsPic" , KNET_BUS_I2C , 0 , NULL );
	 mot1 = knet_open( "Khepera3:mot1" , KNET_BUS_I2C , 0 , NULL );
	 mot2 = knet_open( "Khepera3:mot2" , KNET_BUS_I2C , 0 , NULL );

	 kmot_SetMode( mot1 , kMotModeStopMotor );
	 kmot_SetMode( mot2 , kMotModeStopMotor );

	 khepera3.motor_left.current_speed = 0;
	 khepera3.motor_right.current_speed = 0;

}

void khepera3_drive_start() {
	 dsPic = knet_open( "Khepera3:dsPic" , KNET_BUS_I2C , 0 , NULL );
	 mot1 = knet_open( "Khepera3:mot1" , KNET_BUS_I2C , 0 , NULL );
	 mot2 = knet_open( "Khepera3:mot2" , KNET_BUS_I2C , 0 , NULL );

	 kmot_SetMode( mot1 , kMotModeStopMotor );
	 kmot_SetMode( mot2 , kMotModeStopMotor );

	 khepera3.motor_left.current_speed = 0;
	 khepera3.motor_right.current_speed = 0;

	 khepera3_drive_set_current_position(0,0);

}

void khepera3_drive_idle() {
	 dsPic = knet_open( "Khepera3:dsPic" , KNET_BUS_I2C , 0 , NULL );
	 mot1 = knet_open( "Khepera3:mot1" , KNET_BUS_I2C , 0 , NULL );
	 mot2 = knet_open( "Khepera3:mot2" , KNET_BUS_I2C , 0 , NULL );

	 kmot_SetMode( mot1 , kMotModeIdle );
	 kmot_SetMode( mot2 , kMotModeIdle );

	 khepera3.motor_left.current_speed = 0;
	 khepera3.motor_right.current_speed = 0;
}

void khepera3_drive_set_speed(long speed_left, long speed_right) {
	 dsPic = knet_open( "Khepera3:dsPic" , KNET_BUS_I2C , 0 , NULL );
	 mot1 = knet_open( "Khepera3:mot1" , KNET_BUS_I2C , 0 , NULL );
	 mot2 = knet_open( "Khepera3:mot2" , KNET_BUS_I2C , 0 , NULL );

	 kmot_SetPoint( mot1 , kMotRegSpeed , speed_left );
	 kmot_SetPoint( mot2 , kMotRegSpeed,  speed_right );

	 khepera3.motor_left.current_speed = speed_left;
	 khepera3.motor_right.current_speed = speed_right;
}

void khepera3_drive_set_speed_differential(float speed, float forward_coefficient, float differential_coefficient) {
	float speed_left;
	float speed_right;

	//speed_left = speed * (forward_coefficient + differential_coefficient);
	//speed_right = speed * (forward_coefficient - differential_coefficient);

	speed_right = speed * (forward_coefficient + differential_coefficient);
	speed_left = speed * (forward_coefficient - differential_coefficient);

	//printf("$SPEED,%f,%f\n", speed_left, speed_right);
	khepera3_drive_set_speed((int)floorf(speed_left + 0.5), (int)floorf(speed_right + 0.5));
}

void khepera3_drive_set_speed_differential_bounded(float speed, float forward_coefficient, float forward_coefficient_max, float differential_coefficient, float differential_coefficient_max) {
	if (forward_coefficient > forward_coefficient_max) {
		forward_coefficient = forward_coefficient_max;
	}
	if (forward_coefficient < -forward_coefficient_max) {
		forward_coefficient = -forward_coefficient_max;
	}

	if (differential_coefficient > differential_coefficient_max) {
		differential_coefficient = differential_coefficient_max;
	}
	if (differential_coefficient < -differential_coefficient_max) {
		differential_coefficient = -differential_coefficient_max;
	}

	khepera3_drive_set_speed_differential(speed, forward_coefficient, differential_coefficient);
}

void khepera3_drive_set_speed_using_profile(long speed_left, long speed_right) {
	 dsPic = knet_open( "Khepera3:dsPic" , KNET_BUS_I2C , 0 , NULL );
	 mot1 = knet_open( "Khepera3:mot1" , KNET_BUS_I2C , 0 , NULL );
	 mot2 = knet_open( "Khepera3:mot2" , KNET_BUS_I2C , 0 , NULL );

	 kmot_SetPoint( mot1 , kMotRegSpeedProfile , speed_left );
	 kmot_SetPoint( mot2 , kMotRegSpeedProfile,  speed_right );

	 khepera3.motor_left.current_speed = speed_left;
	 khepera3.motor_right.current_speed = speed_right;

}

void khepera3_drive_goto_position(long position_left, long position_right) {
	dsPic = knet_open( "Khepera3:dsPic" , KNET_BUS_I2C , 0 , NULL );
		 mot1 = knet_open( "Khepera3:mot1" , KNET_BUS_I2C , 0 , NULL );
		 mot2 = knet_open( "Khepera3:mot2" , KNET_BUS_I2C , 0 , NULL );

		 unsigned long lpos,rpos;//电机编码器位置信息
		 lpos = kmot_GetMeasure(mot1,kMotRegPos);
		 rpos = kmot_GetMeasure(mot2,kMotRegPos);

		 long ReachPoint_Left = lpos + (long)position_left;
		 long ReachPoint_Right = rpos + (long)position_right;

		 kmot_SetPoint( mot1 , kMotRegPosProfile , ReachPoint_Left );
		 kmot_SetPoint( mot2 , kMotRegPosProfile,  ReachPoint_Right );

		 khepera3.motor_left.current_position = ReachPoint_Left;
		 khepera3.motor_right.current_position = ReachPoint_Right;
}

void khepera3_drive_goto_position_using_profile(long position_left, long position_right) {
	 dsPic = knet_open( "Khepera3:dsPic" , KNET_BUS_I2C , 0 , NULL );
	 mot1 = knet_open( "Khepera3:mot1" , KNET_BUS_I2C , 0 , NULL );
	 mot2 = knet_open( "Khepera3:mot2" , KNET_BUS_I2C , 0 , NULL );

	 unsigned long lpos,rpos;//电机编码器位置信息
	 lpos = kmot_GetMeasure(mot1,kMotRegPos);
	 rpos = kmot_GetMeasure(mot2,kMotRegPos);

	 long ReachPoint_Left = lpos + (long)position_left;
	 long ReachPoint_Right = rpos + (long)position_right;

	 kmot_SetPoint( mot1 , kMotRegPosProfile , ReachPoint_Left );
	 kmot_SetPoint( mot2 , kMotRegPosProfile,  ReachPoint_Right );

	 khepera3.motor_left.current_position = ReachPoint_Left;
	 khepera3.motor_right.current_position = ReachPoint_Right;
}

void khepera3_drive_set_current_position(long position_left, long position_right) {

	dsPic = knet_open( "Khepera3:dsPic" , KNET_BUS_I2C , 0 , NULL );
	mot1 = knet_open( "Khepera3:mot1" , KNET_BUS_I2C , 0 , NULL );
	mot2 = knet_open( "Khepera3:mot2" , KNET_BUS_I2C , 0 , NULL );
	kmot_SetPosition(mot1,position_left);
	kmot_SetPosition(mot2,position_right);

	 khepera3.motor_left.current_position = position_left;
	 khepera3.motor_right.current_position = position_right;
}

void khepera3_drive_get_current_speed() {


}

void khepera3_drive_get_current_position() {
	//khepera3_motor_get_current_position(&(khepera3.motor_left));
	//khepera3_motor_get_current_position(&(khepera3.motor_right));

	 dsPic = knet_open( "Khepera3:dsPic" , KNET_BUS_I2C , 0 , NULL );
	 mot1 = knet_open( "Khepera3:mot1" , KNET_BUS_I2C , 0 , NULL );
	 mot2 = knet_open( "Khepera3:mot2" , KNET_BUS_I2C , 0 , NULL );

	 unsigned long lpos,rpos;//电机编码器位置信息
	 lpos = kmot_GetMeasure(mot1,kMotRegPos);
	 rpos = kmot_GetMeasure(mot2,kMotRegPos);

	 khepera3.motor_left.current_position = lpos;
	 khepera3.motor_right.current_position = rpos;
}

//void khepera3_drive_get_current_torque() {
	//khepera3_motor_get_current_torque(&(khepera3.motor_left));
	//khepera3_motor_get_current_torque(&(khepera3.motor_right));
//}
void KTeam_SetPoint(float Point_Left, float Point_Right)
{
	 dsPic = knet_open( "Khepera3:dsPic" , KNET_BUS_I2C , 0 , NULL );
	 mot1 = knet_open( "Khepera3:mot1" , KNET_BUS_I2C , 0 , NULL );
	 mot2 = knet_open( "Khepera3:mot2" , KNET_BUS_I2C , 0 , NULL );
	//单位为毫米
	 float pulsestomm_Left = PULSE_TO_MM_FIRMWARE_BE_3_Left;
	 float pulsestomm_Right = PULSE_TO_MM_FIRMWARE_BE_3_Right;

	 static int Changer = -1;
	 Changer = Changer * (-1);

	 unsigned long lpos,rpos;//电机编码器位置信息

	 //读取编码器的数值
	 lpos = kmot_GetMeasure(mot1,kMotRegPos);
	 rpos = kmot_GetMeasure(mot2,kMotRegPos);

	 float fpos_Left = Point_Left/pulsestomm_Left;
	 float fpos_Right = Point_Right/pulsestomm_Right;

	 long ReachPoint_Left = lpos + (long)fpos_Left;
	 long ReachPoint_Right = rpos + (long)fpos_Right;

	 if(Changer == 1)
	 {
		 kmot_SetPoint( mot1 , kMotRegPosProfile , ReachPoint_Left );
		 kmot_SetPoint( mot2 , kMotRegPosProfile,  ReachPoint_Right );
	 }

	 if(Changer == -1)
	 {
		 kmot_SetPoint( mot2 , kMotRegPosProfile,  ReachPoint_Right );
		 kmot_SetPoint( mot1 , kMotRegPosProfile , ReachPoint_Left );
	 }
}

void KTeam_MakeACricle(float Radius ,float AngularVelocity)
	{
    //速度单位:rad/s , mm/s
    float b_DistanceRoller = WHEEL_Base;
    float R_Inside = Radius - b_DistanceRoller/2;

    if(Radius < 0)
    {
    	printf("错误的转弯半径: %ld \n",(long)Radius);
    	return;
    }

    if(R_Inside < 0)
    {
    	printf("过小的转弯半径: %ld \n",(long)Radius);
    	return;
    }

    long SpeedLeft = (long)(R_Inside * AngularVelocity);
    long SpeedRight = (long)((R_Inside + b_DistanceRoller)*AngularVelocity);

    //KTeam_SetSpeed(SpeedLeft, SpeedRight);
    khepera3_drive_set_speed(SpeedLeft, SpeedRight);
    }

void OnthePointTurn_Degree(float Turn_Degree)
{
	float BothWheelRun_mm = (Turn_Degree * PI * 89.5746/ (180*2) );
	KTeam_SetPoint(-BothWheelRun_mm, BothWheelRun_mm);

}




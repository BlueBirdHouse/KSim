/*
 * BlueBird_Time.c
 *
 *  Created on: Jan 16, 2014
 *      Author: bluebird
 */
#include "BlueBird_Time.h"

double Time_Second()
{
	//返回以秒为单位的系统时间
	struct timeval tv;
	struct timezone tz;

	gettimeofday (&tv , &tz);
	return (double)tv.tv_sec + ((double)(tv.tv_usec))*(1e-6);

}

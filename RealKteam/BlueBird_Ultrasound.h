/*
 * BlueBird_Ultrasound.h
 *
 *  Created on: Dec 18, 2013
 *      Author: bluebird
 */

#ifndef BLUEBIRD_ULTRASOUND_H_
#define BLUEBIRD_ULTRASOUND_H_

#endif /* BLUEBIRD_ULTRASOUND_H_ */

struct USData
{
	short US_EchoNumber;
	short US_Distence_CM[5];
	short US_Amplitude[5];
	int US_Time_Low_ms[5];
};

void measureUS(int USNumber,struct USData *DataToStore);
int ActiveUS(int USNumber);


/*
 * BlueBird_Ultrasound.c
 *
 *  Created on: Dec 18, 2013
 *      Author: bluebird
 */

/*** Module initialization ***/
#include <korebot/korebot.h> //基本初始化文件
#include <korebot/kb_khepera3.h>

//#include "khepera3.h"

#include "BlueBird_Ultrasound.h"

//一种机器人硬件专用数据结构
static knet_dev_t * dsPic;
static knet_dev_t * mot1;
static knet_dev_t * mot2;

#define MAXBUFFERSIZE 100//声纳的缓冲

#define PI 3.14159265358979

int ActiveUS(int USNumber)
{
	  dsPic = knet_open( "Khepera3:dsPic" , KNET_BUS_I2C , 0 , NULL );
	  mot1 = knet_open( "Khepera3:mot1" , KNET_BUS_I2C , 0 , NULL );
	  mot2 = knet_open( "Khepera3:mot2" , KNET_BUS_I2C , 0 , NULL );

	  char Buffer[MAXBUFFERSIZE];

	  unsigned char ActiveNumber = 0;
	  switch(USNumber)
	  {
	  	  case 1:
	  		ActiveNumber = 1;
	  		break;
	  	  case 2:
	  		ActiveNumber = 2;
	  		break;
	  	  case 3:
	  		ActiveNumber = 4;
	  		break;
	  	  case 4:
	  		ActiveNumber = 8;
	  		break;
	  	  case 5:
	  		ActiveNumber = 16;
	  		break;
	  	  case 6:
	  		ActiveNumber = 31;
	  		break;
	  }

	  if(kh3_configure_os((char *)Buffer, 0, ActiveNumber, dsPic))
	  {
		  printf("\r\n US sensors configured.\r\n");
		  return 1;
	  }
	  else
	  {
		 printf("\r\nconfigure OS error!\r\n");
		 return 0;
	  }

}

void measureUS(int USNumber,struct USData *DataToStore)
{

  dsPic = knet_open( "Khepera3:dsPic" , KNET_BUS_I2C , 0 , NULL );
  mot1 = knet_open( "Khepera3:mot1" , KNET_BUS_I2C , 0 , NULL );
  mot2 = knet_open( "Khepera3:mot2" , KNET_BUS_I2C , 0 , NULL );

  char Buffer[MAXBUFFERSIZE];
  int i;
  //short usnoise;	/* Noise on the given adc pin when no us is received */
  short echonbr;	/* Number of echo part of this us measure */
  short usvalue;	/* Variable that handle distances */
  short usampl;		/* Variable than nandle amplitudes */
  //int Answer = kh3_measure_us((char *)Buffer, USNumber, dsPic);
  //Answer = 0;
  unsigned char ActiveNumber = 0;
  switch(USNumber)
  {
  	  case 1:
  		ActiveNumber = 1;
  		break;
  	  case 2:
  		ActiveNumber = 2;
  		break;
  	  case 3:
  		ActiveNumber = 4;
  		break;
  	  case 4:
  		ActiveNumber = 8;
  		break;
  	  case 5:
  		ActiveNumber = 16;
  		break;
  	  case 6:
  		ActiveNumber = 31;
  		break;
  }


  if(kh3_measure_us((char *)Buffer, USNumber, dsPic))
  {
	/* Printout complete frame as received by the khepera3 */
  	printf("\r\n%c", Buffer[0]);
	for(i = 0; i < 22; i++)
		printf(",%4.4u", (Buffer[1+i*2] | Buffer[2+i*2]<<8));
	printf("\r\n");

	/* We guess the echo number ( how many echo has been received from a captor ) */
	echonbr = (Buffer[1] | Buffer[2]<<8);
	printf("echonbr = %d\r\n", echonbr);

	/* Loop as may time it is required */
	if(echonbr < 10)
	{
		DataToStore->US_EchoNumber = echonbr;
		for(i = 0; i < echonbr ; i++)
		{
			/* Get the distance measure from one echo */
			usvalue = (Buffer[i*8+3] | Buffer[i*8+4]<<8);
			usampl  = (Buffer[i*8+5] | Buffer[i*8+6]<<8);

			/* Print out the result */
			printf("Echo %d : Amplitude = %d,  Distance = %dcm.\r\n", i+1, usampl, usvalue);

			DataToStore->US_Distence_CM[i] = usvalue;
			DataToStore->US_Amplitude[i] = usampl;

			DataToStore->US_Time_Low_ms[i] = (Buffer[i*8+7] | Buffer[i*8+8]<<8);

		}
	}
	else
		printf("read error\r\n");
  }
  else
  	printf("\r\ng, error...");

}

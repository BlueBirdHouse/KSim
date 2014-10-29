

#include <assert.h>
#include "khepera3_infrared.h"
#include "khepera3.h"

/*** Module initialization ***/
#include <korebot/korebot.h> //基本初始化文件
#include <korebot/kb_khepera3.h>

//一种机器人硬件专用数据结构
static knet_dev_t * dsPic;
static knet_dev_t * mot1;
static knet_dev_t * mot2;

#define BUFFERSIZE 30//声纳的缓冲

/*** Module initialization ***/

void khepera3_infrared_init() {
}

/*** Low-level functions ***/
/*
int khepera3_infrared_p(struct sKhepera3SensorsInfrared *result, char infrared_command) {
	struct i2c_msg *msg_read;
	int i;

	// Arguments/environment checks
	assert(result != 0);
	assert((infrared_command == 'N') || (infrared_command == 'O'));

	// Send infrared command (N or O) and read 27 bytes
	i2cal_start();
	i2cal_writedata_uint8(2);
	i2cal_writedata_uint8(infrared_command);
	i2cal_write(khepera3.dspic.i2c_address);
	msg_read = i2cal_read(khepera3.dspic.i2c_address, 27);
	if (i2cal_commit() < 0) {
		return 0;
	}

	// Decode sensor values
	for (i = 0; i < 11; i++) {
		result->sensor[i] = i2cal_readdata_int16(msg_read, i * 2 + 3);
	}

	// Decode timestamp
	result->timestamp = i2cal_readdata_uint16(msg_read, 25);

	// Everything OK
	return -1;
}
*/
/*** High-level functions ***/

int khepera3_infrared_ambient()
{
	char Buffer[BUFFERSIZE];
	dsPic  = knet_open( "Khepera3:dsPic" , KNET_BUS_I2C , 0 , NULL );
	mot1  = knet_open( "Khepera3:mot1" , KNET_BUS_I2C , 0 , NULL );
	mot2  = knet_open( "Khepera3:mot2" , KNET_BUS_I2C , 0 , NULL );

	int i;

	if(kh3_ambiant_ir((char *)Buffer, dsPic))
	{
	  for (i=0;i<11;i++)
	  {
		  khepera3.infrared_ambient.sensor[i] = (Buffer[i*2+1] | Buffer[i*2+2]<<8);
	  }
	  khepera3.infrared_ambient.timestamp = ((Buffer[19]|Buffer[20] << 8)|(Buffer[21]|Buffer[22] << 8) << 16);
	  return 1;
	}
	return 0;
}

int khepera3_infrared_ambient_p(struct sKhepera3SensorsInfrared *result) {
	//return khepera3_infrared_p(result, 'O');
	return 0;
}

int khepera3_infrared_proximity()
{
	char Buffer[BUFFERSIZE];
	dsPic  = knet_open( "Khepera3:dsPic" , KNET_BUS_I2C , 0 , NULL );
	mot1  = knet_open( "Khepera3:mot1" , KNET_BUS_I2C , 0 , NULL );
	mot2  = knet_open( "Khepera3:mot2" , KNET_BUS_I2C , 0 , NULL );

	int i;

	if(kh3_proximity_ir((char *)Buffer, dsPic))
	{
	  for (i=0;i<11;i++)
	  {
		  khepera3.infrared_proximity.sensor[i] = (Buffer[i*2+1] | Buffer[i*2+2]<<8);
	  }
	  khepera3.infrared_proximity.timestamp = ((Buffer[19]|Buffer[20] << 8)|(Buffer[21]|Buffer[22] << 8) << 16);
	  return 1;
	}
	return 0;
}

int khepera3_infrared_proximity_p(struct sKhepera3SensorsInfrared *result)
{
	//return khepera3_infrared_p(result, 'N');
	return 0;
}



#include "i2cal.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stropts.h>

// I2C filehandle
int i2cal_fd = -1;

void i2cal_init() {
	i2cal_open();
	i2cal_start();
}

int i2cal_open() {
	// If already open, do nothing
	if (i2cal_fd >= 0) {
		return 2;
	}

	// Try to open
	i2cal_fd = open("/dev/i2c/0", O_RDWR);
	if (i2cal_fd < 0) {
		printf("Fatal error: unable to open the I2C bus (/dev/i2c/0): res=%d\n", i2cal_fd);
		exit(255);
		//return 0;
	}

	return 1;
}

void i2cal_close() {
	if (i2cal_fd < 0) {
		return;
	}
	close(i2cal_fd);
	i2cal_fd = -1;
}

void i2cal_start() {
	// Initialize variables for the next transfer
	i2cal.messages_nw = 0;
	i2cal.write_buffer_nw = 0;
	i2cal.write_buffer_start = 0;
	i2cal.read_buffer_nw = 0;
}

struct i2c_msg *i2cal_read_buffer(int i2c_address, unsigned char *buffer, int len) {
	struct i2c_msg *msg;

	// If our message buffer is full, give up
	if (i2cal.messages_nw > I2CAL_MESSAGES_LEN) {
		return 0;
	}

	// Create and add a new read message
	msg = &(i2cal.messages[i2cal.messages_nw]);
	i2cal.messages_nw++;

	msg->addr  = i2c_address;
	msg->flags = I2C_M_RD;
	msg->buf   = buffer;
	msg->len   = len;
	//i2cal.read_buffer_nw+=len;

	return msg;
}

struct i2c_msg *i2cal_write_buffer(int i2c_address, unsigned char *buffer, int len) {
	struct i2c_msg *msg;

	// If our message buffer is full, give up
	if (i2cal.messages_nw > I2CAL_MESSAGES_LEN) {
		return 0;
	}

	// Create and add a new write message
	msg = &(i2cal.messages[i2cal.messages_nw]);
	i2cal.messages_nw++;

	msg->addr  = i2c_address;
	msg->flags = 0;  // TODO: add proper flag
	msg->buf   = buffer;
	msg->len   = len;
	//i2cal.write_buffer_start=i2cal.write_buffer_nw;

	return msg;
}

struct i2c_msg *i2cal_read(int i2c_address, int len) {
	struct i2c_msg *msg;

	// If our read buffer would be full, give up
	if (i2cal.read_buffer_nw + len > I2CAL_READ_BUFFER_LEN) {
		return 0;
	}

	// Add a new read message and reserve the necessary space in the read buffer
	msg = i2cal_read_buffer(i2c_address, &(i2cal.read_buffer[i2cal.read_buffer_nw]), len);
	if (msg == 0) {
		return 0;
	}
	i2cal.read_buffer_nw += len;

	return msg;
}

struct i2c_msg *i2cal_write(int i2c_address) {
	struct i2c_msg *msg;

	// If our write buffer would be full, give up
	if (i2cal.write_buffer_nw > I2CAL_WRITE_BUFFER_LEN) {
		return 0;
	}

	// Add a new write message with the content of the write buffer
	msg = i2cal_write_buffer(i2c_address, &(i2cal.write_buffer[i2cal.write_buffer_start]), i2cal.write_buffer_nw - i2cal.write_buffer_start);
	if (msg == 0) {
		return 0;
	}
	i2cal.write_buffer_start = i2cal.write_buffer_nw;

	return msg;
}

unsigned char *i2cal_writedata_uint8(unsigned char value) {
	unsigned char *buf = i2cal_writedata_buffer(1);

	if (buf == 0) {
		return 0;
	}
	*buf = value;
	return buf;
}

unsigned char *i2cal_writedata_int16(int value) {
	unsigned char *arr = (unsigned char *)(&value);
	unsigned char *buf = i2cal_writedata_buffer(2);

	if (buf == 0) {
		return 0;
	}
	buf[0] = arr[0];
	buf[1] = arr[1];
	return buf;
}

unsigned char *i2cal_writedata_uint16(unsigned int value) {
	unsigned char *arr = (unsigned char *)(&value);
	unsigned char *buf = i2cal_writedata_buffer(2);

	if (buf == 0) {
		return 0;
	}
	buf[0] = arr[0];
	buf[1] = arr[1];
	return buf;
}

unsigned char *i2cal_writedata_int32(int value) {
	unsigned char *arr = (unsigned char *)(&value);
	unsigned char *buf = i2cal_writedata_buffer(4);

	if (buf == 0) {
		return 0;
	}
	buf[0] = arr[0];
	buf[1] = arr[1];
	buf[2] = arr[2];
	buf[3] = arr[3];
	return buf;
}

unsigned char *i2cal_writedata_uint32(unsigned int value) {
	unsigned char *arr = (unsigned char *)(&value);
	unsigned char *buf = i2cal_writedata_buffer(4);

	if (buf == 0) {
		return 0;
	}
	buf[0] = arr[0];
	buf[1] = arr[1];
	buf[2] = arr[2];
	buf[3] = arr[3];
	return buf;
}

unsigned char *i2cal_writedata_float(float value) {
	unsigned char *arr = (unsigned char *)(&value);
	unsigned char *buf = i2cal_writedata_buffer(4);

	if (buf == 0) {
		return 0;
	}
	buf[0] = arr[0];
	buf[1] = arr[1];
	buf[2] = arr[2];
	buf[3] = arr[3];
	return buf;
}

unsigned char *i2cal_writedata_buffer(int len) {
	unsigned char *buf;

	// Reserve bytes in the write buffer (which can then be filled by other functions)
	if (i2cal.write_buffer_nw + len > I2CAL_WRITE_BUFFER_LEN) {
		i2cal.write_buffer_nw = I2CAL_WRITE_BUFFER_LEN + 1;
		return 0;
	}
	buf = &(i2cal.write_buffer[i2cal.write_buffer_nw]);
	i2cal.write_buffer_nw += len;
	return buf;
}

int i2cal_commit() {
	struct i2c_rdwr_ioctl_data msgset;

	// Create a message set
	msgset.msgs = i2cal.messages;
	msgset.nmsgs = i2cal.messages_nw;

	// Execute that message set
	return ioctl(i2cal_fd, I2C_RDWR, &msgset);
}

unsigned char i2cal_readdata_uint8(struct i2c_msg *message, int offset) {
	return message->buf[offset];
}

int i2cal_readdata_int16(struct i2c_msg *message, int offset) {
	unsigned char arr[2];
	short int *value = (short int *)arr;

	arr[0] = message->buf[offset];
	arr[1] = message->buf[++offset];
	return (int)(*value);
}

unsigned int i2cal_readdata_uint16(struct i2c_msg *message, int offset) {
	unsigned char arr[4];
	unsigned int *value = (unsigned int *)arr;

	arr[0] = message->buf[offset];
	arr[1] = message->buf[++offset];
	arr[2] = 0;
	arr[3] = 0;
	return *value;
}

int i2cal_readdata_int32(struct i2c_msg *message, int offset) {
	unsigned char arr[4];
	int *value = (int *)arr;

	arr[0] = message->buf[offset];
	arr[1] = message->buf[++offset];
	arr[2] = message->buf[++offset];
	arr[3] = message->buf[++offset];
	return *value;
}

unsigned int i2cal_readdata_uint32(struct i2c_msg *message, int offset) {
	unsigned char arr[4];
	unsigned int *value = (unsigned int *)arr;

	arr[0] = message->buf[offset];
	arr[1] = message->buf[++offset];
	arr[2] = message->buf[++offset];
	arr[3] = message->buf[++offset];
	return *value;
}

float i2cal_readdata_float(struct i2c_msg *message, int offset) {
	unsigned char arr[4];
	float *value = (float *)arr;

	arr[0] = message->buf[offset];
	arr[1] = message->buf[++offset];
	arr[2] = message->buf[++offset];
	arr[3] = message->buf[++offset];
	return *value;
}



#ifndef I2CAL
#define I2CAL

#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

// Buffer lengths
#define I2CAL_MESSAGES_LEN 16
#define I2CAL_WRITE_BUFFER_LEN 256
#define I2CAL_READ_BUFFER_LEN 256

// Data structure holding all buffers for one transaction
struct sI2CAL {
	struct i2c_msg messages[I2CAL_MESSAGES_LEN];
	int messages_nw;
	unsigned char write_buffer[I2CAL_WRITE_BUFFER_LEN];
	int write_buffer_nw;
	int write_buffer_start;
	unsigned char read_buffer[I2CAL_READ_BUFFER_LEN];
	int read_buffer_nw;
};

struct sI2CAL i2cal;

// Initialization
void i2cal_init();

// (private) Open, close
int i2cal_open();
void i2cal_close();

// Start a new transaction
void i2cal_start();

// Add general read/write messages
struct i2c_msg *i2cal_read_buffer(int i2c_address, unsigned char *buffer, int len);
struct i2c_msg *i2cal_write_buffer(int i2c_address, unsigned char *buffer, int len);

// Simple read/write, using built-in read/write buffers (use i2cal_writedata_* functions)
struct i2c_msg *i2cal_read(int i2c_address, int len);
struct i2c_msg *i2cal_write(int i2c_address);

// Write data to built-in buffer
unsigned char *i2cal_writedata_uint8(unsigned char value);
unsigned char *i2cal_writedata_int16(int value);
unsigned char *i2cal_writedata_uint16(unsigned int value);
unsigned char *i2cal_writedata_int32(int value);
unsigned char *i2cal_writedata_uint32(unsigned int value);
unsigned char *i2cal_writedata_float(float value);
unsigned char *i2cal_writedata_buffer(int len);

// Commit transaction by sending/receiving data. Returns the ioctl return value, which is negative on error.
int i2cal_commit();

// Read data from result buffer
unsigned char i2cal_readdata_uint8(struct i2c_msg *message, int offset);
int i2cal_readdata_int16(struct i2c_msg *message, int offset);
unsigned int i2cal_readdata_uint16(struct i2c_msg *message, int offset);
int i2cal_readdata_int32(struct i2c_msg *message, int offset);
unsigned int i2cal_readdata_uint32(struct i2c_msg *message, int offset);
float i2cal_readdata_float(struct i2c_msg *message, int offset);

#endif

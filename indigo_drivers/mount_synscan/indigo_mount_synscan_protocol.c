//
//  indigo_mount_synscan_protocol.c
//  indigo_server
//
//  Created by David Hulse on 07/08/2018.
//  Copyright © 2018 CloudMakers, s. r. o. All rights reserved.
//

#include <errno.h>
#include <pthread.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/time.h>
#include "indigo_driver.h"
#include "indigo_io.h"
#include "indigo_mount_synscan_private.h"
#include "indigo_mount_synscan_protocol.h"


#define SYNSCAN_START_CHAR								 ':'
#define SYNSCAN_REPLY_CHAR								 '='
#define SYNSCAN_ERROR_CHAR								 '!'
#define SYNSCAN_END_CHAR									 '\r'

#define SYNSCAN_FIRMWARE_VERSION					 ":e1"



enum MountType {
	kMountTypeEQ6 = 0x00,
	kMountTypeHEQ5 = 0x01,
	kMountTypeEQ5 = 0x02,
	kMountTypeEQ3 = 0x03,
	kMountTypeGT = 0x80,
	kMountTypeMF = 0x81,
	kMountType114GT = 0x82,
	kMountTypeDOB = 0x90
};


//  HELPERS

static const char hexDigit[] = "0123456789ABCDEF";

static inline int hexValue(char h) {
	if (h >= '0' && h <= '9')
		return h - '0';
	else if (h >= 'A' && h <= 'F')
		return h - 'A' + 10;
	else
		return 0;
}

static const char* longToHex(long n) {
	static char num[7];
	num[1] = hexDigit[n & 0xF];	n >>= 4;
	num[0] = hexDigit[n & 0xF];	n >>= 4;
	num[3] = hexDigit[n & 0xF];	n >>= 4;
	num[2] = hexDigit[n & 0xF];	n >>= 4;
	num[5] = hexDigit[n & 0xF];	n >>= 4;
	num[4] = hexDigit[n & 0xF];
	num[6] = 0;
	return num;
}

static long hexToLong(const char* b) {
	long num = 0;
	while (*b != 0) {
		num <<= 4;
		num |= hexValue(*b);
		b++;
	}
	return num;
}

static long hexResponseToLong(const char* b) {
	long num = 0;
	num |= hexValue(b[4]); num <<= 4;
	num |= hexValue(b[5]); num <<= 4;
	num |= hexValue(b[2]); num <<= 4;
	num |= hexValue(b[3]); num <<= 4;
	num |= hexValue(b[0]); num <<= 4;
	num |= hexValue(b[1]);
	return num;
}

//  GENERIC SYNSCAN COMMAND

static bool synscan_command(indigo_device* device, const char* cmd, char* r) {
	int nretries = 0;
	pthread_mutex_lock(&PRIVATE_DATA->port_mutex);
	while (nretries < 2) {
		//  Count the attempts
		nretries++;

		//  Flush input
		unsigned char c;
		struct timeval tv;
		while (true) {
			fd_set readout;
			FD_ZERO(&readout);
			FD_SET(PRIVATE_DATA->handle, &readout);
			tv.tv_sec = 0;
			tv.tv_usec = 100000;
			long result = select(1, &readout, NULL, NULL, &tv);
			if (result == 0)
				break;
			if (result < 0) {
				pthread_mutex_unlock(&PRIVATE_DATA->port_mutex);
				printf("SELECT FAIL 1\n");
				return false;
			}
			result = read(PRIVATE_DATA->handle, &c, 1);
			if (result < 1) {
				pthread_mutex_unlock(&PRIVATE_DATA->port_mutex);
				printf("READ FAIL 1\n");
				return false;
			}
		}

		//  Send the command to the port
		printf("CMD: [%s]  -->  ", cmd);
		if (!indigo_write(PRIVATE_DATA->handle, cmd, strlen(cmd))) {
			printf("Sending command failed\n");
			break;
		}
		if (!indigo_write(PRIVATE_DATA->handle, "\r", 1)) {
			printf("Sending command terminator failed\n");
			break;
		}

		//  Read a response
		char resp[20];
		long total_bytes = 0;
		while (total_bytes < sizeof(resp)) {
			long bytes_read = read(PRIVATE_DATA->handle, &c, 1);
			if (bytes_read == 0) {
				printf("SYNSCAN_TIMEOUT\n");
				break;
			}
			if (bytes_read > 0) {
				if (c == SYNSCAN_END_CHAR)
					break;
				resp[total_bytes++] = c;
			}
		}
		resp[total_bytes] = 0;
		if (total_bytes <= 0) {
			printf("Reading response failed\n");
			continue;
		}

		//  Check response syntax [=|!]...<cr>, if invalid retry
		size_t len = strlen(resp);
		printf("[%s]\n", resp);
		if (len == 0 || (resp[0] != SYNSCAN_REPLY_CHAR && resp[0] != SYNSCAN_ERROR_CHAR)) {
			printf("Response syntax error\n");
			continue;
		}

		//  Check for error response
		if (resp[0] == SYNSCAN_ERROR_CHAR) {
			printf("ERROR response code: %s\n", resp+1);
			pthread_mutex_unlock(&PRIVATE_DATA->port_mutex);
			return false;
		}

		//  Extract response payload, return
		if (r) {
			strncpy(r, resp+1, len-1);
			r[len-1] = 0;
		}
		pthread_mutex_unlock(&PRIVATE_DATA->port_mutex);
		return true;
	}

	//  Mount command failed
	pthread_mutex_unlock(&PRIVATE_DATA->port_mutex);
	return false;
}

#if 0
static bool synscan_command(indigo_device *device, char *command, char *response, int max, int sleep) {
	pthread_mutex_lock(&PRIVATE_DATA->port_mutex);
	unsigned char c;
	struct timeval tv;
	// flush
//	while (true) {
//		fd_set readout;
//		FD_ZERO(&readout);
//		FD_SET(PRIVATE_DATA->handle, &readout);
//		tv.tv_sec = 0;
//		tv.tv_usec = 100000;
//		long result = select(1, &readout, NULL, NULL, &tv);
//		if (result == 0)
//			break;
//		if (result < 0) {
//			pthread_mutex_unlock(&PRIVATE_DATA->port_mutex);
//			printf("SELECT FAIL 1\n");
//			return false;
//		}
//		result = read(PRIVATE_DATA->handle, &c, 1);
//		if (result < 1) {
//			pthread_mutex_unlock(&PRIVATE_DATA->port_mutex);
//			printf("READ FAIL 1\n");
//			return false;
//		}
//	}
	if (tcflush(PRIVATE_DATA->handle, TCIOFLUSH) < 0) {
		printf("FLUSH ERR\n");
	}
	// write command
	indigo_write(PRIVATE_DATA->handle, command, strlen(command));
//	if (write(PRIVATE_DATA->handle, ":e1\r", 4) != 4) {
//		printf("WRITE FAILED\n");
//	}
//	if (write(PRIVATE_DATA->handle, ":", 1) != 1) {
//		printf("WRITE FAILED\n");
//	}
//	usleep(100000);
//	if (write(PRIVATE_DATA->handle, "e", 1) != 1) {
//		printf("WRITE FAILED\n");
//	}
//	usleep(100000);
//	if (write(PRIVATE_DATA->handle, "1", 1) != 1) {
//		printf("WRITE FAILED\n");
//	}
//	usleep(100000);
//	if (write(PRIVATE_DATA->handle, "\r", 1) != 1) {
//		printf("WRITE FAILED\n");
//	}
//	usleep(100000);
//	if (write(PRIVATE_DATA->handle, "\n", 1) != 1) {
//		printf("WRITE FAILED\n");
//	}
//	usleep(100000);

	indigo_write(PRIVATE_DATA->handle, "\r", 1);
	if (sleep > 0)
		usleep(sleep);
	// read response
	if (response != NULL) {
		int index = 0;
		int remains = max;
		int timeout = 3;
		while (remains > 0) {
			fd_set readout;
			FD_ZERO(&readout);
			FD_SET(PRIVATE_DATA->handle, &readout);
			tv.tv_sec = 3;//timeout;
			tv.tv_usec = 0;//1000000;
			timeout = 0;
			long result = select(PRIVATE_DATA->handle+1, &readout, NULL, NULL, &tv);
			if (result <= 0) {
				printf("SELECT FAIL 2   %ld\n", result);
				break;
			}
			result = read(PRIVATE_DATA->handle, &c, 1);
			if (result < 1) {
				printf("READ FAIL 2\n");
				INDIGO_DRIVER_ERROR(DRIVER_NAME, "Failed to read from %s -> %s (%d)", DEVICE_PORT_ITEM->text.value, strerror(errno), errno);
				pthread_mutex_unlock(&PRIVATE_DATA->port_mutex);
				return false;
			}
			//if (c < 0)
			//	c = ':';
			if (index == 0 && c == '=')
				continue;
			if (c == '\r')
				break;
			response[index++] = c;
		}
//		int foo = indigo_read_line2(PRIVATE_DATA->handle, response, 19);
//		printf("foo %d\n", foo);

		response[index] = 0;
	}
	pthread_mutex_unlock(&PRIVATE_DATA->port_mutex);
	printf("Command %s -> %s", command, response != NULL ? response : "NULL");
	INDIGO_DRIVER_DEBUG(DRIVER_NAME, "Command %s -> %s", command, response != NULL ? response : "NULL");
	return true;
}
#endif

static bool synscan_command_with_long_result(indigo_device* device, char* cmd, long* val) {
	char buffer[20];
	if (!synscan_command(device, cmd, buffer))
		return false;
	if (val)
		*val = hexResponseToLong(buffer);
	return true;
}

static bool synscan_command_with_code_result(indigo_device* device, char* cmd, long* val) {
	char buffer[20];
	if (!synscan_command(device, cmd, buffer))
		return false;
	if (val)
		*val = hexToLong(buffer);
	return true;
}

bool synscan_firmware_version(indigo_device* device, long* v) {
	return synscan_command_with_long_result(device, ":e1", v);
}

//  Returns number of steps required to rotate the axis 360 degrees
bool synscan_total_axis_steps(indigo_device* device, enum AxisID axis, long* v) {
	char buffer[5];
	sprintf(buffer, ":a%c", axis);
	return synscan_command_with_long_result(device, buffer, v);
}

//  Returns number of steps required to rotate the axis worm gear 360 degrees.
//  Multiple worm rotations are required to rotate the axis 360 degrees.
//  This data is useful for determining the period of any PE.

//  pecPeriodSteps
bool synscan_worm_rotation_steps(indigo_device* device, enum AxisID axis, long* v) {
	char buffer[5];
	sprintf(buffer, ":s%c", axis);
	return synscan_command_with_long_result(device, buffer, v);
}

bool synscan_step_timer_frequency(indigo_device* device, enum AxisID axis, long* v) {
	char buffer[5];
	sprintf(buffer, ":b%c", axis);
	return synscan_command_with_long_result(device, buffer, v);
}

//  speed_steps = rads/sec * (timer_freq / (axisSteps / 2PI))
//              = rads/sec * (steps/axisSteps * 2PI/sec)

bool synscan_high_speed_ratio(indigo_device* device, enum AxisID axis, long* v) {
	char buffer[5];
	sprintf(buffer, ":g%c", axis);
	return synscan_command_with_long_result(device, buffer, v);
}

bool synscan_motor_status(indigo_device* device, enum AxisID axis, long* v) {
	char buffer[5];
	sprintf(buffer, ":f%c", axis);
	return synscan_command_with_code_result(device, buffer, v);
}

//  Read back the motor position
bool synscan_axis_position(indigo_device* device, enum AxisID axis, long* v) {
	char buffer[5];
	sprintf(buffer, ":j%c", axis);
	return synscan_command_with_long_result(device, buffer, v);
}

//  Set the encoder reference position for a given axis
bool synscan_init_axis_position(indigo_device* device, enum AxisID axis, long pos) {
	char buffer[11];
	sprintf(buffer, ":E%c%s", axis, longToHex(pos));
	//printf("*************************************************** INIT AXIS %s  (pos is %ld / 0x%lX\n", buffer, pos, pos);
	return synscan_command(device, buffer, NULL);
}

//  Energise the motor for a given axis
bool synscan_init_axis(indigo_device* device, enum AxisID axis) {
	char buffer[5];
	sprintf(buffer, ":F%c", axis);
	return synscan_command(device, buffer, NULL);
}

//  Stop any motion for the given axis
bool synscan_stop_axis(indigo_device* device, enum AxisID axis) {
	char buffer[5];
	sprintf(buffer, ":K%c", axis);
	return synscan_command(device, buffer, NULL);
}

//  Stop any motion for the given axis
bool synscan_instant_stop_axis(indigo_device* device, enum AxisID axis) {
	char buffer[5];
	sprintf(buffer, ":L%c", axis);
	return synscan_command(device, buffer, NULL);
}

//  Set gearing for the given axis
//  setAxisMode
bool synscan_set_axis_gearing(indigo_device* device, enum AxisID axis, enum AxisDirectionID d, enum AxisSpeedID g) {
	static char codes[2] = {'0', '1'}/*, {'2', '3'}}*/;
	char buffer[7];
	sprintf(buffer, ":G%c%c%c", axis, g, codes[d]);
	//printf(buffer);
	return synscan_command(device, buffer, NULL);
}

//  Set slew step count for the given axis
//  setAxisTargetIncrement
bool synscan_set_axis_step_count(indigo_device* device, enum AxisID axis, long s) {
	char buffer[11];
	sprintf(buffer, ":H%c%s", axis, longToHex(s));
	//printf(buffer);
	return synscan_command(device, buffer, NULL);
}

//  Set slew rate for the given axis
//  setAxisStepPeriod
bool synscan_set_axis_slew_rate(indigo_device* device, enum AxisID axis, long r) {
	char buffer[11];
	sprintf(buffer, ":I%c%s", axis, longToHex(r));
	//printf(buffer);
	return synscan_command(device, buffer, NULL);
}

//  Start slewing the given axis
//  startAxis
bool synscan_slew_axis(indigo_device* device, enum AxisID axis) {
	char buffer[5];
	sprintf(buffer, ":J%c", axis);
	//printf(buffer);
	return synscan_command(device, buffer, NULL);
}

//  Set slew slowdown count for the given axis
//  setAxisBreakpointIncrement
bool synscan_set_axis_slowdown(indigo_device* device, enum AxisID axis, long s) {
	char buffer[11];
	sprintf(buffer, ":M%c%s", axis, longToHex(s));
	//printf(buffer);
	return synscan_command(device, buffer, NULL);
}

//  setAxisBreakSteps:(enum AxisID)axis steps:(unsigned long)s
//     :U...

//  setSwitch (on/off)
//     :O10\r    :O11\r

bool synscan_set_polarscope_brightness(indigo_device* device, unsigned char brightness) {
	char buffer[7];
	sprintf(buffer, ":V1%02X", brightness);
	return synscan_command(device, buffer, NULL);
}

bool synscan_set_st4_guide_rate(indigo_device* device, enum AxisID axis, int rate) {
	char buffer[7];
	sprintf(buffer, ":P%c", axis);
	return synscan_command(device, buffer, NULL);
}


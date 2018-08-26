//
//  indigo_mount_synscan_protocol.h
//  indigo_server
//
//  Created by David Hulse on 07/08/2018.
//  Copyright © 2018 CloudMakers, s. r. o. All rights reserved.
//

#ifndef indigo_mount_synscan_protocol_h
#define indigo_mount_synscan_protocol_h

#include <stdbool.h>

enum AxisID {
	kAxisRA = '1',
	kAxisDEC = '2'
};

enum AxisDirectionID {
	kAxisDirectionFwd = 0,
	kAxisDirectionRev = 1
};

enum AxisSpeedID {
	kAxisSpeedHigh = '3',
	kAxisSpeedLow = '1',
	kAxisSpeedAbsSlew = '0'
};

enum MotorStatus {
	//kStatusNotInitialised = 0x100,


//	kStatusIdleForward = 0x100,
//	kStatusActiveForward = 0x110,
//	kStatusActiveBackward = 0x310,
//	kStatusIdleBackward = 0x300,
//	kStatusIdleTurboForward = 0x500,
//	kStatusActiveTurboForward = 0x510,
//	kStatusIdleTurboBackward = 0x700,
//	kStatusActiveTurboBackward = 0x710,

	kStatusSlewing = 0x100,
	kStatusSlewingTo = 0x000,
	kStatusForward = 0x000,
	kStatusBackward = 0x200,
	kStatusTurbo = 0x400,
	kStatusNormal = 0x000,

	kStatusActiveMask = 0x010,
	kStatusInitMask = 0x001
	//=      0x1 == slewing, 0x0 == slewingTo, 0x2==slewing back, 0x4==turbo       0 == stopped, 1 == moving           0==not init, 1==init ok
};

bool synscan_firmware_version(indigo_device* device, long* v);
bool synscan_total_axis_steps(indigo_device* device, enum AxisID axis, long* v);
bool synscan_worm_rotation_steps(indigo_device* device, enum AxisID axis, long* v);
bool synscan_step_timer_frequency(indigo_device* device, enum AxisID axis, long* v);
bool synscan_high_speed_ratio(indigo_device* device, enum AxisID axis, long* v);
bool synscan_motor_status(indigo_device* device, enum AxisID axis, long* v);
bool synscan_axis_position(indigo_device* device, enum AxisID axis, long* v);
bool synscan_init_axis_position(indigo_device* device, enum AxisID axis, long pos);
bool synscan_init_axis(indigo_device* device, enum AxisID axis);
bool synscan_stop_axis(indigo_device* device, enum AxisID axis);
bool synscan_instant_stop_axis(indigo_device* device, enum AxisID axis);
bool synscan_set_axis_gearing(indigo_device* device, enum AxisID axis, enum AxisDirectionID d, enum AxisSpeedID g);
bool synscan_set_axis_step_count(indigo_device* device, enum AxisID axis, long s);
bool synscan_set_axis_slew_rate(indigo_device* device, enum AxisID axis, long r);
bool synscan_slew_axis(indigo_device* device, enum AxisID axis);
bool synscan_set_axis_slowdown(indigo_device* device, enum AxisID axis, long s);
bool synscan_set_polarscope_brightness(indigo_device* device, unsigned char brightness);

#endif /* indigo_mount_synscan_protocol_h */

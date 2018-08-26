//
//  indigo_mount_synscan_axis.c
//  indigo_server
//
//  Created by David Hulse on 23/08/2018.
//  Copyright © 2018 CloudMakers, s. r. o. All rights reserved.
//

#include "indigo_mount_synscan_private.h"
#include "indigo_mount_synscan_axis.h"

//  stop_axis
//  slew axis at rate
//  slew axis to position
//  axis position

//  There are 4 possible things an axis can be doing. It can only be doing one at a time.
//		Stopped / Idle
//		Slewing at a rate indefinitely
//    Slewing to a position
//    Slewing at a rate for a pulse

//  If the axis is doing one thing and it is required to do another, then typically it must first be stopped.

//  The need to make an axis do something will come from higher level operations, generally running in the background.
//  Therefore, these operations should be synchronous, but abortable.


void
axis_management_thread(indigo_device* device, enum AxisID axis) {
	//  This thread should monitor what the axis is doing, basically at all times
	//  And when commanded, should move the axis from one state to another
}

//  The axis API should only be invoked by the driver global operations stuff

//  The global ops stuff should operate asynchronously. Basically operations are started by events when properties change
//  Background operations will make properties busy and when the operations are done, the properties will become OK again.

//  What happens if a client tries to change a property that is busy?

//  How to recover from a property that is in a state of alert?


bool mount_wait_for_axis_stopped(indigo_device* device, enum AxisID axis) {
	while (true) {
		//  Get the axis status
		long axisStatus;
		if (!synscan_motor_status(device, axis, &axisStatus))
			return false;

		//  Mark axis as stopped if motor has come to rest
		if ((axisStatus & kStatusActiveMask) == 0) {
			//*axisMode = kAxisModeIdle;
			return true;
		}
	}
}

bool mount_stop_axis(indigo_device* device, enum AxisID axis) {
	if (!synscan_stop_axis(device, axis))
		return false;

	return mount_wait_for_axis_stopped(device, axis);
}

static bool synscan_should_reconfigure_axis(indigo_device* device, enum AxisID axis, struct AxisConfig* config) {
	//  Check if we need to re-configure
	//  Mount seems to lose the turbo status and needs to be reconfigured each time
	if (config->turbo)
		return true;

	//  Reference the relevant axis config cache
	struct AxisConfig* cachedConfig = (axis == kAxisRA) ? &PRIVATE_DATA->raAxisConfig : &PRIVATE_DATA->decAxisConfig;

	//  Check if reconfiguration is needed
	if (!cachedConfig->valid)
		return true;
	if (cachedConfig->direction != config->direction)
		return true;
	if (cachedConfig->turbo)
		return true;
	return false;
}

static void synscan_axis_config_for_rate(indigo_device* device, enum AxisID axis, double rate, struct AxisConfig* config) {
	//  Invert RA direction for southern hemisphere
	bool south = false;//MOUNT_GEOGRAPHIC_COORDINATES_LATITUDE_ITEM->number.value < 0;
	if (south && axis == kAxisRA)
		rate = -rate;

	//  Capture original rate
	config->slewRate = rate;

	//  Determine direction from rate
	config->direction = kAxisDirectionFwd;
	if  (rate < 0) {
		config->direction = kAxisDirectionRev;
		rate = -rate;
	}

	//  Determine whether turbo mode is required for the specified rate
	config->turbo = false;
	if (rate > 128 * SIDEREAL_RATE) {
		config->turbo = true;
		rate /= (axis == kAxisRA) ? PRIVATE_DATA->raHighSpeedFactor : PRIVATE_DATA->decHighSpeedFactor;
	}

	//  Compute rate code
	rate *= ((axis == kAxisRA) ? PRIVATE_DATA->raTotalSteps : PRIVATE_DATA->decTotalSteps);
	rate /= 3600.0 * 360.0;
	double freq = (axis == kAxisRA) ? PRIVATE_DATA->raTimerFreq : PRIVATE_DATA->decTimerFreq;
	rate = freq / rate;
	config->rateCode = lrint(rate);

	//  Mark as valid
	config->valid = true;
}

bool synscan_configure_axis_for_rate(indigo_device* device, enum AxisID axis, double rate) {
	//  Compute config required
	struct AxisConfig requiredConfig;
	synscan_axis_config_for_rate(device, axis, rate, &requiredConfig);

	//  Determine if simple rate change is sufficient (or full reconfiguration)
	bool reconfigure = synscan_should_reconfigure_axis(device, axis, &requiredConfig);

	//  Reference relevant axis config cache
	struct AxisConfig* cachedConfig = (axis == kAxisRA) ? &PRIVATE_DATA->raAxisConfig : &PRIVATE_DATA->decAxisConfig;

	//  Determine whether backlash compensation will be needed
	//requiredConfig.compensateBacklash = (requiredConfig.direction != cachedConfig->direction);

	//  Invalidate gearing in case of error
	cachedConfig->valid = false;

	//  Set the gearing and slew rate
	bool ok = true;
	if (reconfigure) {
		ok = ok && synscan_set_axis_gearing(device, axis, requiredConfig.direction, requiredConfig.turbo ? kAxisSpeedHigh : kAxisSpeedLow);
		if (!ok)
			return false;
	}
	ok = ok && synscan_set_axis_slew_rate(device, axis, requiredConfig.rateCode);
	if (!ok)
		return false;

	//  Validate the axis config cache
	*cachedConfig = requiredConfig;
	return true;
}

static bool synscan_update_axis_to_rate(indigo_device* device, enum AxisID axis, double rate, bool* result) {
	//  Compute config required
	struct AxisConfig requiredConfig;
	synscan_axis_config_for_rate(device, axis, rate, &requiredConfig);

	//  Determine if simple rate change is sufficient (or full reconfiguration)
	bool reconfigure = synscan_should_reconfigure_axis(device, axis, &requiredConfig);
	if (reconfigure) {
		*result = false;
		return true;
	}
	else {
		*result = true;
	}

	//  Reference relevant axis config cache
	struct AxisConfig* cachedConfig = (axis == kAxisRA) ? &PRIVATE_DATA->raAxisConfig : &PRIVATE_DATA->decAxisConfig;

	//  Invalidate gearing in case of error
	cachedConfig->valid = false;

	bool ok = synscan_set_axis_slew_rate(device, axis, requiredConfig.rateCode);
	if (!ok)
		return false;

	//  Validate the axis config cache
	*cachedConfig = requiredConfig;
	return true;
}

static bool mount_slew_axis_to_position(indigo_device* device, enum AxisID axis, double position) {
	//  Reference relevant axis config cache
	struct AxisConfig* cachedConfig = (axis == kAxisRA) ? &PRIVATE_DATA->raAxisConfig : &PRIVATE_DATA->decAxisConfig;

	//  Get axis position
	AxisPosition currentPosition;
	bool ok = synscan_axis_position(device, axis, &currentPosition);
	if (!ok)
		return false;

	//  Convert position argument to steps
	AxisPosition steps = (axis == kAxisRA) ? ha_position_to_steps(device, position) : dec_position_to_steps(device, position);

	//  Compute step delta
	AxisPosition delta = steps - currentPosition;

	//  Initiate axis slew
	if (delta != 0) {
		enum AxisDirectionID d = (delta < 0) ? kAxisDirectionRev : kAxisDirectionFwd;
		delta = labs(delta);

		//		//  Check for backlash
		//		if (cachedConfig->direction != d) {
		//			//  Extend delta by the anti-backlash amount
		//			delta += [self antibacklashStepsForAxis:axis];
		//			printf("Doing ANTI-BACKLASH slew extension\n");
		//		}

		AxisPosition slowdown = delta - 80000;
		if (slowdown < 0)
			slowdown = delta / 2;

		//  Invalidate axis config since we change to absolute slew mode
		cachedConfig->valid = false;

		//  Always keep the direction up to date for backlash detection
		cachedConfig->direction = d;

		//  Initiate the slew
		ok = ok && synscan_set_axis_gearing(device, axis, d, kAxisSpeedAbsSlew);
		ok = ok && synscan_set_axis_step_count(device, axis, delta);
		ok = ok && synscan_set_axis_slowdown(device, axis, slowdown);
		ok = ok && synscan_slew_axis(device, axis);
	}
	return ok;
}


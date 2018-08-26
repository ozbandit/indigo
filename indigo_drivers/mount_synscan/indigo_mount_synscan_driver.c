//
//  indigo_mount_synscan_driver.c
//  indigo_server
//
//  Created by David Hulse on 07/08/2018.
//  Copyright © 2018 CloudMakers, s. r. o. All rights reserved.
//

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include "indigo_driver.h"
#include "indigo_novas.h"
#include "indigo_mount_driver.h"
#include "indigo_mount_synscan_private.h"
#include "indigo_mount_synscan_protocol.h"
#include "indigo_mount_synscan_driver.h"


//  All rates are arcsecs/sec
const double SIDEREAL_RATE = (360.0 * 3600.0) / 86164.090530833;
const double LUNAR_RATE = 14.511415;
const double SOLAR_RATE = 15.0;


//  Home position (arbitrary constant) is defined as:
//      HA == 6 (vertical shaft)
//      -/+90 degrees (pointing at pole)
static const AxisPosition RA_HOME_POSITION = 0x800000;
static const AxisPosition DEC_HOME_POSITION = 0x800000;


#define GET_RELEASE(v)				(int)(((v) >> 16) & 0xFF)
#define GET_REVISION(v)				(int)(((v) >> 8) & 0xFF)
#define GET_PATCH(v)          (int)((v) & 0xFF)


bool synscan_configure(indigo_device* device) {
	//  Start with an assumption about each axis
	PRIVATE_DATA->raAxisMode = PRIVATE_DATA->decAxisMode = kAxisModeIdle;

	//  Get firmware version
	long version;
	if (!synscan_firmware_version(device, &version)) {
		printf("ERROR GETTING FIRMWARE\n");
		return false;
	}
	if (device->master_device == device)
		snprintf(MOUNT_INFO_FIRMWARE_ITEM->text.value, INDIGO_VALUE_SIZE, "%2d.%02d.%02d", GET_RELEASE(version), GET_REVISION(version), GET_PATCH(version));

	printf("MOUNT CODE %02lX\n", version & 0xFF);

	//  Query motor status
	long raMotorStatus = 0;
	long decMotorStatus = 0;
	if (!synscan_motor_status(device, kAxisRA, &raMotorStatus)) {
		printf("ERROR GETTING RA STATUS\n");
		return false;
	}
	if (!synscan_motor_status(device, kAxisDEC, &decMotorStatus)) {
		printf("ERROR GETTING DEC STATUS\n");
		return false;
	}

	//  If the motors are moving, can we read back the config? Do we need to stop them?
	if ((raMotorStatus & kStatusActiveMask) != 0) {
		printf("RA MOTOR IS ACTIVE DURING CONFIGURATION!\n");
		synscan_stop_axis(device, kAxisRA);
	}
	if ((decMotorStatus & kStatusActiveMask) != 0) {
		printf("DEC MOTOR IS ACTIVE DURING CONFIGURATION!\n");
		synscan_stop_axis(device, kAxisDEC);
	}

	//  Read all the mount data
	//  Query axis 360 degree rotation steps
	if (!synscan_total_axis_steps(device, kAxisRA, &PRIVATE_DATA->raTotalSteps))
		return false;
	if (!synscan_total_axis_steps(device, kAxisDEC, &PRIVATE_DATA->decTotalSteps))
		return false;

	//  Query axis worm rotation steps
	if (!synscan_worm_rotation_steps(device, kAxisRA, &PRIVATE_DATA->raWormSteps))
		return false;
	if (!synscan_worm_rotation_steps(device, kAxisDEC, &PRIVATE_DATA->decWormSteps))
		return false;

	//  Query axis step timer frequency
	if (!synscan_step_timer_frequency(device, kAxisRA, &PRIVATE_DATA->raTimerFreq))
		return false;
	if (!synscan_step_timer_frequency(device, kAxisDEC, &PRIVATE_DATA->decTimerFreq))
		return false;

	//  Query axis high speed ratio
	if (!synscan_high_speed_ratio(device, kAxisRA, &PRIVATE_DATA->raHighSpeedFactor))
		return false;
	if (!synscan_high_speed_ratio(device, kAxisDEC, &PRIVATE_DATA->decHighSpeedFactor))
		return false;

//		PRIVATE_DATA->raTotalSteps = PRIVATE_DATA->decTotalSteps = 9024000;
//		PRIVATE_DATA->raWormSteps = PRIVATE_DATA->decWormSteps = 50133;
//		PRIVATE_DATA->raTimerFreq = PRIVATE_DATA->decTimerFreq = 64935;
//		PRIVATE_DATA->raHighSpeedFactor = PRIVATE_DATA->decHighSpeedFactor = 16;

	//  Check if mount understands polarscope LED brightness
	unsigned char brightness = 0;
	if (synscan_set_polarscope_brightness(device, brightness)) {
		MOUNT_POLARSCOPE_PROPERTY->hidden = false;
		MOUNT_POLARSCOPE_BRIGHTNESS_ITEM->number.value = 255;
	}

	//  Determine ZERO positions
	PRIVATE_DATA->raZeroPos = RA_HOME_POSITION - (PRIVATE_DATA->raTotalSteps / 4);     //  HA == 0 (horizontal shaft, rotated clockwise)
	PRIVATE_DATA->decZeroPos = DEC_HOME_POSITION - (PRIVATE_DATA->decTotalSteps / 4);  //  DEC has ZERO both east and west sides. This is the EAST zero.

	//  We do not redefine the ZERO positions for SOUTHERN HEMISPHERE because the mount does not redefine the direction and only understands
	//  step counts that increase going in an anti-clockwise direction. Therefore, we take account of southern hemisphere changes in the
	//  slewing routines.

	//  Dump out mount data
	printf("\n");
	printf("Total Steps:  RA == %10lu   DEC == %10lu\n", PRIVATE_DATA->raTotalSteps, PRIVATE_DATA->decTotalSteps);
	printf(" Worm Steps:  RA == %10lu   DEC == %10lu\n", PRIVATE_DATA->raWormSteps, PRIVATE_DATA->decWormSteps);
	printf(" Timer Freq:  RA == %10lu   DEC == %10lu\n", PRIVATE_DATA->raTimerFreq, PRIVATE_DATA->decTimerFreq);
	printf("  HS Factor:  RA == %10lu   DEC == %10lu\n", PRIVATE_DATA->raHighSpeedFactor, PRIVATE_DATA->decHighSpeedFactor);
	printf("   Home Pos:  RA == %10lu   DEC == %10lu\n", RA_HOME_POSITION, DEC_HOME_POSITION);
	printf("   Zero Pos:  RA == %10lu   DEC == %10lu\n", PRIVATE_DATA->raZeroPos, PRIVATE_DATA->decZeroPos);
	printf(" Polarscope:  %s\n", PRIVATE_DATA->canSetPolarscopeBrightness ? "YES" : "NO");
	printf("\n");

	//  Determine mount vendor and model

	//  Initialize motors if necessary
	if ((raMotorStatus & kStatusInitMask) == 0) {
		if (!synscan_init_axis(device, kAxisRA))
			return false;
		if (!synscan_motor_status(device, kAxisRA, &raMotorStatus))
			return false;
		if ((raMotorStatus & kStatusInitMask) == 0) {
			//  This is a fatal error - can't get the motor to initialize
			printf("RA MOTOR FAILURE: WILL NOT INITIALIZE!\n");
			return false;
		}
		if (!synscan_init_axis_position(device, kAxisRA, RA_HOME_POSITION))
			return false;
		printf("RA MOTOR INITIALIZED!\n");
	}
	else {
		//  if mount was previously configured - assume that configuration is still valid

		//  else this is also a fatal error - we don't know why the motors are not needing INIT
		//  maybe the software restarted??
		//  we could receover by configuring, but the user should be told that the assumption is the mount must be homed

		//  actually if the assumption is that the software restarted AND EQMac previously configured it, then we can recover
		//  by just reading the parameters and trusting the current position and computing where the home position should be

		printf("RA MOTOR OK %06lX\n", raMotorStatus);
	}
	if ((decMotorStatus & kStatusInitMask) == 0) {
		if (!synscan_init_axis(device, kAxisDEC))
			return false;
		if (!synscan_motor_status(device, kAxisDEC, &decMotorStatus))
			return false;
		if ((decMotorStatus & kStatusInitMask) == 0) {
			//  This is a fatal error - can't get the motor to initialize
			printf("DEC MOTOR FAILURE: WILL NOT INITIALIZE!\n");
			return false;
		}
		if (!synscan_init_axis_position(device, kAxisDEC, DEC_HOME_POSITION))
			return false;
		printf("DEC MOTOR INITIALIZED!\n");
	}
	else {
		//  if mount was previously configured - assume that configuration is still valid

		//  else this is also a fatal error - we don't know why the motors are not needing INIT
		//  maybe the software restarted??
		//  we could receover by configuring, but the user should be told that the assumption is the mount must be homed
		printf("DEC MOTOR OK %06lX\n", decMotorStatus);
	}

	//  Configure the mount modes
	PRIVATE_DATA->raAxisMode = PRIVATE_DATA->raDesiredAxisMode = kAxisModeIdle;
	PRIVATE_DATA->decAxisMode = PRIVATE_DATA->decDesiredAxisMode = kAxisModeIdle;
	PRIVATE_DATA->globalMode = kGlobalModeIdle;
	//PRIVATE_DATA->trackingMode = kTrackingModeOff;

	//  Invalidate axis configurations
	PRIVATE_DATA->raAxisConfig.valid = false;
	PRIVATE_DATA->decAxisConfig.valid = false;

	//  Read the current position
	synscan_get_coords(device);
	printf("   Position:  RA == %g   DEC == %g\n", PRIVATE_DATA->raPosition, PRIVATE_DATA->decPosition);

	//  Consider the mount configured once we reach here
	return true;
}

static double synscan_tracking_rate(indigo_device* device) {
	if (MOUNT_TRACK_RATE_SIDEREAL_ITEM->sw.value)
		return SIDEREAL_RATE;
	else if (MOUNT_TRACK_RATE_SOLAR_ITEM->sw.value)
		return SOLAR_RATE;
	else if (MOUNT_TRACK_RATE_LUNAR_ITEM->sw.value)
		return LUNAR_RATE;
	else
		return 0.0;
}






void synscan_get_coords(indigo_device *device) {
	char response[128];
	long haPos, decPos;
	//  Get the DEC first since we want the HA position to be changing as little as possible till
	//  we combine it with LST to get RA
	if (synscan_axis_position(device, kAxisDEC, &decPos))
		PRIVATE_DATA->decPosition = dec_steps_to_position(device, decPos);
	if (synscan_axis_position(device, kAxisRA, &haPos))
		PRIVATE_DATA->raPosition = ha_steps_to_position(device, haPos);
}

void synscan_stop_and_resume_tracking_for_axis(indigo_device* device, enum AxisID axis) {
	if (axis == kAxisRA) {
		if (MOUNT_TRACKING_ON_ITEM->sw.value) {
			PRIVATE_DATA->raDesiredRate = synscan_tracking_rate(device);
			PRIVATE_DATA->raDesiredAxisMode = kAxisModeTracking;
		}
		else {
			PRIVATE_DATA->raDesiredAxisMode = kAxisModeIdle;
		}
	}
	else {
		PRIVATE_DATA->decDesiredAxisMode = kAxisModeIdle;
	}
}

void synscan_slew_axis_at_rate(indigo_device* device, enum AxisID axis, double rate) {
	if (axis == kAxisRA) {
		PRIVATE_DATA->raDesiredRate = rate;
		PRIVATE_DATA->raDesiredAxisMode = kAxisModeManualSlewing;
	}
	else {
		PRIVATE_DATA->decDesiredRate = rate;
		PRIVATE_DATA->decDesiredAxisMode = kAxisModeManualSlewing;
	}
}


void synscan_start_tracking_mode(indigo_device* device, enum TrackingMode mode) {
	if (mode == kTrackingModeOff) {
		PRIVATE_DATA->raDesiredAxisMode = kAxisModeIdle;
	}
	else {
		PRIVATE_DATA->raDesiredRate = synscan_tracking_rate(device);
		PRIVATE_DATA->raDesiredAxisMode = kAxisModeTracking;
	}
}

static int synscan_select_best_encoder_point(indigo_device* device, double haPos[], double decPos[]) {
	//  Check if we're allowed to do CW-UP slews
//	NSUserDefaults* prefs = [NSUserDefaults standardUserDefaults];
//	NSInteger ra_slew_priority = [prefs integerForKey:@"ra_slew_priority"];
//	bool limitsEnabled = [[prefs objectForKey:@"limits_enabled"] boolValue];

	//  Return the SAFE point if CW-UP slews are disabled or limits are disabled in general
	//if (ra_slew_priority == 0 || !limitsEnabled)
	//	return &eps[0];
	return 0;

#if 0
	//  Load limits from prefs
	NSNumber* leftPosition = [prefs objectForKey:@"left_ra_limit"];
	NSNumber* rightPosition = [prefs objectForKey:@"right_ra_limit"];

	//  Cache RA limit positions
	double leftLimitPosition = 0.5;
	double rightLimitPosition = 0.0;
	if (leftPosition)
		leftLimitPosition = [leftPosition doubleValue];
		if (rightPosition)
			rightLimitPosition = [rightPosition doubleValue];

			//  Rebase right limit
			if (rightLimitPosition > 0.75)
				rightLimitPosition -= 1.0;

				//  Normalise the cwup_h for comparison
				double cwup_h = eps[1].ha;
				if (cwup_h > 0.75)
					cwup_h -= 1.0;

					//  Check if the CW-UP point is within limits
					if (cwup_h > leftLimitPosition || cwup_h < rightLimitPosition) {
						printf("CWUP HA IS OUTSIDE LIMITS - CANNOT SLEW CW UP\n");
						return &eps[0];
					}

	//  Check if the CW-UP point is desirable over the normal point
	bool south = [PreferencesController latitude] < 0.0;
	if (!south) {
		//  Desirable if time to flip (reach 0.5) is better
		if (0.5 - cwup_h > 0.5 - eps[0].ha) {
			printf("CWUP SLEW IS DESIRABLE!\n");
			return &eps[1];
		}
	}
	else {
		//  Desirable if time to flip (reach 0.0) is better
		if (cwup_h > eps[0].ha) {
			printf("CWUP SLEW IS DESIRABLE!\n");
			return &eps[1];
		}
	}

	//  Return the safe position
	return &eps[0];
#endif
}

enum TrackingMode mount_tracking_mode(indigo_device* device) {
	//  Get the desired tracking rate
	enum TrackingMode trackingMode = kTrackingModeOff;
	if (MOUNT_TRACK_RATE_SIDEREAL_ITEM->sw.value)
		trackingMode = kTrackingModeSidereal;
	else if (MOUNT_TRACK_RATE_SOLAR_ITEM->sw.value)
		trackingMode = kTrackingModeSolar;
	else if (MOUNT_TRACK_RATE_LUNAR_ITEM->sw.value)
		trackingMode = kTrackingModeLunar;
	return trackingMode;
}

void slew_timer_callback(indigo_device *device) {
//	if (PRIVATE_DATA->globalMode != kGlobalModeSlewing) {
//		indigo_reschedule_timer(device, 0.25, &PRIVATE_DATA->slew_timer);
//		return;
//	}

	switch (PRIVATE_DATA->slew_state) {
		case SLEW_PHASE0:
			//  Perform approximate slew on both axes
			{
				//  Compute initial target positions
				double ra = MOUNT_EQUATORIAL_COORDINATES_RA_ITEM->number.target * M_PI / 12.0;
				double dec = MOUNT_EQUATORIAL_COORDINATES_DEC_ITEM->number.target * M_PI / 180.0;
				double lng = MOUNT_GEOGRAPHIC_COORDINATES_LONGITUDE_ITEM->number.value;
				double lst = indigo_lst(lng) * M_PI / 12.0;
				double ha = lst - ra;
				double haPos[2], decPos[2];
				coords_eq_to_encoder2(device, ha, dec, haPos, decPos);

				//  Select best encoder point based on limits
				int idx = synscan_select_best_encoder_point(device, haPos, decPos);
				printf("SLEW TO:  %g   /   %g     (HA %g / DEC %g)\n", haPos[idx], decPos[idx], ha, dec);

				//  Start the first slew
				//if (!synscan_slew_axis_to_position(device, kAxisRA, haPos[idx]) || !synscan_slew_axis_to_position(device, kAxisDEC, decPos[idx]) {
					//  Abort slew
					//  alert state the coordinates
				//}

				//PRIVATE_DATA->raTargetPosition = haPos[idx];
				//PRIVATE_DATA->decTargetPosition = decPos[idx];
				//PRIVATE_DATA->raDesiredAxisMode = kAxisModeSlewing;
				//PRIVATE_DATA->decDesiredAxisMode = kAxisModeSlewing;

				//  Wait for HA axis mode to be slewing and then idle - but need to be careful here
				//if (PRIVATE_DATA->raAxisMode != kAxisModeSlewIdle)
				//	break;
				//if (!mount_wait_for_axis_stopped(device, kAxisRA)) {
					//  Abort slew
					//  alaert state
				//}

				//  Move to next phase
				PRIVATE_DATA->slew_state = SLEW_PHASE1;
			}
			break;
		case SLEW_PHASE1:
			//  Wait for HA slew to complete
			{
				//    Compute precise HA slew for LST + 5 seconds
				double ra = MOUNT_EQUATORIAL_COORDINATES_RA_ITEM->number.target * M_PI / 12.0;
				double dec = MOUNT_EQUATORIAL_COORDINATES_DEC_ITEM->number.target * M_PI / 180.0;
				double lng = MOUNT_GEOGRAPHIC_COORDINATES_LONGITUDE_ITEM->number.value;
				PRIVATE_DATA->target_lst = indigo_lst(lng) + (5 / 3600.0);
				double ha = (PRIVATE_DATA->target_lst * M_PI / 12.0) - ra;
				double haPos[2], decPos[2];
				coords_eq_to_encoder2(device, ha, dec, haPos, decPos);

				//  Select best encoder point based on limits
				int idx = synscan_select_best_encoder_point(device, haPos, decPos);
				printf("SLEW TO:  %g   /   %g\n", haPos[idx], decPos[idx]);

				//  Set the desired axis mode to slewing and provide the target coordinate

				//  Start new HA slew
				PRIVATE_DATA->raTargetPosition = haPos[idx];
				PRIVATE_DATA->raDesiredAxisMode = kAxisModeSlewing;

				//	Move to next phase
				PRIVATE_DATA->slew_state = SLEW_PHASE2;
			}
			break;
		case SLEW_PHASE2:
			//  Wait for precise HA slew to complete
			{
				//  Wait for HA axis mode to be slewing and then idle - but need to be careful here
				//  Might need a new SlewIdle state so we dont get confused and see Idle when we are just stopping to start a slew
				if (PRIVATE_DATA->raAxisMode != kAxisModeSlewIdle)
					break;

				//	Move to next phase
				PRIVATE_DATA->slew_state = SLEW_PHASE3;
			}
			break;
		case SLEW_PHASE3:
			//  Wait for LST to match
			{
				//  Get current LST
				double lng = MOUNT_GEOGRAPHIC_COORDINATES_LONGITUDE_ITEM->number.value;
				double lst = indigo_lst(lng);

				//  Wait for LST to reach target LST
				if (lst < PRIVATE_DATA->target_lst)
					break;

				//  Set axis mode to tracking and provide correct rate
				//  Otherwise set desired mode to Idle

				//    Start tracking if we need to
				if (MOUNT_ON_COORDINATES_SET_TRACK_ITEM->sw.value) {
					synscan_start_tracking_mode(device, mount_tracking_mode(device));
				}
				else {
					PRIVATE_DATA->raDesiredAxisMode = kAxisModeIdle;
				}

				//	Move to next phase
				PRIVATE_DATA->slew_state = SLEW_PHASE4;
			}
			break;
		case SLEW_PHASE4:
			//  Wait for DEC slew to complete
			{
				//  Wait for DEC axis mode to be slewing and then idle - but need to be careful here
				//  Might need a new SlewIdle state so we dont get confused and see Idle when we are just stopping to start a slew
				if (PRIVATE_DATA->decAxisMode != kAxisModeSlewIdle)
					break;

				//  Set desired mode to Idle
				PRIVATE_DATA->decDesiredAxisMode = kAxisModeIdle;

				//	Move to next phase
				PRIVATE_DATA->slew_state = SLEW_NONE;
				PRIVATE_DATA->globalMode = kGlobalModeIdle;
			}
			break;
		case SLEW_NONE:
		default:
			//  Slew is not happening
			break;
	}

	if (PRIVATE_DATA->slew_state != SLEW_NONE) {
		indigo_reschedule_timer(device, 0.25, &PRIVATE_DATA->slew_timer);
	} else {
		PRIVATE_DATA->slew_timer = NULL;
		//MOUNT_EQUATORIAL_COORDINATES_PROPERTY->state = INDIGO_OK_STATE;
		indigo_update_property(device, MOUNT_EQUATORIAL_COORDINATES_PROPERTY, NULL);
	}
}


bool synscan_open(indigo_device *device) {
	char *name = DEVICE_PORT_ITEM->text.value;
	if (strncmp(name, "synscan://", 10)) {
		PRIVATE_DATA->handle = indigo_open_serial(name);
	} else {
		char *host = name + 10;
		char *colon = strchr(host, ':');
		if (colon == NULL) {
			PRIVATE_DATA->handle = indigo_open_tcp(host, 4030);
		} else {
			char host_name[INDIGO_NAME_SIZE];
			strncpy(host_name, host, colon - host);
			int port = atoi(colon + 1);
			PRIVATE_DATA->handle = indigo_open_tcp(host_name, port);
		}
	}
	if (PRIVATE_DATA->handle >= 0) {
		INDIGO_DRIVER_LOG(DRIVER_NAME, "opened %s", name);
		return true;
	} else {
		INDIGO_DRIVER_ERROR(DRIVER_NAME, "failed to open %s", name);
		return false;
	}
}

void synscan_close(indigo_device *device) {
	if (PRIVATE_DATA->handle > 0) {
		close(PRIVATE_DATA->handle);
		PRIVATE_DATA->handle = 0;
		INDIGO_DRIVER_LOG(DRIVER_NAME, "disconnected from %s", DEVICE_PORT_ITEM->text.value);
	}
}

void mount_handle_coordinates(indigo_device *device) {
	if (PRIVATE_DATA->globalMode != kGlobalModeIdle) {
		MOUNT_EQUATORIAL_COORDINATES_PROPERTY->state = INDIGO_ALERT_STATE;
	}
	else {
		MOUNT_EQUATORIAL_COORDINATES_PROPERTY->state = INDIGO_BUSY_STATE;

		//  GOTO requested
		PRIVATE_DATA->globalMode = kGlobalModeSlewing;
		PRIVATE_DATA->slew_state = SLEW_PHASE0;
		PRIVATE_DATA->slew_timer = indigo_set_timer(device, 0, &slew_timer_callback);
	}
	indigo_update_coordinates(device, NULL);
}

void synscan_tracking_timer(indigo_device* device) {
	const char* message = NULL;
	if (MOUNT_TRACKING_ON_ITEM->sw.value) {
		//  Get the desired tracking rate
		double rate = synscan_tracking_rate(device);

		//  Start tracking
		if (!synscan_configure_axis_for_rate(device, kAxisRA, rate) || !synscan_slew_axis(device, kAxisRA)) {
			MOUNT_TRACKING_PROPERTY->state = INDIGO_ALERT_STATE;
		}
		message = "tracking started";
	}
	else if (MOUNT_TRACKING_OFF_ITEM->sw.value) {
		if (!synscan_stop_axis(device, kAxisRA)) {
			MOUNT_TRACKING_PROPERTY->state = INDIGO_ALERT_STATE;
		}
		message = "tracking stopped";
	}

	MOUNT_TRACKING_PROPERTY->state = INDIGO_OK_STATE;
	indigo_update_property(device, MOUNT_TRACKING_PROPERTY, message);
}

void mount_handle_tracking(indigo_device *device) {
	//  Ignore property change if busy
	if (MOUNT_TRACKING_PROPERTY->state == INDIGO_BUSY_STATE)
		return;

	//  Ignore tracking request unless idle
	if (PRIVATE_DATA->globalMode != kGlobalModeIdle) {
		MOUNT_TRACKING_PROPERTY->state = INDIGO_ALERT_STATE;
		indigo_update_property(device, MOUNT_TRACKING_PROPERTY, "Tracking change refused - mount busy!");
		return;
	}

	//  Start a timer to configure tracking
	MOUNT_TRACKING_PROPERTY->state = INDIGO_BUSY_STATE;
	indigo_update_property(device, MOUNT_TRACKING_PROPERTY, NULL);
	indigo_set_timer(device, 0, &synscan_tracking_timer);
}

static int mount_slew_rate(indigo_device* device) {
	if (MOUNT_SLEW_RATE_GUIDE_ITEM->sw.value)
		return 1;
	else if (MOUNT_SLEW_RATE_CENTERING_ITEM->sw.value)
		return 4;
	else if (MOUNT_SLEW_RATE_FIND_ITEM->sw.value)
		return 6;
	else if (MOUNT_SLEW_RATE_MAX_ITEM->sw.value)
		return 9;
	else
		return 1;
}

static void mount_handle_slew_rate(indigo_device *device) {
	int slewRate = mount_slew_rate(device);
	MOUNT_SLEW_RATE_PROPERTY->state = INDIGO_OK_STATE;
	indigo_update_property(device, MOUNT_SLEW_RATE_PROPERTY, "slew speed = %d", slewRate);
}

static void mount_handle_motion_dec(indigo_device *device) {
#if 0
	char message[INDIGO_VALUE_SIZE];
	double axisRate = decRates[mount_slew_rate(device)] * SIDEREAL_RATE;
	if(MOUNT_MOTION_NORTH_ITEM->sw.value) {
		synscan_slew_axis_at_rate(device, kAxisDEC, -axisRate);
		strncpy(message,"Moving North...",sizeof(message));
		MOUNT_MOTION_DEC_PROPERTY->state = INDIGO_BUSY_STATE;
	} else if (MOUNT_MOTION_SOUTH_ITEM->sw.value) {
		synscan_slew_axis_at_rate(device, kAxisDEC, axisRate);
		strncpy(message,"Moving South...",sizeof(message));
		MOUNT_MOTION_DEC_PROPERTY->state = INDIGO_BUSY_STATE;
	} else {
		synscan_stop_and_resume_tracking_for_axis(device, kAxisDEC);
		strncpy(message,"Stopped moving.",sizeof(message));
		MOUNT_MOTION_DEC_PROPERTY->state = INDIGO_OK_STATE;
	}
	indigo_update_property(device, MOUNT_MOTION_DEC_PROPERTY, message);
#endif
}

static void mount_handle_motion_ra(indigo_device *device) {
#if 0
	char message[INDIGO_VALUE_SIZE];
	double axisRate = raRates[mount_slew_rate(device)] * SIDEREAL_RATE;
	if (MOUNT_MOTION_WEST_ITEM->sw.value) {
		synscan_slew_axis_at_rate(device, kAxisRA, axisRate);
		strncpy(message,"Moving West...",sizeof(message));
		MOUNT_MOTION_RA_PROPERTY->state = INDIGO_BUSY_STATE;
	} else if (MOUNT_MOTION_EAST_ITEM->sw.value) {
		synscan_slew_axis_at_rate(device, kAxisRA, -axisRate);
		strncpy(message,"Moving East...",sizeof(message));
		MOUNT_MOTION_RA_PROPERTY->state = INDIGO_BUSY_STATE;
	}
	else {
		synscan_stop_and_resume_tracking_for_axis(device, kAxisRA);
		strncpy(message,"Stopped moving.",sizeof(message));
		MOUNT_MOTION_RA_PROPERTY->state = INDIGO_OK_STATE;
	}
	indigo_update_property(device, MOUNT_MOTION_RA_PROPERTY, NULL);
#endif
}

static void mount_handle_st4_guiding_rate(indigo_device *device) {
#if 0
	int offset = 1;                                             /* for Ceslestron 0 is 1% and 99 is 100% */
	if (PRIVATE_DATA->vendor_id == VNDR_SKYWATCHER) offset = 0; /* there is no offset for Sky-Watcher */

	/* reset only if input value is changed - better begaviour for Sky-Watcher as there are no separate RA and DEC rates */
	if ((int)(MOUNT_GUIDE_RATE_RA_ITEM->number.value) != PRIVATE_DATA->st4_ra_rate) {
		if (!synscan_set_st4_guide_rate(device, (int)(MOUNT_GUIDE_RATE_RA_ITEM->number.value)-1)) {
			INDIGO_DRIVER_ERROR(DRIVER_NAME, "tc_set_autoguide_rate(%d) = %d", dev_id, res);
			MOUNT_GUIDE_RATE_PROPERTY->state = INDIGO_ALERT_STATE;
		} else {
			//PRIVATE_DATA->st4_ra_rate = (int)(MOUNT_GUIDE_RATE_RA_ITEM->number.value);
		}
	}

	/* reset only if input value is changed - better begaviour for Sky-Watcher as there are no separate RA and DEC rates */
	if ((int)(MOUNT_GUIDE_RATE_DEC_ITEM->number.value) != PRIVATE_DATA->st4_dec_rate) {
		if (!synscan_set_st4_guide_rate(device, (int)(MOUNT_GUIDE_RATE_DEC_ITEM->number.value)-1)) {
			INDIGO_DRIVER_ERROR(DRIVER_NAME, "tc_set_autoguide_rate(%d) = %d", dev_id, res);
			MOUNT_GUIDE_RATE_PROPERTY->state = INDIGO_ALERT_STATE;
		} else {
			//PRIVATE_DATA->st4_dec_rate = (int)(MOUNT_GUIDE_RATE_DEC_ITEM->number.value);
		}
	}
#endif
	/* read set values as Sky-Watcher rounds to 12, 25 ,50, 75 and 100 % */
	//	int st4_ra_rate = tc_get_autoguide_rate(dev_id, TC_AXIS_RA);
	//	if (st4_ra_rate < 0) {
	//		INDIGO_DRIVER_ERROR(DRIVER_NAME, "tc_get_autoguide_rate(%d) = %d", dev_id, st4_ra_rate);
	//	} else {
	//		MOUNT_GUIDE_RATE_RA_ITEM->number.value = st4_ra_rate + offset;
	//	}
	//
	//	int st4_dec_rate = tc_get_autoguide_rate(dev_id, TC_AXIS_DE);
	//	if (st4_dec_rate < 0) {
	//		INDIGO_DRIVER_ERROR(DRIVER_NAME, "tc_get_autoguide_rate(%d) = %d", dev_id, st4_dec_rate);
	//	} else {
	//		MOUNT_GUIDE_RATE_DEC_ITEM->number.value = st4_dec_rate + offset;
	//	}

	MOUNT_GUIDE_RATE_PROPERTY->state = INDIGO_OK_STATE;
	indigo_update_property(device, MOUNT_GUIDE_RATE_PROPERTY, NULL);
}

static bool mount_cancel_slew(indigo_device *device) {
	if (PRIVATE_DATA->globalMode == kGlobalModeParking || PRIVATE_DATA->globalMode == kGlobalModeSlewing) {
		PRIVATE_DATA->globalMode = kGlobalModeIdle;
	}
	PRIVATE_DATA->raDesiredAxisMode = kAxisModeIdle;
	PRIVATE_DATA->decDesiredAxisMode = kAxisModeIdle;

	//	MOUNT_MOTION_NORTH_ITEM->sw.value = false;
	//	MOUNT_MOTION_SOUTH_ITEM->sw.value = false;
	//	MOUNT_MOTION_DEC_PROPERTY->state = INDIGO_OK_STATE;
	//	indigo_update_property(device, MOUNT_MOTION_DEC_PROPERTY, NULL);
	//
	//	MOUNT_MOTION_WEST_ITEM->sw.value = false;
	//	MOUNT_MOTION_EAST_ITEM->sw.value = false;
	//	MOUNT_MOTION_RA_PROPERTY->state = INDIGO_OK_STATE;
	//	indigo_update_property(device, MOUNT_MOTION_RA_PROPERTY, NULL);
	//
	//	MOUNT_EQUATORIAL_COORDINATES_RA_ITEM->number.target = MOUNT_EQUATORIAL_COORDINATES_RA_ITEM->number.value;
	//	MOUNT_EQUATORIAL_COORDINATES_DEC_ITEM->number.target = MOUNT_EQUATORIAL_COORDINATES_DEC_ITEM->number.value;
	//	MOUNT_EQUATORIAL_COORDINATES_PROPERTY->state = INDIGO_OK_STATE;
	//	indigo_update_coordinates(device, NULL);

	MOUNT_ABORT_MOTION_ITEM->sw.value = false;
	MOUNT_ABORT_MOTION_PROPERTY->state = INDIGO_OK_STATE;
	indigo_update_property(device, MOUNT_ABORT_MOTION_PROPERTY, "Aborted.");
	return true;
}

static void park_timer_callback(indigo_device *device) {
	if (PRIVATE_DATA->globalMode == kGlobalModeParking) {
		switch (PRIVATE_DATA->park_state) {
			case PARK_NONE:
				break;
			case PARK_PHASE0:
			{
				//  Compute the axis positions for parking
				double ha = MOUNT_PARK_POSITION_HA_ITEM->number.value * M_PI / 12.0;
				double dec = MOUNT_PARK_POSITION_DEC_ITEM->number.value * M_PI / 180.0;
				double haPos[2], decPos[2];
				coords_eq_to_encoder2(device, ha, dec, haPos, decPos);

				//  Start the axes moving
				PRIVATE_DATA->raTargetPosition = haPos[0];
				PRIVATE_DATA->decTargetPosition = decPos[0];
				PRIVATE_DATA->raDesiredAxisMode = kAxisModeSlewing;
				PRIVATE_DATA->decDesiredAxisMode = kAxisModeSlewing;

				//  Move to next state
				PRIVATE_DATA->park_state = PARK_PHASE1;
			}
				break;

			case PARK_PHASE1:
			{
				//  Check for HA parked
				if (PRIVATE_DATA->raAxisMode != kAxisModeSlewIdle)
					break;

				//  Set desired mode to Idle
				PRIVATE_DATA->raDesiredAxisMode = kAxisModeIdle;

				//  Move to next state
				PRIVATE_DATA->park_state = PARK_PHASE2;
			}
				break;

			case PARK_PHASE2:
			{
				//  Check for DEC parked
				if (PRIVATE_DATA->decAxisMode != kAxisModeSlewIdle)
					break;

				//  Set desired mode to Idle
				PRIVATE_DATA->decDesiredAxisMode = kAxisModeIdle;

				//  Update state
				PRIVATE_DATA->park_state = PARK_NONE;
				PRIVATE_DATA->globalMode = kGlobalModeParked;
				PRIVATE_DATA->parked = true;
				MOUNT_PARK_PARKED_ITEM->sw.value = true;
				MOUNT_PARK_PROPERTY->state = INDIGO_OK_STATE;
			}
				break;
		}
	}
	if (PRIVATE_DATA->globalMode == kGlobalModeParking) {
//		indigo_reschedule_timer(device, REFRESH_SECONDS, &PRIVATE_DATA->park_timer);
	}
	else {
		PRIVATE_DATA->park_timer = NULL;
		indigo_update_property(device, MOUNT_PARK_PROPERTY, "Mount Parked.");
	}
}

static void position_timer_callback(indigo_device *device) {
	if (PRIVATE_DATA->handle > 0 && !PRIVATE_DATA->parked) {
		//  Longitude needed for LST
		double lng = MOUNT_GEOGRAPHIC_COORDINATES_LONGITUDE_ITEM->number.value;

		//  Get the raw coords
		synscan_get_coords(device);

		//  Get the LST as quickly as we can after the HA
		double lst = indigo_lst(lng) * M_PI / 12.0;

		//  Convert Encoder position to EQ
		coords_encoder_to_eq(device, PRIVATE_DATA->raPosition, PRIVATE_DATA->decPosition, &PRIVATE_DATA->currentHA, &PRIVATE_DATA->currentDec);

		//  Add in LST to get RA
		PRIVATE_DATA->currentRA = lst - PRIVATE_DATA->currentHA;
		PRIVATE_DATA->currentRA *= 12.0 / M_PI;
		PRIVATE_DATA->currentDec *= 180.0 / M_PI;
		if (PRIVATE_DATA->currentRA < 0)
			PRIVATE_DATA->currentRA += 24.0;
		if (PRIVATE_DATA->currentRA >= 24.0)
			PRIVATE_DATA->currentRA -= 24.0;

		printf("LST: %g\nHA: %g\n", lst * 12.0 / M_PI, PRIVATE_DATA->currentHA);

		//		double diffRA = fabs(MOUNT_EQUATORIAL_COORDINATES_RA_ITEM->number.target - PRIVATE_DATA->currentRA);
		//		double diffDec = fabs(MOUNT_EQUATORIAL_COORDINATES_DEC_ITEM->number.target - PRIVATE_DATA->currentDec);
		if (PRIVATE_DATA->slew_state == SLEW_NONE)
			MOUNT_EQUATORIAL_COORDINATES_PROPERTY->state = INDIGO_OK_STATE;
		else
			MOUNT_EQUATORIAL_COORDINATES_PROPERTY->state = INDIGO_BUSY_STATE;


		MOUNT_EQUATORIAL_COORDINATES_RA_ITEM->number.value = PRIVATE_DATA->currentRA;
		MOUNT_EQUATORIAL_COORDINATES_DEC_ITEM->number.value = PRIVATE_DATA->currentDec;
		indigo_update_coordinates(device, NULL);

		//meade_get_utc(device);
		//indigo_update_property(device, MOUNT_UTC_TIME_PROPERTY, NULL);
	}
	indigo_reschedule_timer(device, 0.5, &PRIVATE_DATA->position_timer);
}

void synscan_connect_timer_callback(indigo_device* device) {
	//  Open and configure the mount
	bool result = synscan_open(device->master_device) && synscan_configure(device);
	if (result) {
		PRIVATE_DATA->connections++;
		CONNECTION_PROPERTY->state = INDIGO_OK_STATE;
		indigo_update_property(device, CONNECTION_PROPERTY, "connected to mount!");

		//  Here I need to invoke the code in indigo_mount_driver.c on lines 270-334 to define the properties that should now be present.

	}
	else {
		CONNECTION_PROPERTY->state = INDIGO_ALERT_STATE;
		indigo_set_switch(CONNECTION_PROPERTY, CONNECTION_DISCONNECTED_ITEM, true);
		indigo_update_property(device, CONNECTION_PROPERTY, "Failed to connect to mount");
	}
}

void synscan_connect(indigo_device* device) {
	//  No need to store the timer as this is a one-shot
	indigo_set_timer(device, 0, &synscan_connect_timer_callback);
}

void synscan_disconnect(indigo_device* device) {
//	indigo_cancel_timer(device, &PRIVATE_DATA->ha_axis_timer);
//	indigo_cancel_timer(device, &PRIVATE_DATA->dec_axis_timer);
//	indigo_cancel_timer(device, &PRIVATE_DATA->position_timer);
//	indigo_cancel_timer(device, &PRIVATE_DATA->slew_timer);
//	PRIVATE_DATA->ha_axis_timer = NULL;
//	PRIVATE_DATA->dec_axis_timer = NULL;
//	PRIVATE_DATA->position_timer = NULL;
//	PRIVATE_DATA->slew_timer = NULL;
//	PRIVATE_DATA->device_count--;
//	if (PRIVATE_DATA->device_count <= 0) {
//		synscan_close(device);
//	}
//	CONNECTION_PROPERTY->state = INDIGO_OK_STATE;

	synscan_close(device->master_device);
}



void synscan_mount_connect(indigo_device* device) {
	//  Ignore if we are already processing a connection change
	if (CONNECTION_PROPERTY->state == INDIGO_BUSY_STATE)
		return;

	//  Handle connect/disconnect commands
	if (CONNECTION_CONNECTED_ITEM->sw.value) {
		//  CONNECT to the mount
		if (PRIVATE_DATA->connections == 0) {
			CONNECTION_PROPERTY->state = INDIGO_BUSY_STATE;
			synscan_connect(device);
			return;
		}
		else
			PRIVATE_DATA->connections++;
	}
	else if (CONNECTION_DISCONNECTED_ITEM->sw.value) {
		//  DISCONNECT from mount
		if (PRIVATE_DATA->connections > 0) {
			PRIVATE_DATA->connections--;
			if (PRIVATE_DATA->connections == 0)
				synscan_disconnect(device);
		}
	}
	CONNECTION_PROPERTY->state = INDIGO_OK_STATE;
}


//
//	bool result = true;
//	if (PRIVATE_DATA->device_count == 0) {
//		PRIVATE_DATA->device_count++;
//		CONNECTION_PROPERTY->state = INDIGO_BUSY_STATE;
//		indigo_update_property(device, CONNECTION_PROPERTY, NULL);//"Connecting to mount...");
//		result = synscan_open(device);
//	}
//	if (result)
//		result = synscan_configure(device);
//	if (result) {
//		strcpy(MOUNT_INFO_VENDOR_ITEM->text.value, "Sky-Watcher");
//		strcpy(MOUNT_INFO_MODEL_ITEM->text.value, "SynScan");
//
//		//  Start timers
//		PRIVATE_DATA->ha_axis_timer = indigo_set_timer(device, 0, ha_axis_timer_callback);
//		PRIVATE_DATA->dec_axis_timer = indigo_set_timer(device, 0, dec_axis_timer_callback);
//		PRIVATE_DATA->position_timer = indigo_set_timer(device, 0, position_timer_callback);
//		PRIVATE_DATA->slew_timer = indigo_set_timer(device, 0, slew_timer_callback);
//		CONNECTION_PROPERTY->state = INDIGO_OK_STATE;
//	}
//	else {
//		PRIVATE_DATA->device_count--;
//		CONNECTION_PROPERTY->state = INDIGO_ALERT_STATE;
//		indigo_set_switch(CONNECTION_PROPERTY, CONNECTION_DISCONNECTED_ITEM, true);
//	}
//}


void synscan_mount_park(indigo_device* device) {
#if 0
	if(PRIVATE_DATA->park_in_progress) {
		indigo_update_property(device, MOUNT_PARK_PROPERTY, WARN_PARKING_PROGRESS_MSG);
		return INDIGO_OK;
	}
#endif
//	indigo_property_copy_values(MOUNT_PARK_PROPERTY, property, false);
	if (MOUNT_PARK_PARKED_ITEM->sw.value) {
		if (PRIVATE_DATA->globalMode == kGlobalModeIdle) {
			MOUNT_PARK_PROPERTY->state = INDIGO_BUSY_STATE;
			indigo_update_property(device, MOUNT_PARK_PROPERTY, "Parking...");
			PRIVATE_DATA->globalMode = kGlobalModeParking;
			PRIVATE_DATA->park_state = PARK_PHASE0;
			PRIVATE_DATA->park_timer = indigo_set_timer(device, 0, park_timer_callback);
		}
		else {
			//  Can't park while mount is doing something else
			MOUNT_PARK_PROPERTY->state = INDIGO_ALERT_STATE;
			indigo_update_property(device, MOUNT_PARK_PROPERTY, "Mount busy!");
		}
	} else {
		MOUNT_PARK_PROPERTY->state = INDIGO_BUSY_STATE;
		indigo_update_property(device, MOUNT_PARK_PROPERTY, "Unparking...");

		//  We should try to move to an unpark position here - but we don't have one

		PRIVATE_DATA->parked = false;
		PRIVATE_DATA->globalMode = kGlobalModeIdle;
		MOUNT_PARK_PROPERTY->state = INDIGO_OK_STATE;
		indigo_update_property(device, MOUNT_PARK_PROPERTY, "Mount unparked.");
	}
}


static void axis_timer_callback(indigo_device *device, enum AxisID axis) {
#if 0
	enum AxisMode* axisMode = (axis == kAxisRA) ? &PRIVATE_DATA->raAxisMode : &PRIVATE_DATA->decAxisMode;
	enum AxisMode* desiredAxisMode = (axis == kAxisRA) ? &PRIVATE_DATA->raDesiredAxisMode : &PRIVATE_DATA->decDesiredAxisMode;
	double desiredAxisRate = (axis == kAxisRA) ? PRIVATE_DATA->raDesiredRate : PRIVATE_DATA->decDesiredRate;
	struct AxisConfig* cachedConfig = (axis == kAxisRA) ? &PRIVATE_DATA->raAxisConfig : &PRIVATE_DATA->decAxisConfig;

	switch (*axisMode) {
		case kAxisModeIdle:
		{
			switch (*desiredAxisMode) {
				case kAxisModeIdle:
					//  Nothing to do
					break;
				case kAxisModeGuiding:
					//  NOT IMPLEMENTED YET
					break;
				case kAxisModeTracking:
				case kAxisModeManualSlewing:
					//  Configure and slew the axis
					if (!synscan_configure_axis_for_rate(device, axis, desiredAxisRate) || !synscan_slew_axis(device, axis)) {
						break;
					}

					//  Move to tracking/slewing state
					*axisMode = *desiredAxisMode;
					break;
				case kAxisModeSlewing:
				{
					if (!synscan_slew_axis_to_position(device, axis, (axis == kAxisRA) ? PRIVATE_DATA->raTargetPosition : PRIVATE_DATA->decTargetPosition))
						break;

					//  Get the axis status
					while (true) {
						long axisStatus;
						if (!synscan_motor_status(device, axis, &axisStatus))
							break;

						//  Wait for axis to start moving
						if ((axisStatus & kStatusActiveMask) != 0)
							break;
					}

					*axisMode = kAxisModeSlewing;
					//usleep(500000);
				}
					break;
				case kAxisModeStopping:
				case kAxisModeSlewIdle:
					//  Should never encounter these as desired modes
					break;
			}
		}
			break;

		case kAxisModeStopping:
		{
			//  Get the axis status
			long axisStatus;
			if (!synscan_motor_status(device, axis, &axisStatus))
				break;

			//  Mark axis as stopped if motor has come to rest
			if ((axisStatus & kStatusActiveMask) == 0)
				*axisMode = kAxisModeIdle;
		}
			break;

		case kAxisModeGuiding:
			//  NOT IMPLEMENTED YET
			break;

		case kAxisModeTracking:
		case kAxisModeManualSlewing:
		{
			switch (*desiredAxisMode) {
				case kAxisModeIdle:
				case kAxisModeSlewing:
					synscan_stop_axis(device, axis);
					*axisMode = kAxisModeStopping;
					break;
				case kAxisModeManualSlewing:
					//  Nothing to do
					break;
				case kAxisModeGuiding:
					//  NOT IMPLEMENTED YET
					break;
				case kAxisModeTracking:
					if (desiredAxisRate != cachedConfig->slewRate) {
						//  Try a simple rate update on the axis
						bool result;
						if (!synscan_update_axis_to_rate(device, axis, desiredAxisRate, &result))
							break;

						//  If a simple rate adjustment wasn't possible, stop the axis first
						if (!result) {
							synscan_stop_axis(device, axis);
							*axisMode = kAxisModeStopping;
						}
					}
					break;
				case kAxisModeStopping:
				case kAxisModeSlewIdle:
					//  Should never encounter these as desired modes
					break;
			}
		}
			break;

		case kAxisModeSlewing:
		{
			switch (*desiredAxisMode) {
				case kAxisModeIdle:
					synscan_stop_axis(device, axis);
					*axisMode = kAxisModeStopping;
					break;
				case kAxisModeGuiding:
				case kAxisModeTracking:
				case kAxisModeManualSlewing:
					//  NOT POSSIBLE - ABORT IT
					break;
				case kAxisModeSlewing:
				{
					//  Get the axis status
					long axisStatus;
					if (!synscan_motor_status(device, axis, &axisStatus))
						break;

					//  Mark axis as stopped if motor has come to rest
					if ((axisStatus & kStatusActiveMask) == 0) {
						*desiredAxisMode = kAxisModeSlewIdle;
						*axisMode = kAxisModeSlewIdle;
					}
				}
					break;
				case kAxisModeStopping:
				case kAxisModeSlewIdle:
					//  Should never encounter these as desired modes
					break;
			}
		}
			break;

		case kAxisModeSlewIdle:
		{
			switch (*desiredAxisMode) {
				case kAxisModeIdle:
					*axisMode = kAxisModeIdle;
					//  Nothing to do
					break;
				case kAxisModeGuiding:
					//  NOT IMPLEMENTED YET
					break;
				case kAxisModeTracking:
				case kAxisModeManualSlewing:
					//  Configure and slew the axis
					if (!synscan_configure_axis_for_rate(device, axis, desiredAxisRate) || !synscan_slew_axis(device, axis)) {
						break;
					}

					//  Move to tracking state
					*axisMode = *desiredAxisMode;
					break;
				case kAxisModeSlewing:
					if (!synscan_slew_axis_to_position(device, axis, (axis == kAxisRA) ? PRIVATE_DATA->raTargetPosition : PRIVATE_DATA->decTargetPosition))
						break;
					*axisMode = kAxisModeSlewing;
					break;
				case kAxisModeStopping:
				case kAxisModeSlewIdle:
					//  Should never encounter these as desired modes
					break;
			}
		}
			break;
	}
#endif
}

void ha_axis_timer_callback(indigo_device* device) {
	axis_timer_callback(device, kAxisRA);
	indigo_reschedule_timer(device, 0.25, &PRIVATE_DATA->ha_axis_timer);
}

void dec_axis_timer_callback(indigo_device* device) {
	axis_timer_callback(device, kAxisDEC);
	indigo_reschedule_timer(device, 0.25, &PRIVATE_DATA->dec_axis_timer);
}



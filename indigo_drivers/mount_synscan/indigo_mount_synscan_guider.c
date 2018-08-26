//
//  indigo_mount_synscan_guider.c
//  indigo_server
//
//  Created by David Hulse on 21/08/2018.
//  Copyright © 2018 CloudMakers, s. r. o. All rights reserved.
//

#include <assert.h>
#include "indigo_guider_driver.h"
#include "indigo_mount_synscan_guider.h"

#define COMMAND_GUIDE_RATE_PROPERTY     (PRIVATE_DATA->command_guide_rate_property)
#define GUIDE_50_ITEM                   (COMMAND_GUIDE_RATE_PROPERTY->items+0)
#define GUIDE_100_ITEM                  (COMMAND_GUIDE_RATE_PROPERTY->items+1)

#define COMMAND_GUIDE_RATE_PROPERTY_NAME   "COMMAND_GUIDE_RATE"
#define GUIDE_50_ITEM_NAME                 "GUIDE_50"
#define GUIDE_100_ITEM_NAME                "GUIDE_100"

// -------------------------------------------------------------------------------- INDIGO guider device implementation

indigo_result
guider_enumerate_properties(indigo_device *device, indigo_client *client, indigo_property *property) {
	if (IS_CONNECTED) {
		//		if (indigo_property_match(COMMAND_GUIDE_RATE_PROPERTY, property))
		//			indigo_define_property(device, COMMAND_GUIDE_RATE_PROPERTY, NULL);
	}
	return indigo_guider_enumerate_properties(device, NULL, NULL);
}

static void guider_timer_callback_ra(indigo_device *device) {
	PRIVATE_DATA->guider_timer_ra = NULL;
//	res = tc_slew_fixed(dev_id, TC_AXIS_RA, TC_DIR_POSITIVE, 0); // STOP move
//	if (res != RC_OK) {
//		INDIGO_DRIVER_ERROR(DRIVER_NAME, "tc_slew_fixed(%d) = %d", dev_id, res);
//	}
//
	GUIDER_GUIDE_EAST_ITEM->number.value = 0;
	GUIDER_GUIDE_WEST_ITEM->number.value = 0;
	GUIDER_GUIDE_RA_PROPERTY->state = INDIGO_OK_STATE;
	indigo_update_property(device, GUIDER_GUIDE_RA_PROPERTY, NULL);
}

static void guider_timer_callback_dec(indigo_device *device) {
//	PRIVATE_DATA->guider_timer_dec = NULL;
//	int dev_id = PRIVATE_DATA->dev_id;
//	int res;
//
//	res = tc_slew_fixed(dev_id, TC_AXIS_DE, TC_DIR_POSITIVE, 0); // STOP move
//	if (res != RC_OK) {
//		INDIGO_DRIVER_ERROR(DRIVER_NAME, "tc_slew_fixed(%d) = %d", dev_id, res);
//	}
//
//	GUIDER_GUIDE_NORTH_ITEM->number.value = 0;
//	GUIDER_GUIDE_SOUTH_ITEM->number.value = 0;
//	GUIDER_GUIDE_DEC_PROPERTY->state = INDIGO_OK_STATE;
//	indigo_update_property(device, GUIDER_GUIDE_DEC_PROPERTY, NULL);
}


static void guider_handle_guide_rate(indigo_device *device) {
//	if(GUIDE_50_ITEM->sw.value) {
//		PRIVATE_DATA->guide_rate = 1;
//	} else if (GUIDE_100_ITEM->sw.value) {
//		PRIVATE_DATA->guide_rate = 2;
//	}
//	COMMAND_GUIDE_RATE_PROPERTY->state = INDIGO_OK_STATE;
//	if (PRIVATE_DATA->guide_rate == 1)
//		indigo_update_property(device, COMMAND_GUIDE_RATE_PROPERTY, "Command guide rate set to 7.5\"/s (50%% sidereal).");
//	else if (PRIVATE_DATA->guide_rate == 2)
//		indigo_update_property(device, COMMAND_GUIDE_RATE_PROPERTY, "Command guide rate set to 15\"/s (100%% sidereal).");
//	else
//		indigo_update_property(device, COMMAND_GUIDE_RATE_PROPERTY, "Command guide rate set.");
}


//- (void) pulseGuideRA:(double)ra DEC:(double)dec {
#if 0
	@synchronized(self) {
		//  Ignore if the mount is not tracking
		if (self.globalMode != kGlobalModeTracking || self.trackingMode == kTrackingModeOff)
			return;

		//  Get tracking base rate
		double base_rate = self.trackingRate;

		//  Get pulse rates
		NSUserDefaults* prefs = [NSUserDefaults standardUserDefaults];
		NSInteger raPulseRatePercent = [prefs integerForKey:@"raPulseGuideRate"];
		NSInteger decPulseRatePercent = [prefs integerForKey:@"decPulseGuideRate"];
		NSInteger minimumPulseLength = [prefs integerForKey:@"minPulseLength"];
		double raPulseRate = ((double)raPulseRatePercent / 100.0) * base_rate;
		double decPulseRate = ((double)decPulseRatePercent / 100.0) * base_rate;

		//  Apply RA pulse
		if (raPulseRatePercent > 0 && ra != 0.0) {
			[raPulseQueue addOperationWithBlock:^{
			 NSOperation* op;
			 if (ra < 0) {
				 //  Start a WEST slew pulse
				 op = [raAxis slewAxisAtRate:base_rate+raPulseRate];
			 }
			 else {
				 //  Start an EAST slew pulse
				 if (base_rate - raPulseRate < FLT_EPSILON /* 0.001 */)
				 	op = [raAxis stopAxis];
				 else
				 	op = [raAxis slewAxisAtRate:base_rate-raPulseRate];
			 }

			 //  Delay for the pulse duration
			 printf("Doing RA pulse %g secs\n", ra);
			 [op waitUntilFinished];

			 useconds_t duration = (useconds_t)lrint(fabs(ra) * 1000000);
			 if (duration < minimumPulseLength*1000)
				 duration = (useconds_t)minimumPulseLength*1000;
			 [self delayWithBlock:^{
				usleep(duration);
			 }];

			 //  Restore tracking
			 op = [raAxis slewAxisAtRate:base_rate];
		}
		else if (_raGuideStatus != nil) {
		}

		//  FIXME - these guiding status messages could overlap if we pulse both axes at once.
		//          Maybe we need to have separate labels in the UI that light up only when there is a N/S/E/W pulse happening.
		//          We'd have two of those so we can represent both axes pulsing at once.

		//  Apply DEC pulse
		if (decPulseRatePercent > 0 && dec != 0.0) {
			[decPulseQueue addOperationWithBlock:^{
			 NSOperation* op;
			 if (dec < 0) {
			 //  Start a SOUTH slew pulse
			 op = [decAxis slewAxisAtRate:-decPulseRate];
			 }
			 else {
			 //  Start an NORTH slew pulse
			 op = [decAxis slewAxisAtRate:decPulseRate];
			 }

			 //  Delay for the pulse duration
			 printf("Doing DEC pulse %g secs\n", dec);
			 [op waitUntilFinished];
			 useconds_t duration = (useconds_t)lrint(fabs(dec) * 1000000);
			 if (duration < minimumPulseLength*1000)
			 	 duration = (useconds_t)minimumPulseLength*1000;
			 [self delayWithBlock:^{
				usleep(duration);
				}];

			 //  Stop motor
			 op = [decAxis stopAxis];
		}
		else if (_decGuideStatus != nil) {
		}
	}
#endif
//}



indigo_result guider_attach(indigo_device *device) {
	assert(device != NULL);
	assert(PRIVATE_DATA != NULL);
	if (indigo_guider_attach(device, DRIVER_VERSION) == INDIGO_OK) {
		PRIVATE_DATA->pulse_guide_rate = 1; /* 1 -> 0.5 siderial rate , 2 -> siderial rate */
		COMMAND_GUIDE_RATE_PROPERTY = indigo_init_switch_property(NULL, device->name, COMMAND_GUIDE_RATE_PROPERTY_NAME, GUIDER_MAIN_GROUP, "Guide rate", INDIGO_IDLE_STATE, INDIGO_RW_PERM, INDIGO_ONE_OF_MANY_RULE, 2);
		if (COMMAND_GUIDE_RATE_PROPERTY == NULL)
			return INDIGO_FAILED;
		indigo_init_switch_item(GUIDE_50_ITEM, GUIDE_50_ITEM_NAME, "50% sidereal", true);
		indigo_init_switch_item(GUIDE_100_ITEM, GUIDE_100_ITEM_NAME, "100% sidereal", false);

		INDIGO_DEVICE_ATTACH_LOG(DRIVER_NAME, device->name);
		return indigo_guider_enumerate_properties(device, NULL, NULL);
	}
	return INDIGO_FAILED;
}

indigo_result guider_change_property(indigo_device *device, indigo_client *client, indigo_property *property) {
	assert(device != NULL);
	assert(DEVICE_CONTEXT != NULL);
	assert(property != NULL);
	// -------------------------------------------------------------------------------- CONNECTION
	if (indigo_property_match(CONNECTION_PROPERTY, property)) {
		indigo_property_copy_values(CONNECTION_PROPERTY, property, false);
		//  Talk to the mount - see if its there - read configuration data
		//  Once we have it all, update the property to OK or ALERT state
		//  This can be done by a background thread
		synscan_mount_connect(device);
		//  Falls through to base code to complete connection


//					indigo_define_property(device, COMMAND_GUIDE_RATE_PROPERTY, NULL);
//					PRIVATE_DATA->guider_timer_ra = NULL;
//					PRIVATE_DATA->guider_timer_dec = NULL;
//					CONNECTION_PROPERTY->state = INDIGO_OK_STATE;
//					GUIDER_GUIDE_DEC_PROPERTY->hidden = false;
//					GUIDER_GUIDE_RA_PROPERTY->hidden = false;
//
//			 indigo_delete_property(device, COMMAND_GUIDE_RATE_PROPERTY, NULL);
	} else if (indigo_property_match(GUIDER_GUIDE_RA_PROPERTY, property)) {
		// -------------------------------------------------------------------------------- GUIDER_GUIDE_RA
		indigo_cancel_timer(device, &PRIVATE_DATA->guider_timer_ra);
		indigo_property_copy_values(GUIDER_GUIDE_RA_PROPERTY, property, false);
		GUIDER_GUIDE_RA_PROPERTY->state = INDIGO_OK_STATE;
		int duration = GUIDER_GUIDE_EAST_ITEM->number.value;
		if (duration > 0) {
			//  Guide east for duration

			//			int res = tc_slew_fixed(PRIVATE_DATA->dev_id, TC_AXIS_RA, TC_DIR_POSITIVE, PRIVATE_DATA->guide_rate);
			//			GUIDER_GUIDE_RA_PROPERTY->state = INDIGO_BUSY_STATE;
			//			PRIVATE_DATA->guider_timer_ra = indigo_set_timer(device, duration/1000.0, guider_timer_callback_ra);
		} else {
			int duration = GUIDER_GUIDE_WEST_ITEM->number.value;
			if (duration > 0) {
				//  Guide west for duration
				//				int res = tc_slew_fixed(PRIVATE_DATA->dev_id, TC_AXIS_RA, TC_DIR_NEGATIVE, PRIVATE_DATA->guide_rate);
				//				GUIDER_GUIDE_RA_PROPERTY->state = INDIGO_BUSY_STATE;
				//				PRIVATE_DATA->guider_timer_ra = indigo_set_timer(device, duration/1000.0, guider_timer_callback_ra);
			}
		}
		indigo_update_property(device, GUIDER_GUIDE_RA_PROPERTY, NULL);
		return INDIGO_OK;
	} else if (indigo_property_match(GUIDER_GUIDE_DEC_PROPERTY, property)) {
		// -------------------------------------------------------------------------------- GUIDER_GUIDE_DEC
		indigo_cancel_timer(device, &PRIVATE_DATA->guider_timer_dec);
		indigo_property_copy_values(GUIDER_GUIDE_DEC_PROPERTY, property, false);
		GUIDER_GUIDE_DEC_PROPERTY->state = INDIGO_OK_STATE;
		int duration = GUIDER_GUIDE_NORTH_ITEM->number.value;
		if (duration > 0) {
			//  Guide north for duration
//			int res = tc_slew_fixed(PRIVATE_DATA->dev_id, TC_AXIS_DE, TC_DIR_POSITIVE, PRIVATE_DATA->guide_rate);
//			GUIDER_GUIDE_DEC_PROPERTY->state = INDIGO_BUSY_STATE;
//			PRIVATE_DATA->guider_timer_dec = indigo_set_timer(device, duration/1000.0, guider_timer_callback_dec);
		} else {
			int duration = GUIDER_GUIDE_SOUTH_ITEM->number.value;
			if (duration > 0) {
				//  Guide south for duration
//				int res = tc_slew_fixed(PRIVATE_DATA->dev_id, TC_AXIS_DE, TC_DIR_NEGATIVE, PRIVATE_DATA->guide_rate);
//				GUIDER_GUIDE_DEC_PROPERTY->state = INDIGO_BUSY_STATE;
//				PRIVATE_DATA->guider_timer_dec = indigo_set_timer(device, duration/1000.0, guider_timer_callback_dec);
			}
		}
		indigo_update_property(device, GUIDER_GUIDE_DEC_PROPERTY, NULL);
		return INDIGO_OK;
	} else if (indigo_property_match(COMMAND_GUIDE_RATE_PROPERTY, property)) {
		// -------------------------------------------------------------------------------- COMMAND_GUIDE_RATE
		indigo_property_copy_values(COMMAND_GUIDE_RATE_PROPERTY, property, false);
		guider_handle_guide_rate(device);
		return INDIGO_OK;
	}
	// --------------------------------------------------------------------------------
	return indigo_guider_change_property(device, client, property);
}

indigo_result guider_detach(indigo_device *device) {
	assert(device != NULL);
	if (CONNECTION_CONNECTED_ITEM->sw.value)
		indigo_device_disconnect(NULL, device->name);

	indigo_release_property(COMMAND_GUIDE_RATE_PROPERTY);
	INDIGO_DEVICE_DETACH_LOG(DRIVER_NAME, device->name);
	return indigo_guider_detach(device);
}

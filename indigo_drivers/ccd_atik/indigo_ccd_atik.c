// Copyright (c) 2016-2018 CloudMakers, s. r. o.
// All rights reserved.
//
// You can use this software under the terms of 'INDIGO Astronomy
// open-source license' (see LICENSE.md).
//
// THIS SOFTWARE IS PROVIDED BY THE AUTHORS 'AS IS' AND ANY EXPRESS
// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// version history
// 2.0 by Peter Polakovic <peter.polakovic@cloudmakers.eu>

/** INDIGO Atik CCD driver
 \file indigo_ccd_atik.c
 */

#define DRIVER_VERSION 0x000A
#define DRIVER_NAME "indigo_ccd_atik"

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <assert.h>
#include <pthread.h>
#include <sys/time.h>

#if defined(INDIGO_FREEBSD)
#include <libusb.h>
#else
#include <libusb-1.0/libusb.h>
#endif

#include "AtikCameras.h"

#include "indigo_usb_utils.h"
#include "indigo_driver_xml.h"

#include "indigo_ccd_atik.h"

#define PRIVATE_DATA        ((atik_private_data *)device->private_data)

#define ATIK_VID1	0x20E7
#define ATIK_VID2 0x04B4

#define ATIK_GUIDE_EAST             0x04     /* RA+ */
#define ATIK_GUIDE_NORTH            0x01     /* DEC+ */
#define ATIK_GUIDE_SOUTH            0x02     /* DEC- */
#define ATIK_GUIDE_WEST             0x08     /* RA- */

typedef struct {
	ArtemisHandle handle;
	int index;
	libusb_device *dev;
	int device_count;
	indigo_timer *exposure_timer, *temperature_timer, *guider_timer;
	unsigned short relay_mask;
	unsigned char *buffer;
	bool can_check_temperature;
} atik_private_data;

static bool do_log = true;

static void debug_log(const char *message) {
	if (do_log)
		indigo_debug("%s: SDK - %s", DRIVER_NAME, message);
}

// -------------------------------------------------------------------------------- INDIGO CCD device implementation

static void exposure_timer_callback(indigo_device *device) {
	PRIVATE_DATA->exposure_timer = NULL;
	CCD_EXPOSURE_ITEM->number.value = 0;
	indigo_update_property(device, CCD_EXPOSURE_PROPERTY, NULL);
	double remaining = ArtemisExposureTimeRemaining(PRIVATE_DATA->handle);
	if (remaining > 0)
		usleep(remaining * 1000000);
	while (!ArtemisImageReady(PRIVATE_DATA->handle)) {
		do_log = false;
		usleep(1000);
	}
	do_log = true;
	int left, top, width, height, binx, biny;
	if (ArtemisGetImageData(PRIVATE_DATA->handle, &left, &top, &width, &height, &binx, &biny) == ARTEMIS_OK) {
		void *buffer = ArtemisImageBuffer(PRIVATE_DATA->handle);
		memcpy(PRIVATE_DATA->buffer + FITS_HEADER_SIZE, buffer, width * height * 2);
		indigo_process_image(device, PRIVATE_DATA->buffer, width, height, 16, true, true, NULL);
		CCD_EXPOSURE_PROPERTY->state = INDIGO_OK_STATE;
		indigo_update_property(device, CCD_EXPOSURE_PROPERTY, NULL);
	} else {
		CCD_EXPOSURE_PROPERTY->state = INDIGO_ALERT_STATE;
		indigo_update_property(device, CCD_EXPOSURE_PROPERTY, "Exposure failed");
	}
	PRIVATE_DATA->can_check_temperature = true;
}

static void ccd_temperature_callback(indigo_device *device) {
	if (!CONNECTION_CONNECTED_ITEM->sw.value)
		return;
	if (PRIVATE_DATA->can_check_temperature) {
		int temperature, flags, level, min_level, max_level, set_point;
		if (ArtemisTemperatureSensorInfo(PRIVATE_DATA->handle, 1, &temperature) == ARTEMIS_OK) {
			CCD_TEMPERATURE_ITEM->number.value = round(temperature / 10.0) / 10.0;
			CCD_TEMPERATURE_PROPERTY->state = INDIGO_OK_STATE;
			if (CCD_TEMPERATURE_PROPERTY->perm == INDIGO_RW_PERM && CCD_COOLER_ON_ITEM->sw.value && ArtemisCoolingInfo(PRIVATE_DATA->handle, &flags, &level, &min_level, &max_level, &set_point) == ARTEMIS_OK) {
				CCD_TEMPERATURE_ITEM->number.target = round(set_point / 10.0) / 10.0;
				double diff = CCD_TEMPERATURE_ITEM->number.value - CCD_TEMPERATURE_ITEM->number.target;
				CCD_TEMPERATURE_PROPERTY->state = fabs(diff) > 1 ? INDIGO_BUSY_STATE : INDIGO_OK_STATE;
				CCD_COOLER_POWER_ITEM->number.value = round(100.0 * (level - min_level) / (max_level - min_level));
				CCD_COOLER_POWER_PROPERTY->state = INDIGO_OK_STATE;
				indigo_update_property(device, CCD_COOLER_POWER_PROPERTY, NULL);
			}
			indigo_update_property(device, CCD_TEMPERATURE_PROPERTY, NULL);
		}
	}
	indigo_reschedule_timer(device, 5, &PRIVATE_DATA->temperature_timer);
}

static void ccd_connect_callback(indigo_device *device) {
	if (PRIVATE_DATA->device_count++ == 0) {
		CONNECTION_PROPERTY->state = INDIGO_BUSY_STATE;
		indigo_update_property(device, CONNECTION_PROPERTY, NULL);
		if (indigo_try_global_lock(device) != INDIGO_OK) {
			INDIGO_DRIVER_ERROR(DRIVER_NAME, "indigo_try_global_lock(): failed to get lock.");
			PRIVATE_DATA->handle = NULL;
		} else {
			PRIVATE_DATA->handle = ArtemisConnect(PRIVATE_DATA->index);
		}
	}
	if (PRIVATE_DATA->handle) {
		struct ARTEMISPROPERTIES properties;
		int temperature, flags, level, min_level, max_level, set_point;
		if (ArtemisProperties(PRIVATE_DATA->handle, &properties) == ARTEMIS_OK) {
			if (properties.nPixelsX == 3354 && properties.nPixelsY == 2529) {
				properties.nPixelsX = 3326;
				properties.nPixelsY = 2504;
			}
			CCD_INFO_WIDTH_ITEM->number.value = CCD_FRAME_WIDTH_ITEM->number.value = CCD_FRAME_WIDTH_ITEM->number.max = CCD_FRAME_LEFT_ITEM->number.max = properties.nPixelsX;
			CCD_INFO_HEIGHT_ITEM->number.value = CCD_FRAME_HEIGHT_ITEM->number.value = CCD_FRAME_HEIGHT_ITEM->number.max = CCD_FRAME_TOP_ITEM->number.max = properties.nPixelsY;
			CCD_INFO_PIXEL_SIZE_ITEM->number.value = CCD_INFO_PIXEL_WIDTH_ITEM->number.value = round(properties.PixelMicronsX * 100)/100;
			CCD_INFO_PIXEL_HEIGHT_ITEM->number.value = round(properties.PixelMicronsX * 100) / 100;
			CCD_MODE_PROPERTY->perm = INDIGO_RW_PERM;
			CCD_MODE_PROPERTY->count = 3;
			char name[32];
			sprintf(name, "RAW 16 %dx%d", properties.nPixelsX, properties.nPixelsY);
			indigo_init_switch_item(CCD_MODE_ITEM, "BIN_1x1", name, true);
			sprintf(name, "RAW 16 %dx%d", properties.nPixelsX / 2, properties.nPixelsY / 2);
			indigo_init_switch_item(CCD_MODE_ITEM+1, "BIN_2x2", name, false);
			sprintf(name, "RAW 16 %dx%d", properties.nPixelsX / 4, properties.nPixelsY / 4);
			indigo_init_switch_item(CCD_MODE_ITEM+2, "BIN_4x4", name, false);
			PRIVATE_DATA->buffer = indigo_alloc_blob_buffer(2 * CCD_INFO_WIDTH_ITEM->number.value * CCD_INFO_HEIGHT_ITEM->number.value + FITS_HEADER_SIZE);
			assert(PRIVATE_DATA->buffer != NULL);
			CONNECTION_PROPERTY->state = INDIGO_OK_STATE;
			if (ArtemisTemperatureSensorInfo(PRIVATE_DATA->handle, 0, &temperature) == ARTEMIS_OK && temperature > 0 && ArtemisTemperatureSensorInfo(PRIVATE_DATA->handle, 1, &temperature) == ARTEMIS_OK) {
				CCD_TEMPERATURE_PROPERTY->hidden = false;
				CCD_TEMPERATURE_PROPERTY->perm = INDIGO_RO_PERM;
				CCD_TEMPERATURE_PROPERTY->state = INDIGO_OK_STATE;
				CCD_TEMPERATURE_ITEM->number.value = round(temperature / 10.0) / 10.0;
				PRIVATE_DATA->temperature_timer = indigo_set_timer(device, 5, ccd_temperature_callback);
				PRIVATE_DATA->can_check_temperature = true;
			}
			if (ArtemisCoolingInfo(PRIVATE_DATA->handle, &flags, &level, &min_level, &max_level, &set_point) == ARTEMIS_OK && (flags & 3) == 3) {
				CCD_COOLER_PROPERTY->hidden = false;
				CCD_COOLER_POWER_PROPERTY->hidden = false;
				CCD_COOLER_POWER_PROPERTY->perm = INDIGO_RO_PERM;
				CCD_TEMPERATURE_PROPERTY->perm = INDIGO_RW_PERM;
				CCD_TEMPERATURE_ITEM->number.target = round(set_point / 10.0) / 10.0;
				double diff = CCD_TEMPERATURE_ITEM->number.value - CCD_TEMPERATURE_ITEM->number.target;
				if (CCD_COOLER_ON_ITEM->sw.value)
					CCD_TEMPERATURE_PROPERTY->state = fabs(diff) > 1 ? INDIGO_BUSY_STATE : INDIGO_OK_STATE;
				else
					CCD_TEMPERATURE_PROPERTY->state = INDIGO_OK_STATE;
				CCD_COOLER_POWER_PROPERTY->state = INDIGO_OK_STATE;
				CCD_COOLER_POWER_ITEM->number.value = round(100.0 * (level - min_level) / (max_level - min_level));
			}
		} else {
			CONNECTION_PROPERTY->state = INDIGO_ALERT_STATE;
			ArtemisDisconnect(PRIVATE_DATA->handle);
			PRIVATE_DATA->handle = NULL;
		}
	}
	if (PRIVATE_DATA->handle == NULL) {
		indigo_cancel_timer(device, &PRIVATE_DATA->temperature_timer);
		if (PRIVATE_DATA->buffer != NULL) {
			free(PRIVATE_DATA->buffer);
			PRIVATE_DATA->buffer = NULL;
		}
		PRIVATE_DATA->device_count--;
		CONNECTION_PROPERTY->state = INDIGO_ALERT_STATE;
		indigo_set_switch(CONNECTION_PROPERTY, CONNECTION_DISCONNECTED_ITEM, true);
	}
	indigo_ccd_change_property(device, NULL, CONNECTION_PROPERTY);
}


static indigo_result ccd_attach(indigo_device *device) {
	assert(device != NULL);
	assert(PRIVATE_DATA != NULL);
	if (indigo_ccd_attach(device, DRIVER_VERSION) == INDIGO_OK) {
		// --------------------------------------------------------------------------------
		CCD_EXPOSURE_ITEM->number.min = 0.001;
		CCD_BIN_PROPERTY->perm = INDIGO_RW_PERM;
		CCD_BIN_HORIZONTAL_ITEM->number.max = CCD_INFO_MAX_HORIZONAL_BIN_ITEM->number.value = 4;
		CCD_BIN_VERTICAL_ITEM->number.max = CCD_INFO_MAX_VERTICAL_BIN_ITEM->number.value = 4;
		CCD_INFO_BITS_PER_PIXEL_ITEM->number.value = 16;
		CCD_READ_MODE_PROPERTY->hidden = false;
		// --------------------------------------------------------------------------------
		INDIGO_DEVICE_ATTACH_LOG(DRIVER_NAME, device->name);
		return indigo_ccd_enumerate_properties(device, NULL, NULL);
	}
	return INDIGO_FAILED;
}

static indigo_result ccd_change_property(indigo_device *device, indigo_client *client, indigo_property *property) {
	assert(device != NULL);
	assert(DEVICE_CONTEXT != NULL);
	assert(property != NULL);
	if (indigo_property_match(CONNECTION_PROPERTY, property)) {
		// -------------------------------------------------------------------------------- CONNECTION -> CCD_INFO, CCD_COOLER, CCD_TEMPERATURE
		indigo_property_copy_values(CONNECTION_PROPERTY, property, false);
		if (CONNECTION_CONNECTED_ITEM->sw.value) {
			indigo_set_timer(device, 0, ccd_connect_callback);
			return INDIGO_OK;
		} else {
			indigo_cancel_timer(device, &PRIVATE_DATA->temperature_timer);
			if (PRIVATE_DATA->buffer != NULL) {
				free(PRIVATE_DATA->buffer);
				PRIVATE_DATA->buffer = NULL;
			}
			if (--PRIVATE_DATA->device_count == 0) {
				ArtemisCoolerWarmUp(PRIVATE_DATA->handle);
				ArtemisDisconnect(PRIVATE_DATA->handle);
				PRIVATE_DATA->handle = NULL;
				indigo_global_unlock(device);
			}
			CONNECTION_PROPERTY->state = INDIGO_OK_STATE;
		}
	} else if (indigo_property_match(CCD_EXPOSURE_PROPERTY, property)) {
		// -------------------------------------------------------------------------------- CCD_EXPOSURE
		if (CCD_EXPOSURE_PROPERTY->state == INDIGO_BUSY_STATE)
			return INDIGO_OK;
		indigo_property_copy_values(CCD_EXPOSURE_PROPERTY, property, false);
		indigo_use_shortest_exposure_if_bias(device);
		CCD_EXPOSURE_PROPERTY->state = INDIGO_BUSY_STATE;
		indigo_update_property(device, CCD_EXPOSURE_PROPERTY, NULL);
		while (true) {
			int state = ArtemisCameraState(PRIVATE_DATA->handle);
			do_log = true;
			if (state == CAMERA_IDLE) {
				ArtemisSetPreview(PRIVATE_DATA->handle, CCD_READ_MODE_HIGH_SPEED_ITEM->sw.value);
				ArtemisSetDarkMode(PRIVATE_DATA->handle, CCD_FRAME_TYPE_DARK_ITEM->sw.value);
				ArtemisBin(PRIVATE_DATA->handle, (int)CCD_BIN_HORIZONTAL_ITEM->number.value, (int)CCD_BIN_VERTICAL_ITEM->number.value);
				ArtemisSubframe(PRIVATE_DATA->handle, (int)CCD_FRAME_LEFT_ITEM->number.value, (int)CCD_FRAME_TOP_ITEM->number.value, (int)CCD_FRAME_WIDTH_ITEM->number.value, (int)CCD_FRAME_HEIGHT_ITEM->number.value);
				if (ArtemisStartExposure(PRIVATE_DATA->handle, CCD_EXPOSURE_ITEM->number.target) == ARTEMIS_OK) {
					PRIVATE_DATA->exposure_timer = indigo_set_timer(device, CCD_EXPOSURE_ITEM->number.target, exposure_timer_callback);
				} else {
					CCD_EXPOSURE_PROPERTY->state = INDIGO_ALERT_STATE;
					indigo_update_property(device, CCD_EXPOSURE_PROPERTY, NULL);
					return INDIGO_OK;
				}
				break;
			} else if (state == CAMERA_FLUSHING) {
				usleep(1000);
				do_log = false;
			} else {
				CCD_EXPOSURE_PROPERTY->state = INDIGO_ALERT_STATE;
				indigo_update_property(device, CCD_EXPOSURE_PROPERTY, NULL);
				return INDIGO_OK;
			}
		}
	} else if (indigo_property_match(CCD_ABORT_EXPOSURE_PROPERTY, property)) {
		// -------------------------------------------------------------------------------- CCD_ABORT_EXPOSURE
		indigo_property_copy_values(CCD_ABORT_EXPOSURE_PROPERTY, property, false);
		if (CCD_ABORT_EXPOSURE_ITEM->sw.value && CCD_EXPOSURE_PROPERTY->state == INDIGO_BUSY_STATE) {
			ArtemisStopExposure(PRIVATE_DATA->handle);
			indigo_cancel_timer(device, &PRIVATE_DATA->exposure_timer);
		}
		PRIVATE_DATA->can_check_temperature = true;
	} else if (indigo_property_match(CCD_BIN_PROPERTY, property)) {
		// -------------------------------------------------------------------------------- CCD_BIN
		int h = CCD_BIN_HORIZONTAL_ITEM->number.value;
		int v = CCD_BIN_VERTICAL_ITEM->number.value;
		indigo_property_copy_values(CCD_BIN_PROPERTY, property, false);
		if (!(CCD_BIN_HORIZONTAL_ITEM->number.value == 1 || CCD_BIN_HORIZONTAL_ITEM->number.value == 2 || CCD_BIN_HORIZONTAL_ITEM->number.value == 4) || CCD_BIN_HORIZONTAL_ITEM->number.value != CCD_BIN_VERTICAL_ITEM->number.value) {
			CCD_BIN_HORIZONTAL_ITEM->number.value = h;
			CCD_BIN_VERTICAL_ITEM->number.value = v;
			CCD_BIN_PROPERTY->state = INDIGO_ALERT_STATE;
			if (IS_CONNECTED) {
				indigo_update_property(device, CCD_BIN_PROPERTY, NULL);
			}
			return INDIGO_OK;
		}
	} else if (indigo_property_match(CCD_COOLER_PROPERTY, property)) {
		// -------------------------------------------------------------------------------- CCD_COOLER
		indigo_property_copy_values(CCD_COOLER_PROPERTY, property, false);
		if (CONNECTION_CONNECTED_ITEM->sw.value && !CCD_COOLER_PROPERTY->hidden) {
			ArtemisSetCooling(PRIVATE_DATA->handle, CCD_TEMPERATURE_ITEM->number.target * 100);
		} else {
			ArtemisCoolerWarmUp(PRIVATE_DATA->handle);
			CCD_COOLER_POWER_ITEM->number.value = 0;
			CCD_COOLER_POWER_PROPERTY->state = INDIGO_OK_STATE;
			indigo_update_property(device, CCD_COOLER_POWER_PROPERTY, NULL);
		}
		CCD_COOLER_PROPERTY->state = INDIGO_OK_STATE;
		indigo_update_property(device, CCD_COOLER_PROPERTY, NULL);
		return INDIGO_OK;
	} else if (indigo_property_match(CCD_TEMPERATURE_PROPERTY, property)) {
		// -------------------------------------------------------------------------------- CCD_TEMPERATURE
		double temperature = CCD_TEMPERATURE_ITEM->number.value;
		indigo_property_copy_values(CCD_TEMPERATURE_PROPERTY, property, false);
		CCD_TEMPERATURE_ITEM->number.value = temperature;
		if (CONNECTION_CONNECTED_ITEM->sw.value && !CCD_COOLER_PROPERTY->hidden) {
			ArtemisSetCooling(PRIVATE_DATA->handle, CCD_TEMPERATURE_ITEM->number.target * 100);
			if (CCD_COOLER_OFF_ITEM->sw.value) {
				indigo_set_switch(CCD_COOLER_PROPERTY, CCD_COOLER_ON_ITEM, true);
				CCD_COOLER_PROPERTY->state = INDIGO_OK_STATE;
				indigo_update_property(device, CCD_COOLER_PROPERTY, NULL);
			}
			CCD_TEMPERATURE_PROPERTY->state = INDIGO_BUSY_STATE;
			indigo_update_property(device, CCD_TEMPERATURE_PROPERTY, NULL);
		}
		return INDIGO_OK;
		// --------------------------------------------------------------------------------
	}
	return indigo_ccd_change_property(device, client, property);
}

static indigo_result ccd_detach(indigo_device *device) {
	assert(device != NULL);
	if (CONNECTION_CONNECTED_ITEM->sw.value)
		indigo_device_disconnect(NULL, device->name);
	if (device == device->master_device)
		indigo_global_unlock(device);
	INDIGO_DEVICE_DETACH_LOG(DRIVER_NAME, device->name);
	return indigo_ccd_detach(device);
}

// -------------------------------------------------------------------------------- INDIGO guider device implementation

static void guider_timer_callback(indigo_device *device) {
	PRIVATE_DATA->guider_timer = NULL;
	if (!CONNECTION_CONNECTED_ITEM->sw.value)
		return;
	
	ArtemisGuidePort(PRIVATE_DATA->handle, 0);
	if (PRIVATE_DATA->relay_mask & (ATIK_GUIDE_NORTH | ATIK_GUIDE_SOUTH)) {
		GUIDER_GUIDE_NORTH_ITEM->number.value = 0;
		GUIDER_GUIDE_SOUTH_ITEM->number.value = 0;
		GUIDER_GUIDE_DEC_PROPERTY->state = INDIGO_OK_STATE;
		indigo_update_property(device, GUIDER_GUIDE_DEC_PROPERTY, NULL);
	}
	if (PRIVATE_DATA->relay_mask & (ATIK_GUIDE_EAST | ATIK_GUIDE_WEST)) {
		GUIDER_GUIDE_EAST_ITEM->number.value = 0;
		GUIDER_GUIDE_WEST_ITEM->number.value = 0;
		GUIDER_GUIDE_RA_PROPERTY->state = INDIGO_OK_STATE;
		indigo_update_property(device, GUIDER_GUIDE_RA_PROPERTY, NULL);
	}
	PRIVATE_DATA->relay_mask = 0;
}

static indigo_result guider_attach(indigo_device *device) {
	assert(device != NULL);
	assert(PRIVATE_DATA != NULL);
	if (indigo_guider_attach(device, DRIVER_VERSION) == INDIGO_OK) {
		INDIGO_DEVICE_ATTACH_LOG(DRIVER_NAME, device->name);
		return indigo_guider_enumerate_properties(device, NULL, NULL);
	}
	return INDIGO_FAILED;
}

static indigo_result guider_change_property(indigo_device *device, indigo_client *client, indigo_property *property) {
	assert(device != NULL);
	assert(DEVICE_CONTEXT != NULL);
	assert(property != NULL);
	if (indigo_property_match(CONNECTION_PROPERTY, property)) {
		// -------------------------------------------------------------------------------- CONNECTION
		indigo_property_copy_values(CONNECTION_PROPERTY, property, false);
		if (CONNECTION_CONNECTED_ITEM->sw.value) {
			if (PRIVATE_DATA->device_count++ == 0) {
				CONNECTION_PROPERTY->state = INDIGO_BUSY_STATE;
				indigo_update_property(device, CONNECTION_PROPERTY, NULL);
				if (indigo_try_global_lock(device) != INDIGO_OK) {
					INDIGO_DRIVER_ERROR(DRIVER_NAME, "indigo_try_global_lock(): failed to get lock.");
					PRIVATE_DATA->handle = NULL;
				} else {
					PRIVATE_DATA->handle = ArtemisConnect(PRIVATE_DATA->index);
				}
			}
			if (PRIVATE_DATA->handle) {
				ArtemisGuidePort(PRIVATE_DATA->handle, PRIVATE_DATA->relay_mask = 0);
				CONNECTION_PROPERTY->state = INDIGO_OK_STATE;
			} else {
				PRIVATE_DATA->device_count--;
				CONNECTION_PROPERTY->state = INDIGO_ALERT_STATE;
				indigo_set_switch(CONNECTION_PROPERTY, CONNECTION_DISCONNECTED_ITEM, true);
			}
		} else {
			if (--PRIVATE_DATA->device_count == 0) {
				ArtemisDisconnect(PRIVATE_DATA->handle);
				PRIVATE_DATA->handle = NULL;
				indigo_global_unlock(device);
			}
			CONNECTION_PROPERTY->state = INDIGO_OK_STATE;
		}
	} else if (indigo_property_match(GUIDER_GUIDE_DEC_PROPERTY, property)) {
		// -------------------------------------------------------------------------------- GUIDER_GUIDE_DEC
		indigo_property_copy_values(GUIDER_GUIDE_DEC_PROPERTY, property, false);
		indigo_cancel_timer(device, &PRIVATE_DATA->guider_timer);
		PRIVATE_DATA->relay_mask &= ~(ATIK_GUIDE_NORTH | ATIK_GUIDE_SOUTH);
		int duration = GUIDER_GUIDE_NORTH_ITEM->number.value;
		if (duration > 0) {
			PRIVATE_DATA->relay_mask |= ATIK_GUIDE_NORTH;
			PRIVATE_DATA->guider_timer = indigo_set_timer(device, duration/1000.0, guider_timer_callback);
		} else {
			int duration = GUIDER_GUIDE_SOUTH_ITEM->number.value;
			if (duration > 0) {
				PRIVATE_DATA->relay_mask |= ATIK_GUIDE_SOUTH;
				PRIVATE_DATA->guider_timer = indigo_set_timer(device, duration/1000.0, guider_timer_callback);
			}
		}
		ArtemisGuidePort(PRIVATE_DATA->handle, PRIVATE_DATA->relay_mask);
		GUIDER_GUIDE_DEC_PROPERTY->state = PRIVATE_DATA->relay_mask & (ATIK_GUIDE_NORTH | ATIK_GUIDE_SOUTH) ? INDIGO_BUSY_STATE : INDIGO_OK_STATE;
		indigo_update_property(device, GUIDER_GUIDE_DEC_PROPERTY, NULL);
		return INDIGO_OK;
	} else if (indigo_property_match(GUIDER_GUIDE_RA_PROPERTY, property)) {
		// -------------------------------------------------------------------------------- GUIDER_GUIDE_RA
		indigo_property_copy_values(GUIDER_GUIDE_RA_PROPERTY, property, false);
		indigo_cancel_timer(device, &PRIVATE_DATA->guider_timer);
		PRIVATE_DATA->relay_mask &= ~(ATIK_GUIDE_EAST | ATIK_GUIDE_WEST);
		int duration = GUIDER_GUIDE_EAST_ITEM->number.value;
		if (duration > 0) {
			PRIVATE_DATA->relay_mask |= ATIK_GUIDE_EAST;
			PRIVATE_DATA->guider_timer = indigo_set_timer(device, duration/1000.0, guider_timer_callback);
		} else {
			int duration = GUIDER_GUIDE_WEST_ITEM->number.value;
			if (duration > 0) {
				PRIVATE_DATA->relay_mask |= ATIK_GUIDE_WEST;
				PRIVATE_DATA->guider_timer = indigo_set_timer(device, duration/1000.0, guider_timer_callback);
			}
		}
		ArtemisGuidePort(PRIVATE_DATA->handle, PRIVATE_DATA->relay_mask);
		GUIDER_GUIDE_RA_PROPERTY->state = PRIVATE_DATA->relay_mask & (ATIK_GUIDE_EAST | ATIK_GUIDE_WEST) ? INDIGO_BUSY_STATE : INDIGO_OK_STATE;
		indigo_update_property(device, GUIDER_GUIDE_RA_PROPERTY, NULL);
		return INDIGO_OK;
		// --------------------------------------------------------------------------------
	}
	return indigo_guider_change_property(device, client, property);
}

static indigo_result guider_detach(indigo_device *device) {
	assert(device != NULL);
	if (CONNECTION_CONNECTED_ITEM->sw.value)
		indigo_device_disconnect(NULL, device->name);
	if (device == device->master_device)
		indigo_global_unlock(device);
	INDIGO_DEVICE_DETACH_LOG(DRIVER_NAME, device->name);
	return indigo_guider_detach(device);
}

// -------------------------------------------------------------------------------- INDIGO wheel device implementation

static void wheel_timer_callback(indigo_device *device) {
	if (!CONNECTION_CONNECTED_ITEM->sw.value)
		return;
	
	int num_filters, moving, current_pos, target_pos;
	if (ArtemisFilterWheelInfo(PRIVATE_DATA->handle, &num_filters, &moving, &current_pos, &target_pos) == ARTEMIS_OK) {
		WHEEL_SLOT_ITEM->number.value = current_pos;
		WHEEL_SLOT_ITEM->number.target = target_pos;
		if (current_pos == target_pos) {
			WHEEL_SLOT_PROPERTY->state = INDIGO_OK_STATE;
		} else {
			WHEEL_SLOT_PROPERTY->state = INDIGO_BUSY_STATE;
			indigo_set_timer(device, 0.5, wheel_timer_callback);
		}
	} else {
		WHEEL_SLOT_PROPERTY->state = INDIGO_ALERT_STATE;
	}
	indigo_update_property(device, WHEEL_SLOT_PROPERTY, NULL);
}

static indigo_result wheel_attach(indigo_device *device) {
	assert(device != NULL);
	assert(PRIVATE_DATA != NULL);
	if (indigo_wheel_attach(device, DRIVER_VERSION) == INDIGO_OK) {
		INDIGO_DEVICE_ATTACH_LOG(DRIVER_NAME, device->name);
		return indigo_wheel_enumerate_properties(device, NULL, NULL);
	}
	return INDIGO_FAILED;
}

static indigo_result wheel_change_property(indigo_device *device, indigo_client *client, indigo_property *property) {
	assert(device != NULL);
	assert(DEVICE_CONTEXT != NULL);
	assert(property != NULL);
	if (indigo_property_match(CONNECTION_PROPERTY, property)) {
		// -------------------------------------------------------------------------------- CONNECTION
		indigo_property_copy_values(CONNECTION_PROPERTY, property, false);
		if (CONNECTION_CONNECTED_ITEM->sw.value) {
			if (PRIVATE_DATA->device_count++ == 0) {
				CONNECTION_PROPERTY->state = INDIGO_BUSY_STATE;
				indigo_update_property(device, CONNECTION_PROPERTY, NULL);
				if (indigo_try_global_lock(device) != INDIGO_OK) {
					INDIGO_DRIVER_ERROR(DRIVER_NAME, "indigo_try_global_lock(): failed to get lock.");
					PRIVATE_DATA->handle = NULL;
				} else {
					PRIVATE_DATA->handle = ArtemisConnect(PRIVATE_DATA->index);
				}
			}
			if (PRIVATE_DATA->handle) {
				int num_filters, moving, current_pos, target_pos;
				if (ArtemisFilterWheelInfo(PRIVATE_DATA->handle, &num_filters, &moving, &current_pos, &target_pos) == ARTEMIS_OK) {
					WHEEL_SLOT_ITEM->number.max = WHEEL_SLOT_NAME_PROPERTY->count = num_filters;
					WHEEL_SLOT_ITEM->number.value = current_pos;
					WHEEL_SLOT_ITEM->number.target = target_pos;
					CONNECTION_PROPERTY->state = INDIGO_OK_STATE;
				}
			} else {
				PRIVATE_DATA->device_count--;
				CONNECTION_PROPERTY->state = INDIGO_ALERT_STATE;
				indigo_set_switch(CONNECTION_PROPERTY, CONNECTION_DISCONNECTED_ITEM, true);
			}
		} else {
			if (--PRIVATE_DATA->device_count == 0) {
				ArtemisDisconnect(PRIVATE_DATA->handle);
				PRIVATE_DATA->handle = NULL;
				indigo_global_unlock(device);
			}
			CONNECTION_PROPERTY->state = INDIGO_OK_STATE;
		}
	} else if (indigo_property_match(WHEEL_SLOT_PROPERTY, property)) {
		// -------------------------------------------------------------------------------- WHEEL_SLOT
		double slot = WHEEL_SLOT_ITEM->number.value;
		indigo_property_copy_values(WHEEL_SLOT_PROPERTY, property, false);
		WHEEL_SLOT_ITEM->number.value = slot;
		if (WHEEL_SLOT_ITEM->number.value < 1 || WHEEL_SLOT_ITEM->number.value > WHEEL_SLOT_ITEM->number.max) {
			WHEEL_SLOT_PROPERTY->state = INDIGO_ALERT_STATE;
		} else {
			WHEEL_SLOT_PROPERTY->state = INDIGO_BUSY_STATE;
			ArtemisFilterWheelMove(PRIVATE_DATA->handle, (int)WHEEL_SLOT_ITEM->number.target);
			indigo_set_timer(device, 0.5, wheel_timer_callback);
		}
		indigo_update_property(device, WHEEL_SLOT_PROPERTY, NULL);
		return INDIGO_OK;
		// --------------------------------------------------------------------------------
	}
	return indigo_wheel_change_property(device, client, property);
}

static indigo_result wheel_detach(indigo_device *device) {
	assert(device != NULL);
	if (CONNECTION_CONNECTED_ITEM->sw.value)
		indigo_device_disconnect(NULL, device->name);
	if (device == device->master_device)
		indigo_global_unlock(device);
	INDIGO_DEVICE_DETACH_LOG(DRIVER_NAME, device->name);
	return indigo_wheel_detach(device);
}


// -------------------------------------------------------------------------------- hot-plug support

#define MAX_DEVICES                   10

static indigo_device *devices[MAX_DEVICES];

static void plug_handler(indigo_device *device) {
	static indigo_device ccd_template = INDIGO_DEVICE_INITIALIZER(
		"",
		ccd_attach,
		indigo_ccd_enumerate_properties,
		ccd_change_property,
		NULL,
		ccd_detach
		);
	static indigo_device guider_template = INDIGO_DEVICE_INITIALIZER(
		"",
		guider_attach,
		indigo_guider_enumerate_properties,
		guider_change_property,
		NULL,
		guider_detach
		);
	static indigo_device wheel_template = INDIGO_DEVICE_INITIALIZER(
		"",
		wheel_attach,
		indigo_wheel_enumerate_properties,
		wheel_change_property,
		NULL,
		wheel_detach
		);
	for (int i = 0; i < MAX_DEVICES; i++) {
		indigo_device *device = devices[i];
		if (device)
			PRIVATE_DATA->index = -1;
	}
	int count = ArtemisDeviceCount();
	for (int j = 0; j < count; j++) {
		libusb_device *dev;
		if (ArtemisDeviceGetLibUSBDevice(j, &dev) == ARTEMIS_OK) {
			for (int i = 0; i < MAX_DEVICES; i++) {
				indigo_device *device = devices[i];
				if (device && PRIVATE_DATA->dev == dev) {
					PRIVATE_DATA->index = j;
					dev = NULL;
					break;
				}
			}
		}
		if (dev) {
			atik_private_data *private_data = malloc(sizeof(atik_private_data));
			assert(private_data != NULL);
			memset(private_data, 0, sizeof(atik_private_data));
			private_data->index = j;
			private_data->dev = dev;
			indigo_device *device = malloc(sizeof(indigo_device));
			indigo_device *master_device = device;
			assert(device != NULL);
			memcpy(device, &ccd_template, sizeof(indigo_device));
			device->master_device = master_device;
			char name[INDIGO_NAME_SIZE], usb_path[INDIGO_NAME_SIZE];
			ArtemisDeviceName(j, name);
			indigo_get_usb_path(dev, usb_path);
			snprintf(device->name, INDIGO_NAME_SIZE, "%s #%s", name, usb_path);
			device->private_data = private_data;
			for (int i = 0; i < MAX_DEVICES; i++) {
				if (devices[i] == NULL) {
					indigo_async((void *)(void *)indigo_attach_device, devices[i] = device);
					break;
				}
			}
			if (ArtemisDeviceHasGuidePort(j)) {
				device = malloc(sizeof(indigo_device));
				assert(device != NULL);
				memcpy(device, &guider_template, sizeof(indigo_device));
				device->master_device = master_device;
				snprintf(device->name, INDIGO_NAME_SIZE, "%s (guider) #%s", name, usb_path);
				device->private_data = private_data;
				for (int j = 0; j < MAX_DEVICES; j++) {
					if (devices[j] == NULL) {
						indigo_async((void *)(void *)indigo_attach_device, devices[j] = device);
						break;
					}
				}
			}
			if (ArtemisDeviceHasFilterWheel(j)) {
				device = malloc(sizeof(indigo_device));
				assert(device != NULL);
				memcpy(device, &wheel_template, sizeof(indigo_device));
				device->master_device = master_device;
				snprintf(device->name, INDIGO_NAME_SIZE, "%s (wheel) #%s", name, usb_path);
				device->private_data = private_data;
				for (int j = 0; j < MAX_DEVICES; j++) {
					if (devices[j] == NULL) {
						indigo_async((void *)(void *)indigo_attach_device, devices[j] = device);
						break;
					}
				}
			}
		}
	}
}

static void unplug_handler(indigo_device *device) {
	for (int i = 0; i < MAX_DEVICES; i++) {
		indigo_device *device = devices[i];
		if (device)
			PRIVATE_DATA->index = -1;
	}
	int count = ArtemisDeviceCount();
	for (int j = 0; j < count; j++) {
		libusb_device *dev;
		if (ArtemisDeviceGetLibUSBDevice(j, &dev) == ARTEMIS_OK) {
			for (int i = 0; i < MAX_DEVICES; i++) {
				indigo_device *device = devices[i];
				if (device && PRIVATE_DATA->dev == dev) {
					PRIVATE_DATA->index = j;
					dev = NULL;
					break;
				}
			}
		}
	}
	atik_private_data *private_data = NULL;
	for (int i = 0; i < MAX_DEVICES; i++) {
		indigo_device *device = devices[i];
		if (device && PRIVATE_DATA->index == -1) {
			private_data = PRIVATE_DATA;
			indigo_detach_device(device);
			free(device);
			devices[i] = NULL;
		}
	}
	if (private_data) {
		if (private_data->buffer)
			free(private_data->buffer);
		free(private_data);
	}
}

static int hotplug_callback(libusb_context *ctx, libusb_device *dev, libusb_hotplug_event event, void *user_data) {
	switch (event) {
		case LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED: {
			indigo_set_timer(NULL, 0.5, plug_handler);
			break;
		}
		case LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT: {
			indigo_set_timer(NULL, 0.5, unplug_handler);
			break;
		}
	}
	return 0;
}

static libusb_hotplug_callback_handle callback_handle1, callback_handle2;

indigo_result indigo_ccd_atik(indigo_driver_action action, indigo_driver_info *info) {
	ArtemisSetDebugCallback(debug_log);
	
	static indigo_driver_action last_action = INDIGO_DRIVER_SHUTDOWN;
	
	SET_DRIVER_INFO(info, "Atik Camera", __FUNCTION__, DRIVER_VERSION, true, last_action);
	
	if (action == last_action)
		return INDIGO_OK;
	
	switch(action) {
		case INDIGO_DRIVER_INIT:
			last_action = action;
			for (int i = 0; i < MAX_DEVICES; i++) {
				devices[i] = NULL;
			}
			INDIGO_DRIVER_LOG(DRIVER_NAME, "Artemis SDK %d", ArtemisDLLVersion());
			indigo_start_usb_event_handler();
			int rc = libusb_hotplug_register_callback(NULL, LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT, LIBUSB_HOTPLUG_ENUMERATE, ATIK_VID1, LIBUSB_HOTPLUG_MATCH_ANY, LIBUSB_HOTPLUG_MATCH_ANY, hotplug_callback, NULL, &callback_handle1);
			if (rc >= 0)
				rc = libusb_hotplug_register_callback(NULL, LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT, LIBUSB_HOTPLUG_ENUMERATE, ATIK_VID2, LIBUSB_HOTPLUG_MATCH_ANY, LIBUSB_HOTPLUG_MATCH_ANY, hotplug_callback, NULL, &callback_handle2);
			INDIGO_DRIVER_DEBUG(DRIVER_NAME, "libusb_hotplug_register_callback ->  %s", rc < 0 ? libusb_error_name(rc) : "OK");
			return rc >= 0 ? INDIGO_OK : INDIGO_FAILED;
			
		case INDIGO_DRIVER_SHUTDOWN:
			last_action = action;
			libusb_hotplug_deregister_callback(NULL, callback_handle1);
			libusb_hotplug_deregister_callback(NULL, callback_handle2);
			INDIGO_DRIVER_DEBUG(DRIVER_NAME, "libusb_hotplug_deregister_callback");
			for (int j = 0; j < MAX_DEVICES; j++) {
				if (devices[j] != NULL) {
					indigo_device *device = devices[j];
					hotplug_callback(NULL, PRIVATE_DATA->dev, LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT, NULL);
				}
			}
			ArtemisShutdown();
			break;
			
		case INDIGO_DRIVER_INFO:
			break;
	}
	
	return INDIGO_OK;
}

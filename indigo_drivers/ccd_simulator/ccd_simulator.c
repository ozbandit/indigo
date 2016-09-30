//  Copyright (c) 2016 CloudMakers, s. r. o.
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//  1. Redistributions of source code must retain the above copyright
//  notice, this list of conditions and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above
//  copyright notice, this list of conditions and the following
//  disclaimer in the documentation and/or other materials provided
//  with the distribution.
//
//  3. The name of the author may not be used to endorse or promote
//  products derived from this software without specific prior
//  written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
//  OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
//  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
//  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
//  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
//  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//  version history
//  0.0 PoC by Peter Polakovic <peter.polakovic@cloudmakers.eu>

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>

#include "ccd_simulator.h"
#include "indigo_timer.h"

#define DEVICE "CCD Simulator"
#define VERSION INDIGO_VERSION_CURRENT

static indigo_result ccd_simulator_attach(indigo_driver *driver) {
  assert(driver != NULL);
  if (indigo_ccd_driver_attach(driver, DEVICE, VERSION) == INDIGO_OK) {
    // -------------------------------------------------------------------------------- SIMULATION
    SIMULATION_PROPERTY->perm = INDIGO_RO_PERM;
    SIMULATION_ENABLE_ITEM->switch_value = true;
    SIMULATION_DISABLE_ITEM->switch_value = false;
    // -------------------------------------------------------------------------------- CCD_INFO
    CCD_INFO_WIDTH_ITEM->number_value = 1600;
    CCD_INFO_HEIGHT_ITEM->number_value = 1200;
    CCD_INFO_MAX_HORIZONTAL_BIN_ITEM->number_value = 2;
    CCD_INFO_MAX_VERTICAL_BIN_ITEM->number_value = 2;
    CCD_INFO_PIXEL_SIZE_ITEM->number_value = 5.2;
    CCD_INFO_PIXEL_WIDTH_ITEM->number_value = 5.2;
    CCD_INFO_PIXEL_HEIGHT_ITEM->number_value = 5.2;
    CCD_INFO_BITS_PER_PIXEL_ITEM->number_value = 16;
    // -------------------------------------------------------------------------------- CCD_FRAME
    CCD_FRAME_WIDTH_ITEM->number_max = CCD_FRAME_WIDTH_ITEM->number_value = CCD_INFO_WIDTH_ITEM->number_value;
    CCD_FRAME_HEIGHT_ITEM->number_max = CCD_FRAME_HEIGHT_ITEM->number_value = CCD_INFO_HEIGHT_ITEM->number_value;
    // -------------------------------------------------------------------------------- CCD_IMAGE
    int blob_size = 2 * CCD_INFO_WIDTH_ITEM->number_value * CCD_INFO_HEIGHT_ITEM->number_value;
    char *blob = malloc(blob_size);
    if (blob == NULL)
      return INDIGO_FAILED;
    CCD_IMAGE_ITEM->blob_size = blob_size;
    strncpy(CCD_IMAGE_ITEM->blob_format, ".bin", INDIGO_NAME_SIZE);
    CCD_IMAGE_ITEM->blob_value = blob;
    return INDIGO_OK;
  }
  return INDIGO_FAILED;
}

static void exposure_timer_callback(indigo_driver *driver, int timer_id, void *data) {
  if (CCD_EXPOSURE_PROPERTY->state == INDIGO_BUSY_STATE) {
    CCD_EXPOSURE_PROPERTY->state = INDIGO_OK_STATE;
    CCD_EXPOSURE_ITEM->number_value = 0;
    indigo_update_property(driver, CCD_EXPOSURE_PROPERTY, "Exposure done");
    long size = CCD_IMAGE_ITEM->blob_size;
    char *blob = CCD_IMAGE_ITEM->blob_value;
    for (int i = 0; i < size; i++)
      blob[i] = rand();
    CCD_IMAGE_PROPERTY->state = INDIGO_OK_STATE;
    indigo_update_property(driver, CCD_IMAGE_PROPERTY, NULL);
  }
}

static indigo_result ccd_simulator_change_property(indigo_driver *driver, indigo_client *client, indigo_property *property) {
  assert(driver != NULL);
  assert(driver->driver_context != NULL);
  assert(property != NULL);
  if (indigo_property_match(CONNECTION_PROPERTY, property)) {
    // -------------------------------------------------------------------------------- CONNECTION
    indigo_property_copy_values(CONNECTION_PROPERTY, property, false);
    CONNECTION_PROPERTY->state = INDIGO_OK_STATE;
  } else if (indigo_property_match(CONFIGURATION_PROPERTY, property)) {
    // -------------------------------------------------------------------------------- CONFIG_PROCESS
    indigo_property_copy_values(CONFIGURATION_PROPERTY, property, false);
  } else if (indigo_property_match(CCD_EXPOSURE_PROPERTY, property)) {
    // -------------------------------------------------------------------------------- CCD_EXPOSURE
    indigo_property_copy_values(CCD_EXPOSURE_PROPERTY, property, false);
    CCD_EXPOSURE_PROPERTY->state = INDIGO_BUSY_STATE;
    indigo_update_property(driver, CCD_EXPOSURE_PROPERTY, "Exposure initiated");
    CCD_IMAGE_PROPERTY->state = INDIGO_BUSY_STATE;
    indigo_update_property(driver, CCD_IMAGE_PROPERTY, NULL);
    indigo_set_timer(driver, 2, NULL, CCD_EXPOSURE_ITEM->number_value, exposure_timer_callback);
  } else if (indigo_property_match(CCD_ABORT_EXPOSURE_PROPERTY, property)) {
    // -------------------------------------------------------------------------------- CCD_ABORT_EXPOSURE
    if (CCD_ABORT_EXPOSURE_PROPERTY->state == INDIGO_BUSY_STATE) {
      indigo_cancel_timer(driver, 2);
    }
    indigo_property_copy_values(CCD_ABORT_EXPOSURE_PROPERTY, property, false);
  }
  return indigo_ccd_driver_change_property(driver, client, property);
}

indigo_driver *ccd_simulator() {
  static indigo_driver driver_template = {
    NULL, INDIGO_OK, INDIGO_VERSION_CURRENT,
    ccd_simulator_attach,
    indigo_ccd_driver_enumerate_properties,
    ccd_simulator_change_property,
    indigo_ccd_driver_detach
  };
  indigo_driver *driver = malloc(sizeof(indigo_driver));
  if (driver != NULL) {
    memcpy(driver, &driver_template, sizeof(indigo_driver));
  }
  return driver;
}

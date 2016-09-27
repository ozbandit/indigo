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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <pthread.h>
#include <assert.h>

#include "indigo_driver.h"

indigo_result indigo_driver_attach(indigo_driver *driver, char *device, int version, int interface) {
  assert(driver != NULL);
  assert(device != NULL);
  if (driver->driver_context == NULL)
    driver->driver_context = malloc(sizeof(indigo_driver_context));
  indigo_driver_context *driver_context = driver->driver_context;
  if (driver_context != NULL) {
    // CONNECTION
    indigo_property *connection_property = indigo_init_switch_property(NULL, device, "CONNECTION", "Main", "Connection", INDIGO_IDLE_STATE, INDIGO_RW_PERM, INDIGO_ONE_OF_MANY_RULE, 2);
    if (connection_property == NULL)
      return INDIGO_FAILED;
    indigo_init_switch_item(&connection_property->items[0], "CONNECTED", "Connected", false);
    indigo_init_switch_item(&connection_property->items[1], "DISCONNECTED", "Disconnected", true);
    driver_context->connection_property = connection_property;
    // DRIVER_INFO
    indigo_property *info_property = indigo_init_text_property(NULL, device, "DRIVER_INFO", "Options", "Driver Info", INDIGO_IDLE_STATE, INDIGO_RO_PERM, 5);
    if (info_property == NULL)
      return INDIGO_FAILED;
    indigo_init_text_item(&info_property->items[0], "DRIVER_NAME", "Name", device);
    indigo_init_text_item(&info_property->items[1], "DRIVER_VERSION", "Version", "%d.%d", (version >> 8) & 0xFF, version & 0xFF);
    indigo_init_text_item(&info_property->items[2], "DRIVER_INTERFACE", "Interface", "%d", interface);
    indigo_init_text_item(&info_property->items[3], "FRAMEWORK_NAME", "Framework name", "INDIGO PoC");
    indigo_init_text_item(&info_property->items[4], "FRAMEWORK_VERSION", "Framework version", "%d.%d build %d", (INDIGO_VERSION_CURRENT >> 8) & 0xFF, INDIGO_VERSION_CURRENT & 0xFF, INDIGO_BUILD);
    driver_context->info_property = info_property;
    // DEBUG
    indigo_property *debug_property = indigo_init_switch_property(NULL, device, "DEBUG", "Options", "Debug", INDIGO_IDLE_STATE, INDIGO_RW_PERM, INDIGO_ONE_OF_MANY_RULE, 2);
    if (debug_property == NULL)
      return INDIGO_FAILED;
    indigo_init_switch_item(&debug_property->items[0], "ENABLE", "Enable", false);
    indigo_init_switch_item(&debug_property->items[1], "DISABLE", "Disable", true);
    driver_context->debug_property = debug_property;
    // SIMULATION
    indigo_property *simulation_property = indigo_init_switch_property(NULL, device, "SIMULATION", "Options", "Simulation", INDIGO_IDLE_STATE, INDIGO_RW_PERM, INDIGO_ONE_OF_MANY_RULE, 2);
    if (simulation_property == NULL)
      return INDIGO_FAILED;
    indigo_init_switch_item(&simulation_property->items[0], "ENABLE", "Enable", false);
    indigo_init_switch_item(&simulation_property->items[1], "DISABLE", "Disable", true);
    driver_context->simulation_property = simulation_property;
    // CONFIG_PROCESS
    indigo_property *congfiguration_property = indigo_init_switch_property(NULL, device, "CONFIG_PROCESS", "Options", "Configuration", INDIGO_IDLE_STATE, INDIGO_RW_PERM, INDIGO_ONE_OF_MANY_RULE, 3);
    if (congfiguration_property == NULL)
      return INDIGO_FAILED;
    indigo_init_switch_item(&congfiguration_property->items[0], "CONFIG_LOAD", "Load", false);
    indigo_init_switch_item(&congfiguration_property->items[1], "CONFIG_SAVE", "Save", false);
    indigo_init_switch_item(&congfiguration_property->items[2], "CONFIG_DEFAULT", "Default", false);
    driver_context->congfiguration_property = congfiguration_property;
    driver->driver_context = driver_context;
    return INDIGO_OK;
  }
  return INDIGO_FAILED;
}

indigo_result indigo_driver_enumerate_properties(indigo_driver *driver, indigo_client *client, indigo_property *property) {
  assert(driver != NULL);
  assert(property != NULL);
  indigo_driver_context *driver_context = driver->driver_context;
  assert(driver_context != NULL);
  if (indigo_property_match(driver_context->connection_property, property))
    indigo_define_property(driver, driver_context->connection_property);
  if (indigo_property_match(driver_context->info_property, property))
    indigo_define_property(driver, driver_context->info_property);
  if (indigo_property_match(driver_context->debug_property, property))
    indigo_define_property(driver, driver_context->debug_property);
  if (indigo_property_match(driver_context->simulation_property, property))
    indigo_define_property(driver, driver_context->simulation_property);
  if (indigo_property_match(driver_context->congfiguration_property, property))
    indigo_define_property(driver, driver_context->congfiguration_property);
  return INDIGO_OK;
}

indigo_result indigo_driver_change_property(indigo_driver *driver, indigo_client *client, indigo_property *property) {
  assert(driver != NULL);
  assert(client != NULL);
  assert(property != NULL);
  indigo_driver_context *driver_context = (indigo_driver_context *)driver->driver_context;
  assert(driver_context != NULL);
  if (indigo_property_match(driver_context->connection_property, property)) {
    // CONNECTION
    if (driver_context->connection_property->items[0].switch_value) {
      driver_context->connection_property->state = INDIGO_OK_STATE;
    } else {
      driver_context->connection_property->state = INDIGO_IDLE_STATE;
    }
    indigo_update_property(driver, driver_context->connection_property);
  } else if (indigo_property_match(driver_context->debug_property, property)) {
    // DEBUG
    indigo_property_copy_values(driver_context->debug_property, property);
    indigo_update_property(driver, driver_context->debug_property);
  } else if (indigo_property_match(driver_context->simulation_property, property)) {
    // SIMULATION
    indigo_property_copy_values(driver_context->simulation_property, property);
    indigo_update_property(driver, driver_context->simulation_property);
  } else if (indigo_property_match(driver_context->congfiguration_property, property)) {
    // CONFIG_PROCESS
    if (driver_context->congfiguration_property->items[0].switch_value) {
      // TBD
      driver_context->congfiguration_property->items[0].switch_value = false;
      driver_context->congfiguration_property->state = INDIGO_OK_STATE;
    } else if (driver_context->congfiguration_property->items[1].switch_value) {
      if (driver_context->debug_property->perm == INDIGO_RW_PERM)
        indigo_save_property(driver_context->debug_property);
      if (driver_context->simulation_property->perm == INDIGO_RW_PERM)
        indigo_save_property(driver_context->simulation_property);
      if (indigo_save_properties(driver) == INDIGO_OK)
        driver_context->congfiguration_property->state = INDIGO_OK_STATE;
      else
        driver_context->congfiguration_property->state = INDIGO_ALERT_STATE;
      driver_context->congfiguration_property->items[1].switch_value = false;
    } else if (driver_context->congfiguration_property->items[2].switch_value) {
      // TBD
      driver_context->congfiguration_property->items[2].switch_value = false;
      driver_context->congfiguration_property->state = INDIGO_OK_STATE;
    }
    indigo_update_property(driver, driver_context->congfiguration_property);
  }
  return INDIGO_OK;
}

indigo_result indigo_driver_detach(indigo_driver *driver) {
  return INDIGO_OK;
}

static pthread_mutex_t save_mutex = PTHREAD_RECURSIVE_MUTEX_INITIALIZER;
static indigo_property *save_properties[INDIGO_MAX_PROPERTIES];
static int save_property_count = 0;

static void xprintf(int output, const char *format, ...) {
  char buffer[1024];
  va_list args;
  va_start(args, format);
  int length = vsnprintf(buffer, 1024, format, args);
  va_end(args);
  write(output, buffer, length);
  INDIGO_DEBUG(indigo_debug("saved: %s", buffer));
}

void indigo_save_property(indigo_property *property) {
  if (save_property_count < INDIGO_MAX_PROPERTIES) {
    INDIGO_LOCK(&save_mutex);
    save_properties[save_property_count++] = property;
  }
}

indigo_result indigo_save_properties(indigo_driver *driver) {
  INDIGO_LOCK(&save_mutex);
  int output = 2;  
  for (int i = 0; i < save_property_count; i++) {
    indigo_property *property = save_properties[i];
    switch (property->type) {
      case INDIGO_TEXT_VECTOR:
        xprintf(output, "<newTextVector device='%s' name='%s'>\n", property->device, property->name, indigo_property_state_text[property->state]);
        for (int i = 0; i < property->count; i++) {
          indigo_item *item = &property->items[i];
          xprintf(output, "<oneText name='%s'>%s</oneText>\n", item->name, item->text_value);
        }
        xprintf(output, "</newTextVector>\n");
        break;
      case INDIGO_NUMBER_VECTOR:
        xprintf(output, "<newNumberVector device='%s' name='%s'>\n", property->device, property->name, indigo_property_state_text[property->state]);
        for (int i = 0; i < property->count; i++) {
          indigo_item *item = &property->items[i];
          xprintf(output, "<oneNumber name='%s'>%g</oneNumber>\n", item->name, item->number_value);
        }
        xprintf(output, "</newNumberVector>\n");
        break;
      case INDIGO_SWITCH_VECTOR:
        xprintf(output, "<newSwitchVector device='%s' name='%s'>\n", property->device, property->name, indigo_property_state_text[property->state]);
        for (int i = 0; i < property->count; i++) {
          indigo_item *item = &property->items[i];
          xprintf(output, "<oneSwitch name='%s'>%s</oneSwitch>\n", item->name, item->switch_value ? "On" : "Off");
        }
        xprintf(output, "</newSwitchVector>\n");
        break;
      default:
        break;
    }
    INDIGO_UNLOCK(&save_mutex);
  }
  INDIGO_UNLOCK(&save_mutex);
  return INDIGO_OK;
}


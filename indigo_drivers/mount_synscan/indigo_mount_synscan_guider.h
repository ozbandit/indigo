//
//  indigo_mount_synscan_guider.h
//  indigo_server
//
//  Created by David Hulse on 21/08/2018.
//  Copyright © 2018 CloudMakers, s. r. o. All rights reserved.
//

#ifndef indigo_mount_synscan_guider_h
#define indigo_mount_synscan_guider_h

#include "indigo_mount_synscan_private.h"

extern indigo_result guider_attach(indigo_device *device);
extern indigo_result guider_enumerate_properties(indigo_device *device, indigo_client *client, indigo_property *property);
extern indigo_result guider_change_property(indigo_device *device, indigo_client *client, indigo_property *property);
extern indigo_result guider_detach(indigo_device *device);

#endif /* indigo_mount_synscan_guider_h */

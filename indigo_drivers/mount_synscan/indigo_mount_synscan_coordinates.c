//
//  indigo_mount_synscan_coordinates.c
//  indigo_server
//
//  Created by David Hulse on 23/08/2018.
//  Copyright © 2018 CloudMakers, s. r. o. All rights reserved.
//

#include <assert.h>
#include <math.h>
#include "indigo_mount_driver.h"
#include "indigo_mount_synscan_private.h"
#include "indigo_mount_synscan_coordinates.h"


double ha_steps_to_position(indigo_device* device, AxisPosition steps) {
	double hapos = (double)(steps - PRIVATE_DATA->raZeroPos) / (double)PRIVATE_DATA->raTotalSteps;
	if (hapos < 0)
		hapos += 1;
	return hapos;
}

double dec_steps_to_position(indigo_device* device, AxisPosition steps) {
	double decpos = (double)(steps - PRIVATE_DATA->decZeroPos) / (double)PRIVATE_DATA->decTotalSteps;
	if (decpos < 0)
		decpos += 1;
	return decpos;
}

AxisPosition ha_position_to_steps(indigo_device* device, double position) {
	if (position > 0.75)
		position -= 1.0;
	return lrint(PRIVATE_DATA->raZeroPos + (position * PRIVATE_DATA->raTotalSteps));
}

AxisPosition dec_position_to_steps(indigo_device* device, double position) {
	if (position > 0.75)
		position -= 1.0;
	return lrint(PRIVATE_DATA->decZeroPos + (position * PRIVATE_DATA->decTotalSteps));
}

void coords_encoder_to_eq(indigo_device* device, double ha_enc, double dec_enc, double* ha, double* dec) {
	//  Get hemisphere
	bool south = MOUNT_GEOGRAPHIC_COORDINATES_LATITUDE_ITEM->number.value < 0;

	//  NORTHERN HEMISPHERE
	//  Convert to Declination in range [-90, +90] with WEST flag
	//
	//  DEG:
	//      [0, 90]         dec = deg,          west = false
	//      [90, 180]       dec = 180 - deg,    west = true
	//      [180, 270]      dec = 180 - deg,    west = true
	//      [270, 360]      dec = deg - 360,    west = false
	//
	//
	//  SOUTHERN HEMISPHERE
	//  Convert to Declination in range [-90, +90] with WEST flag
	//
	//  DEG:
	//      [0, 90]         dec = -deg,         west = true
	//      [90, 180]       dec = deg - 180,    west = false
	//      [180, 270]      dec = deg - 180,    west = false
	//      [270, 360]      dec = 360 - deg,    west = true

	//  Normal DEC
	double epdec = dec_enc;
	if (epdec < 0)
		epdec += 1.0;

	bool west = false;
	if (epdec < 0.25)
		*dec = epdec;
	else if (epdec < 0.75) {
		*dec = 0.5 - epdec;
		west = true;
	}
	else // decPos in [0.75, 1.0]
		*dec = epdec - 1.0;

	//  Adjust for southern hemisphere
	if (south) {
		west = !west;
		*dec = -*dec;
	}
	*dec *= 2.0 * M_PI;
	//eq->west = west;


	//  NORTHERN HEMISPHERE
	//  Convert to HA
	//      HA represents the time since transitting the meridian.
	//      Thus:
	//          [0, 12]     Object is west of meridian
	//          [-12, 0]    Object is east of meridian and will transit in -ha hours
	//
	//  HA Hours:
	//      [0, 6) W        HA = Hours
	//      [0, 6) E        HA = Hours - 12
	//      [6, 12) W       HA = Hours
	//      [6, 12) E       HA = Hours - 12
	//      [12, 18) W      HA = Hours - 24
	//      [12, 18) E      HA = Hours - 12
	//      [18, 24] W      HA = Hours - 24
	//      [18, 24] E      HA = Hours - 12
	//
	//  In practice, the mount cannot position itself with HA Hours > 12 because the counterweights would be up
	//  and the scope would be in danger of hitting the mount. With care (i.e. depending on the DEC), we can allow
	//  HA Hours > 12. However, there is a band of HA Hours centred on 18 that we cannot position to at all. We
	//  can represent this band as 18+/-N, where N is half the width of the band (i.e. [16,20] would mean N=2).
	//
	//
	//  SOUTHERN HEMISPHERE
	//  Convert to HA
	//      HA represents the time since transitting the meridian.
	//      Thus:
	//          [0, 12]     Object is east of meridian
	//          [-12, 0]    Object is west of meridian and will transit in -ha hours
	//
	//  HA Hours:
	//      [6, 0] W        HA = 12 - Hours
	//      [6, 0] E        HA = -Hours
	//      [12, 6] W       HA = 12 - Hours
	//      [12, 6] E       HA = -Hours
	//      [18, 12] W      HA = 12 - Hours
	//      [18, 12] E      HA = 24 - Hours
	//      [24, 18] W      HA = 12 - Hours
	//      [24, 18] E      HA = 24 - Hours

	if (!south) {
		if (!west)
			*ha = ha_enc - 0.5;
		else if (ha_enc < 0.5)
			*ha = ha_enc;
		else
			*ha = ha_enc - 1.0;
	}
	else {
		if (west)
			*ha = 0.5 - ha_enc;
		else if (ha_enc < 0.5)
			*ha = -ha_enc;
		else
			*ha = 1.0 - ha_enc;
	}
	if (*ha < -0.5)
		*ha += 1.0;
	if (*ha >= 0.5)
		*ha -= 1.0;

	*ha *= 2.0 * M_PI;
}

void coords_eq_to_encoder2(indigo_device* device, double ha, double dec, double haPos[], double decPos[]) {
	//  To do a slew, we have to decide which side of the meridian we'd like to be. Normally that would be
	//  the side that gives maximum time before a meridian flip would be required. If the object has passed
	//  the meridian already, then it is setting and we won't need to flip. If it is approaching the meridian,
	//  we could consider performing an early meridian flip if it is close enough, otherwise ...

	//  Suppose we've decided that DEC should slew eastwards from HOME to the DEC coordinate. How do we decide
	//  where to slew RA?
	//
	//  If we've done our eastward DEC slew from home, the scope is pointing at LST + 6 hours.
	//  If we slew clockwise in RA for 6 hours, we'd be pointing at LST+12, which might be below horizon if DEC < 90 - LAT.
	//  If we slew anti-clockwise for 6 hours, we'd be pointing at LST and needing to do a meridian flip.
	//
	//  If we take that meridian flip, we're still pointing at LST, but now the DEC slew is westward from home.
	//  If we slew anti-clockwise 12 hours, we'd be pointing at LST-12, which might be below horizon if DEC < 90 - LAT.
	//
	//  With an eastward DEC slew, we can point at targets between LST and LST+12.
	//  With a  westward DEC slew, we can point at targets between LST and LST-12.
	//  The east/west sense is reversed for southern hemisphere.

	//  Get hemisphere
	bool south = MOUNT_GEOGRAPHIC_COORDINATES_LATITUDE_ITEM->number.value < 0;

	//  Convert DEC to Degrees of encoder rotation relative to 0 position
	//  [-90,90]
	//
	//  NORTHERN HEMISPHERE
	//
	//  DEC 90              Degrees = 90 W or 90 E
	//  DEC 75              Degrees = 105 W or 75 E
	//  DEC 45              Degrees = 135 W or 45 E
	//  DEC 20              Degrees = 160 W or 20 E
	//  DEC 0               Degrees = 180 W or 0 E
	//  DEC -20             Degrees = 200 W or 340 E
	//  DEC -45             Degrees = 225 W or 315 E
	//  DEC -75             Degrees = 255 W or 285 E
	//  DEC -90             Degrees = 270
	//
	//
	//  SOUTHERN HEMISPHERE
	//
	//  DEC -90             Degrees = 90
	//  DEC -75             Degrees = 105 E or 75 W
	//  DEC -45             Degrees = 135 E or 45 W
	//  DEC -20             Degrees = 160 E or 20 W
	//  DEC 0               Degrees = 180 E or 0 W
	//  DEC  20             Degrees = 200 E or 340 W
	//  DEC  45             Degrees = 225 E or 315 W
	//  DEC  75             Degrees = 255 E or 285 W
	//  DEC  90             Degrees = 270

	//  Generate the two DEC positions (east and west of meridian)
	double degw = M_PI - dec;   //  NORTH variant by default
	if (south) {
		if (dec < 0)
			degw = -dec;
		else
			degw = M_PI + M_PI - dec;
	}

	double dege = M_PI + dec;   //  SOUTH variant by default
	if (!south) {
		if (dec < 0)
			dege = M_PI + M_PI + dec;
		else
			dege = dec;
	}

	//  Rebase the angles to optimise the DEC slew relative to HOME
	if (degw > M_PI + M_PI_2)
		degw -= M_PI + M_PI;
	if (dege > M_PI + M_PI_2)
		dege -= M_PI + M_PI;
	degw /= M_PI + M_PI;
	dege /= M_PI + M_PI;

	//  HA represents hours since the transit of the object
	//  < 0 means object has yet to transit
	//
	//  convert to [-12,12]
	if (ha > M_PI)
		ha -= M_PI + M_PI;
	if (ha < -M_PI)
		ha += M_PI + M_PI;
	assert(ha >= -M_PI && ha <= M_PI);

	//  Compute HA positions to match each DEC position
	double haw;
	if (ha >= 0)
		haw = ha;
	else
		haw = ha + M_PI + M_PI;
	if (south)
		haw = M_PI - ha;

	double hae = ha + M_PI;
	if (south) {
		if (ha >= 0)
			hae = M_PI + M_PI - ha;
		else
			hae = -ha;
	}
	haw /= M_PI + M_PI;
	hae /= M_PI + M_PI;

	assert(haw < 0.5 || hae < 0.5);

	//  Decide whether EAST or WEST provides the "normal" / CW-Down slew and fill in the positions
	if (haw < 0.5) {
		haPos[0] = haw;
		decPos[0] = degw;
		haPos[1] = hae;
		decPos[1] = dege;
	}
	else {
		haPos[0] = hae;
		decPos[0] = dege;
		haPos[1] = haw;
		decPos[1] = degw;
	}

	printf("SOLUTIONS:\n");
	printf("  OK:  %g,   %g\n", haPos[0], decPos[0]);
	printf("  UP:  %g,   %g\n", haPos[1], decPos[1]);
}

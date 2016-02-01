/*
 * Copyright (C) Roland
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/follow_me/follow_me.c"
 * @author Roland
 * follows a person on the stereo histogram image.
 * It searches for the highest peak and adjusts its roll and pitch to hover at a nice distance.
 */

#include "modules/stereocam/stereocam_forward_velocity_optitrack/stereocam_forward_velocity_optitrack.h"
#include "modules/stereocam/stereocam.h"
#include "state.h"
#include "navigation.h"
#include "subsystems/abi.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "subsystems/datalink/telemetry.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_int.h"
// Know waypoint numbers and blocks
#include "generated/flight_plan.h"


#define CENTER_STEREO_IMAGE_VERTICAL 48 //96/2
#define CENTER_STEREO_IMAGE_HORIZONTAL 50 //???
#define HEIGHT_ACT_DIFF 15
#define HORIZONTAL_ACT_DIFF 15
#define VERTICAL_FOV 0.1 // In Radians
#define HORIZONTAL_FOV 0.1 // In Radians
typedef float distance_meters;
distance_meters preferredSelfieDistance=0.5;

void increase_nav_heading(int32_t *headingToChange, int32_t increment);
void increase_nav_heading(int32_t *headingToChange, int32_t increment)
{
  *headingToChange = *headingToChange + increment;
}

void stereocam_selfie_init()
{

}
void stereocam_selfie_periodic()
{
  if (stereocam_data.fresh) {
    stereocam_data.fresh = 0;

    uint8_t meanX = stereocam_data.data[0];// = (uint8_t) mean_X;
    uint8_t meanY = stereocam_data.data[1];// = (uint8_t) min_y;
    int distanceCM = 2*stereocam_data.data[2];// = (uint8_t) distance;... divided by 2

    // Height:
	// Check if the height is correct
	if(abs(meanY-CENTER_STEREO_IMAGE_VERTICAL)>HEIGHT_ACT_DIFF){
		selfie_alt+=(placeY-CENTER_STEREO_IMAGE_VERTICAL)*VERTICAL_FOV;
	}

	if(abs(placeX-CENTER_STEREO_IMAGE_HORIZONTAL)>HORIZONTAL_ACT_DIFF){
		// Calculate the yaw and... yaw
		moveWaypointAroundMe(followWaypoint, (placeX-CENTER_STEREO_IMAGE_HORIZONTAL),followWaypoint);
	}
	else{ // Calculate the distance and go to the person
		float distance = disparityToDistance(disparity);
		setWaypointAtDistance(distance,preferredSelfieDistance,followWaypoint);
	}
  }
}

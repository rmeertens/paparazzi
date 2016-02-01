/*
 * Copyright (C) Roland
 *
 * This file is part of paparazzi
 *
 */
/**
 *
 * @author Roland
 * follows based on stereo
 */

#ifndef FOLLOW_ME_H
#define FOLLOW_ME_H
#include <inttypes.h>

// Know waypoint numbers and blocks
#include "generated/flight_plan.h"

extern float ref_alt;
extern void stereocam_selfie_init(void);
extern void stereocam_selfie_periodic(void);

#endif


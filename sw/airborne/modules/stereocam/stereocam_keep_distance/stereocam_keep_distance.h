/*
 * Copyright (C) Roland
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/stereocam/stereocam_keep_distance/stereocam_keep_distance.h"
 * @author Roland
 * keeps a distance based on stereo
 */


#ifndef FOLLOW_ME_H
#define FOLLOW_ME_H
#include "math/pprz_algebra_int.h"

extern float ref_pitch;
extern float ref_roll;
extern float ref_alt;

extern float ref_disparity_to_keep;
extern void stereocam_forward_velocity_init(void);
extern void stereocam_forward_velocity_periodic(void);

// Implement own Horizontal loops
extern void guidance_h_module_init(void);
extern void guidance_h_module_enter(void);
extern void guidance_h_module_read_rc(void);
extern void guidance_h_module_run(bool in_flight);

// Implement own Horizontal loops
extern void guidance_v_module_init(void);
extern void guidance_v_module_enter(void);
extern void guidance_v_module_read_rc(void);
extern void guidance_v_module_run(bool in_flight);



#endif


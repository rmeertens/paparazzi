/*
 * Copyright (C) Roland
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/readlocationfromodroid/readlocationfromodroid.h"
 * @author Roland
 * reads from the odroid
 */

#ifndef READLOCATIONFROMODROID_H
#define READLOCATIONFROMODROID_H
#include <inttypes.h>
extern int resetOdroid;

int32_t optitrack_ecef_x;
int32_t optitrack_ecef_y;
int32_t optitrack_ecef_z;
int32_t optitrack_lat;
int32_t optitrack_lon;
int32_t optitrack_alt;

int32_t optitrack_hmsl;
int32_t optitrack_ecef_xd;
int32_t optitrack_ecef_yd;
int32_t optitrack_ecef_zd;
int32_t optitrack_course;
int32_t optitrack_numsv;
int32_t optitrack_tow;
extern void odroid_loc_init();
extern void odroid_loc_periodic();

#endif


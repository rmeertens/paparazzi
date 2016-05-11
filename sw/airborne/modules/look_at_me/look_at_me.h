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
 * @file "modules/look_at_me/look_at_me.h"
 * @author Roland
 * looks at me
 */

#ifndef LOOK_AT_ME_H
#define LOOK_AT_ME_H
#include "inttypes.h"
extern void look_at_me_init();
extern void look_at_me_periodic();
extern void set_location_to_look_at(int32_t x,int32_t y, int32_t z); // In centimeters
#endif


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
 * @file "modules/location_sender/location_sender.h"
 * @author Roland
 * 
 */

#ifndef LOCATION_SENDER_H
#define LOCATION_SENDER_H
#include <inttypes.h>
extern void send_current_location(void);
extern void parse_this(void);
extern uint8_t setWpToNextNUSDemo(uint8_t);
extern uint8_t reset_route(void);
extern uint8_t reset_write(void);
#endif


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
 * @file "modules/location_sender/location_sender.c"
 * @author Roland
 * 
 */

#include "modules/location_sender/location_sender.h"
#include <inttypes.h>

#include "subsystems/datalink/telemetry.h"
void send_current_location() {
	printf("Sending lcoation\n");
	// ROLANDLOC
	// posx
	// posy
	uint32_t posx=10;
	uint32_t posy=20;
    DOWNLINK_SEND_ROLANDLOC(DefaultChannel, DefaultDevice, &posx,&posy);

}



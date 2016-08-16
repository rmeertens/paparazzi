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
#include "state.h"
#include "subsystems/datalink/telemetry.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_orientation_conversion.h"
#define MAX_LENGTH_WAYPOINTER 1000

struct LlaCoor_i historyOtherDrone[MAX_LENGTH_WAYPOINTER];
int writeIndexOtherDrone=0;
int flyIndexOtherDrone=0;

void addPosition(struct LlaCoor_i new_pos){
	if(writeIndexOtherDrone<MAX_LENGTH_WAYPOINTER){
		  printf("adding %d %d %d\n",new_pos.lat,new_pos.lon,new_pos.alt);
		historyOtherDrone[writeIndexOtherDrone++]=new_pos;
	}
}
uint8_t setWpToNextNUSDemo(uint8_t wp_id){
	printf("Set wp to next! %d ",flyIndexOtherDrone );
	if(flyIndexOtherDrone < writeIndexOtherDrone){
	  struct LlaCoor_i c = historyOtherDrone[flyIndexOtherDrone];
	  struct LlaCoor_i *myOwnPos = stateGetPositionLla_i();
	  printf("Going to  %d %d %d\n",c.lat,c.lon,c.alt);
	  printf("My own %d %d %d\n",myOwnPos->lat,myOwnPos->lon,myOwnPos->alt);
//	  waypoint_set_latlon(wp_id,&c);
	  waypoint_set_lla(wp_id,&c);
	  flyIndexOtherDrone++;
	 }
	return 0;
}
void send_current_location() {
	printf("Sending lcoation\n");
	if(stateGetPositionEnu_f()->z > 0.25){
	  struct LlaCoor_i *c = stateGetPositionLla_i();
	  int32_t la = c->lat;
	  int32_t lo = c->lon;
	  int32_t al = c->alt;

	  DOWNLINK_SEND_ROLANDLOC(DefaultChannel, DefaultDevice, &la,&lo,&al);
	}
}
uint8_t reset_route(){
	flyIndexOtherDrone=0;
	return false;
}

uint8_t reset_write(){
	writeIndexOtherDrone=0;
	return false;
}
void parse_this(){
	printf("Parsing this");
	struct LlaCoor_i new_wp;
	  uint8_t ac_id = DL_MOVE_WP_ac_id(dl_buffer);

	  uint8_t wp_id = DL_MOVE_WP_wp_id(dl_buffer);

	  new_wp.lat = DL_MOVE_WP_lat(dl_buffer);

	  new_wp.lon = DL_MOVE_WP_lon(dl_buffer);

	  new_wp.alt = DL_MOVE_WP_alt(dl_buffer);
	  addPosition(new_wp);

/*	  uint8_t leader = DL_FORMATION_STATUS_leader_id(dl_buffer);
	  uint8_t status = DL_FORMATION_STATUS_status(dl_buffer);
	  if (ac_id == AC_ID) { leader_id = leader; }
	  else if (leader == leader_id) { updateFormationStatus(ac_id, status); }
	  else { updateFormationStatus(ac_id, UNSET); }
	}
*/
}



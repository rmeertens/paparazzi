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
 * @file "modules/look_at_me/look_at_me.c"
 * @author Roland
 * looks at me
 */

#include "modules/look_at_me/look_at_me.h"
#include "state.h"
#include "navigation.h"
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "subsystems/navigation/traffic_info.h"
float diff_angle_rad=0.0;

float diff_angle_rad_vertical=0.0;

// Function
struct image_t* paint_object_func(struct image_t* img);
struct image_t* paint_object_func(struct image_t* img)
{
	struct point_t from;
	struct point_t to;
	from.y=0;
	to.y=img->h;
	float viewingAngle=2.618; // 150 degrees
	float viewingAngleVertical=2.318; // ??? degrees
	float diff_abs = fabs(diff_angle_rad);
	printf("\n\nDiff angle: %f. Vertical: %f\n\n",diff_angle_rad,diff_angle_rad_vertical);
	if(diff_abs<viewingAngle/2){
		if(fabs(diff_angle_rad_vertical)<viewingAngleVertical/2){
			from.x=img->w/2+(img->w/2)*(diff_angle_rad/(viewingAngle/2));
			to.x=from.x;


			from.y=img->h/2+(img->h/2)*(diff_angle_rad_vertical/(viewingAngleVertical/2))+img->h/8;
			to.y=img->h/2+(img->h/2)*(diff_angle_rad_vertical/(viewingAngleVertical/2))-img->h/8;
			image_draw_line(img,&from,&to);
		}
	}
  return img; // Colorfilter did not make a new image
}
void look_at_me_init() {
	cv_add(paint_object_func);

}
void look_at_me_periodic() {}


void set_location_to_look_at(int32_t x,int32_t y,int32_t z){
	  struct EcefCoor_i *pos = stateGetPositionEcef_i(); // Get your current position

	printf("Getting location. Diff: %d %d %d \n", pos->x-x, pos->y-y, pos->z - z);

	printf("Getting location. Own z: %d Pocket: %d \n", pos->z , z);

/*	bool nav_set_heading_rad(float rad)
	{
	  nav_heading = ANGLE_BFP_OF_REAL(rad);
	  INT32_COURSE_NORMALIZE(nav_heading);
	  return false;
	}

	bool nav_set_heading_deg(float deg)
	{
	  return nav_set_heading_rad(RadOfDeg(deg));
	}*/
//	if(pos->x-x>0){
//		nav_set_heading_rad(0.2);
//	}
//	else{
//		nav_set_heading_rad(3.4);
//	}

    float heading_f = atan2f(pos->x-x,pos->y-y);
    nav_set_heading_rad(heading_f-3.1415926/2);
    float wanted_angle= ANGLE_FLOAT_OF_BFP(nav_heading);
    float current_angle = ANGLE_FLOAT_OF_BFP(stateGetNedToBodyEulers_i()->psi);

    float current_angle_pitch = ANGLE_FLOAT_OF_BFP(stateGetNedToBodyEulers_i()->theta);//looking down -> minus theta
	float distance_to_pocketdrone = sqrt((pos->x-x)*(pos->x-x)+(pos->y-y)*(pos->y-y));
	float z_diff = pos->z-z;
	printf("calculating atan of z_diff %f and distance %f \n",z_diff,distance_to_pocketdrone);


//
//	struct ac_info_ {
//	  uint8_t ac_id;
//	  float east;   ///< m relative to nav_utm_east0
//	  float north;  ///< m relative to nav_utm_north0
//	  float course; ///< rad (CW)
//	  float alt;    ///< m above Mean Sea Level (geoid)
//	  float gspeed; ///< m/s
//	  float climb;  ///< m/s
//	  uint32_t itow;///< ms
//	};
//
//	extern uint8_t acs_idx;
//	extern uint8_t the_acs_id[NB_ACS_ID];
//	extern struct ac_info_ the_acs[NB_ACS];
//
//	extern void traffic_info_init(void);
//	extern struct ac_info_ *get_ac_info(uint8_t id);

//	struct ac_info_* knowWhere = get_ac_info(164);

	  struct ac_info_ * ac = get_ac_info(164);
	printf("Know the lady lisa is at %f\n",ac->east);


	diff_angle_rad_vertical=atan2f(z_diff,distance_to_pocketdrone);
	diff_angle_rad_vertical+=current_angle_pitch;

    if(current_angle<0){
    	current_angle+=2.*3.1415926535897932384626433832795029;
    }
    /*
    if(current_angle < wanted_angle){
    	printf("Current smaller than wanted\n");
//    	diff_angle_rad = wanted_angle-current_angle;

    	//diff_angle_rad = (3.1415926*2)-current_angle+wanted_angle;
    }
    else{

    	printf("wanted smaller than current\n");
    	diff_angle_rad = (3.1415926*2)-current_angle+wanted_angle;
    }*/
    diff_angle_rad = wanted_angle-current_angle;
    if(diff_angle_rad>3.1415926){
    	diff_angle_rad-=(3.1415926*2);
    }

    printf("Current wanted heading %f actual: %f \n ",wanted_angle, current_angle);
}



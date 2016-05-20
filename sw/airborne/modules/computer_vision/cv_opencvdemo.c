/*
 * Copyright (C) C. De Wagter
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
 * @file "modules/computer_vision/cv_opencvdemo.c"
 * @author C. De Wagter
 * opencv
 */

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/cv_opencvdemo.h"
#include "modules/computer_vision/opencv_example.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/downlink.h"
#include "math/pprz_algebra_int.h"

#include "firmwares/rotorcraft/navigation.h"
// Function
int opencv_func(struct image_t* img);
int opencv_func(struct image_t* img)
{

  if (img->type == IMAGE_YUV422)
  {
    // Call OpenCV (C++ from paparazzi C function)
    opencv_example((char*) img->buf, img->w, img->h);
  }
  return FALSE;
}


/**
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(int32_t *heading, int32_t increment)
{
  *heading = *heading + increment;
  // Check if your turn made it go out of bounds...
  INT32_ANGLE_NORMALIZE(*heading); // HEADING HAS INT32_ANGLE_FRAC....
  return false;
}


/*
bool nav_set_heading_rad(float rad)
{
  nav_heading = ANGLE_BFP_OF_REAL(rad);
  INT32_COURSE_NORMALIZE(nav_heading);
  return false;
}

bool nav_set_heading_deg(float deg)
{
  return nav_set_heading_rad(RadOfDeg(deg));
}*/

float currentHeadingRad=0.0;
int previousCounter=0;
void opencvdemo_periodic(void)
{

	  //printf("Tracking now? %d at point %d %d\n",selfie_var.trackingNow,selfie_var.trackingX,selfie_var.trackingY);
	  if(selfie_var.trackingNow && selfie_var.trackingNumber!=previousCounter){
		  if(selfie_var.trackingPercentageX<0.5){
			  currentHeadingRad+=0.1;
		  }
		  else{
			  currentHeadingRad-=0.1;
		  }
		  nav_set_heading_rad(currentHeadingRad);
	  }

	  if(!selfie_var.trackingNow){
		  currentHeadingRad = ANGLE_FLOAT_OF_BFP(nav_heading);
	  }
	  previousCounter=selfie_var.trackingNumber;
}

void received_start_selfie(){
	printf("Whooo. starting selfie shizzle!\n");
	int x = DL_START_SELFIE_startx(dl_buffer);
	int y = DL_START_SELFIE_starty(dl_buffer);
	int width = DL_START_SELFIE_width(dl_buffer);
	int height = DL_START_SELFIE_height(dl_buffer);
	int downsized_width = DL_START_SELFIE_downsized_width(dl_buffer);

	selfie_var.must_init=true;
	selfie_var.startx=x;
	selfie_var.starty=y;
	selfie_var.width=width;
	selfie_var.height=height;
	selfie_var.downsized_width=downsized_width;
	selfie_var.trackingNumber=0;
	selfie_var.trackingNow=false;
	selfie_var.trackingPercentageX=0.0;
	selfie_var.trackingPercentageX=0.0;

}
void opencvdemo_init(void)
{

	selfie_var.must_init=false;

  cv_add(opencv_func);
}


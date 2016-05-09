/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/computer_vision/cv.c
 *
 * Computer vision framework for onboard processing
 */

#include "cv.h"
#include <stdlib.h> // for malloc
#include "video_thread.h"
void cv_add_to_device(struct video_config_t *device, cvFunction func,struct video_settings* settings_pointer)
{
  // Initialise the device that we want our function to use
  video_thread_initialise_device(device,settings_pointer);

  // Check if we already have this listener for this device
  if (device->pointer_to_first_listener == NULL) {
    struct video_listener *new_listener = malloc(sizeof(struct video_listener));
    new_listener->next = NULL;
    new_listener->func = func;
    device->pointer_to_first_listener = new_listener;
  } else {
    struct video_listener *pointing_to = device->pointer_to_first_listener;
    while (pointing_to->next != NULL) {
      pointing_to = pointing_to->next;
    }

    // The device is not yet sending the image to this function. Add it
    struct video_listener *new_listener = malloc(sizeof(struct video_listener));
    new_listener->next = NULL;
    new_listener->func = func;
    pointing_to->next = new_listener;
  }
}

void cv_run_device(struct video_config_t *device, struct image_t *img)
{
  // For each function added to a device, run this function with the image that was taken
  struct video_listener *pointing_to = device->pointer_to_first_listener;
  while (pointing_to != NULL) {
    pointing_to->func(img);
    pointing_to = pointing_to->next;
  }

}

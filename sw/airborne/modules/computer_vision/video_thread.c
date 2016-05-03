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
 * @file modules/computer_vision/video_thread.c
 */

// Own header
#include "modules/computer_vision/video_thread.h"


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>

// Video
#include "lib/v4l/v4l2.h"
#include "lib/vision/image.h"
#include "lib/vision/bayer.h"
#include "lib/encoding/jpeg.h"

#include "mcu_periph/sys_time.h"

// include board for bottom_camera and front_camera on ARDrone2 and Bebop
#include BOARD_CONFIG

#if JPEG_WITH_EXIF_HEADER
#include "lib/exif/exif_module.h"
#endif

// Threaded computer vision
#include <pthread.h>
#include "rt_priority.h"

// Frames Per Seconds
#ifndef VIDEO_THREAD_FPS
#define VIDEO_THREAD_FPS 4
#endif
PRINT_CONFIG_VAR(VIDEO_THREAD_FPS)

// The place where the shots are saved (without slash on the end)
#ifndef VIDEO_THREAD_SHOT_PATH
#define VIDEO_THREAD_SHOT_PATH "/data/video/images"
#endif
PRINT_CONFIG_VAR(VIDEO_THREAD_SHOT_PATH)

// Main thread
static void *video_thread_function(void *data);
void video_thread_periodic(void) { }

static void video_thread_save_shot(struct video_thread_t threadToSaveShotFrom, struct image_t *img,
                                   struct image_t *img_jpeg)
{

  // Search for a file where we can write to
  char save_name[128];
  for (; threadToSaveShotFrom.shot_number < 99999; threadToSaveShotFrom.shot_number++) {
    sprintf(save_name, "%s/img_%05d.jpg", STRINGIFY(VIDEO_THREAD_SHOT_PATH), threadToSaveShotFrom.shot_number);
    // Check if file exists or not
    if (access(save_name, F_OK) == -1) {

      // Create a high quality image (99% JPEG encoded)
      jpeg_encode_image(img, img_jpeg, 99, TRUE);

#if JPEG_WITH_EXIF_HEADER
      write_exif_jpeg(save_name, img_jpeg->buf, img_jpeg->buf_size, img_jpeg->w, img_jpeg->h);
#else
      FILE *fp = fopen(save_name, "w");
      if (fp == NULL) {
        printf("[video_thread-thread] Could not write shot %s.\n", save_name);
      } else {
        // Save it to the file and close it
        fwrite(img_jpeg->buf, sizeof(uint8_t), img_jpeg->buf_size, fp);
        fclose(fp);
      }
#endif

      // We don't need to seek for a next index anymore
      break;
    }
  }
}


/**
 * Handles all the video streaming and saving of the image shots
 * This is a sepereate thread, so it needs to be thread safe!
 */
static void *video_thread_function(void *data)
{
  struct video_config_t *vid = (struct video_config_t *)data;

  struct image_t img_jpeg;
  struct image_t img_color;

  // create the images
  if (vid->filters) {
    // fixme: don't hardcode size, works for bebop front camera for now
#define IMG_FLT_SIZE 272
    image_create(&img_color, IMG_FLT_SIZE, IMG_FLT_SIZE, IMAGE_YUV422);
    image_create(&img_jpeg, IMG_FLT_SIZE, IMG_FLT_SIZE, IMAGE_JPEG);
  } else {
    image_create(&img_jpeg, vid->w, vid->h, IMAGE_JPEG);
  }

  // Start the streaming of the V4L2 device
  if (!v4l2_start_capture(vid->thread.dev)) {
    printf("[video_thread-thread] Could not start capture of %s.\n", vid->thread.dev->name);
    return 0;
  }

  // be nice to the more important stuff
  set_nice_level(10);

  // Initialize timing
  struct timespec time_now;
  struct timespec time_prev;
  clock_gettime(CLOCK_MONOTONIC, &time_prev);

  // Start streaming
  vid->thread.is_running = true;
  while (vid->thread.is_running) {

    // get time in us since last run
    clock_gettime(CLOCK_MONOTONIC, &time_now);
    uint32_t dt_us = sys_time_elapsed_us(&time_prev, &time_now);
    time_prev = time_now;

    // sleep remaining time to limit to specified fps
    uint32_t fps_period_us = (uint32_t)(1000000. / (float)vid->thread.fps);
    if (dt_us < fps_period_us) {
      usleep(fps_period_us - dt_us);
    } else {
      fprintf(stderr, "video_thread with size %d %d: desired %i fps, only managing %.1f fps\n",
              vid->w, vid->h, vid->thread.fps, 1000000.f / dt_us);
    }

    // Wait for a new frame (blocking)
    struct image_t img;
    v4l2_image_get(vid->thread.dev, &img);

    // pointer to the final image to pass for saving and further processing
    struct image_t *img_final = &img;

    // run selected filters
    if (vid->filters) {
      if (vid->filters & VIDEO_FILTER_DEBAYER) {
        BayerToYUV(&img, &img_color, 0, 0);
      }
      // use color image for further processing
      img_final = &img_color;
    }

    // Check if we need to take a shot
    if (vid->thread.take_shot) {
      video_thread_save_shot(vid->thread, img_final, &img_jpeg);
      vid->thread.take_shot = false;
    }

    // Run processing if required
    cv_run_device(vid, img_final);

    // Free the image
    v4l2_image_free(vid->thread.dev, &img);
  }

  image_free(&img_jpeg);
  image_free(&img_color);

  return 0;
}

/**
 * Initialize the view video
 */
void video_thread_init(void)
{
  // Initialise the front camera
  struct video_config_t *vid = (struct video_config_t *) & (front_camera);
  vid->thread.fps = VIDEO_THREAD_FPS;
  // Initialize the V4L2 subdevice if needed
  if (vid->subdev_name != NULL) {
    // FIXME! add subdev format to config, only needed on bebop front camera so far
    if (!v4l2_init_subdev(vid->subdev_name, 0, 0, V4L2_MBUS_FMT_SGBRG10_1X10, vid->w, vid->h)) {
      printf("[video_thread] Could not initialize the %s subdevice.\n", vid->subdev_name);
      return;
    }

  }
  // Initialize the V4L2 device
  vid->thread.dev = v4l2_init(vid->dev_name, vid->w, vid->h, vid->buf_cnt, vid->format);
  if (vid->thread.dev == NULL) {
    printf("[video_thread] Could not initialize the %s V4L2 device.\n", vid->dev_name);
    return;
  }

  // Create the shot directory for the front camera
  char save_name[128];
  sprintf(save_name, "mkdir -p %s", STRINGIFY(VIDEO_THREAD_SHOT_PATH));
  if (system(save_name) != 0) {
    printf("[video_thread] Could not create shot directory %s.\n", STRINGIFY(VIDEO_THREAD_SHOT_PATH));
    return;
  }


  // Initialise the bottom camera
  struct video_config_t *vid2 = (struct video_config_t *) & (bottom_camera);
  vid2->thread.fps = VIDEO_THREAD_FPS;
  // Initialize the V4L2 subdevice if needed
  if (vid2->subdev_name != NULL) {
    // FIXME! add subdev format to config, only needed on bebop front camera so far
    if (!v4l2_init_subdev(vid2->subdev_name, 0, 0, V4L2_MBUS_FMT_SGBRG10_1X10, vid2->w, vid2->h)) {
      printf("[video_thread] Could not initialize the %s subdevice.\n", vid2->subdev_name);
      return;
    }
  }

  // Initialize the V4L2 device
  vid2->thread.dev = v4l2_init(vid2->dev_name, vid2->w, vid2->h, vid2->buf_cnt, vid2->format);
  if (vid2->thread.dev == NULL) {
    printf("[video_thread] Could not initialize the %s V4L2 device.\n", vid->dev_name);
    return;
  }
}

/**
 * Start with streaming
 */
void video_thread_start(void)
{
  // Check if we are already running the front camera
  if (!front_camera.thread.is_running) {
    // Start the streaming thread for the front camera
    pthread_t tid;
    if (pthread_create(&tid, NULL, video_thread_function, (void *)(&front_camera)) != 0) {
      printf("[vievideo] Could not create streaming thread for the front camera.\n");
      return;
    }
  }

  if (!bottom_camera.thread.is_running) {
    pthread_t tid2;

    if (pthread_create(&tid2, NULL, video_thread_function, (void *)(&bottom_camera)) != 0) {
      printf("[vievideo] Could not create streaming thread for the bottom camera.\n");
      return;
    }
  }
}

/**
 * Stops the streaming
 * This could take some time, because the thread is stopped asynchronous.
 */
void video_thread_stop(void)
{

  if (front_camera.thread.is_running) {

    // Stop the streaming thread
    front_camera.thread.is_running = false;

    // Stop the capturing
    if (!v4l2_stop_capture(front_camera.thread.dev)) {
      printf("[video_thread] Could not stop capture of %s.\n", front_camera.thread.dev->name);
      return;
    }
  }

  if (bottom_camera.thread.is_running) {

    // Stop the streaming thread
    bottom_camera.thread.is_running = false;

    // Stop the capturing
    if (!v4l2_stop_capture(bottom_camera.thread.dev)) {
      printf("[video_thread] Could not stop capture of %s.\n", bottom_camera.thread.dev->name);
      return;
    }
  }
  // TODO: wait for the thread to finish to be able to start the thread again!
}

/**
 * Take a shot and save it
 * This will only work when the streaming is enabled
 */
void video_thread_take_shot(bool take)
{
  front_camera.thread.take_shot = take;
}

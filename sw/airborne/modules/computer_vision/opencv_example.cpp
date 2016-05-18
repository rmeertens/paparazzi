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
 * @file "modules/computer_vision/opencv_example.cpp"
 * @author C. De Wagter
 * opencv
 */


#include "opencv_example.h"
#include "cv_opencvdemo.h"


using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc.hpp>
//#include <opencv2/objdetect/objdetect.hpp>
//#include "opencv2/videoio.hpp"
//#include "opencv2/highgui.hpp"

using namespace cv;

Rect selection;
int vmin = 10, vmax = 256, smin = 30;

bool backprojMode = false;
bool selectObject = false;
int hsize = 16;

float hranges[] = {0, 180};
const float *phranges = hranges;

Rect trackWindow;

int trackObject = 0;
Mat hist;

Mat histimg = Mat::zeros(200, 320, CV_8UC3);
Mat image;
Mat hsv;
Mat mask;


Mat backproj;
Mat hue;
int opencv_example(char *img, int width, int height)
{
	selfie_var.trackingNumber++;
  // Create a new image, using the original bebop image.
  Mat M(width, height, CV_8UC2, img);

  // If you want a color image, uncomment this line
  cvtColor(M, image, CV_YUV2RGB_Y422);
  // For a grayscale image, use this one
  //cvtColor(M, image, CV_YUV2GRAY_Y422);
  cvtColor(image, hsv, COLOR_BGR2HSV);
  // CascadeClassifier cascade, nestedCascade;

  if (selfie_var.must_init) {
    selfie_var.must_init = false;
    selectObject = false;

	printf("Opencv example must init!!!\n");
    selection = Rect(selfie_var.startx, selfie_var.starty, selfie_var.width, selfie_var.height);
    trackObject = -1;
    backprojMode=true;
  }

  // Start select region
  if (trackObject) {
    int _vmin = vmin, _vmax = vmax;

    inRange(hsv, Scalar(0, smin, MIN(_vmin, _vmax)),
            Scalar(180, 256, MAX(_vmin, _vmax)), mask);
    int ch[] = {0, 0};
    hue.create(hsv.size(), hsv.depth());
    mixChannels(&hsv, 1, &hue, 1, ch, 1);
    if (trackObject < 0) {
      Mat roi(hue, selection), maskroi(mask, selection);
      calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
      normalize(hist, hist, 0, 255, NORM_MINMAX);

      trackWindow = selection;
      trackObject = 1;

      histimg = Scalar::all(0);
      int binW = histimg.cols / hsize;
      Mat buf(1, hsize, CV_8UC3);
      for (int i = 0; i < hsize; i++) {
        buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i * 180. / hsize), 255, 255);
      }
      cvtColor(buf, buf, COLOR_HSV2BGR);
      // end select region

      for (int i = 0; i < hsize; i++) {
        int val = saturate_cast<int>(hist.at<float>(i) * histimg.rows / 255);
        rectangle(histimg, Point(i * binW, histimg.rows),
                  Point((i + 1)*binW, histimg.rows - val),
                  Scalar(buf.at<Vec3b>(i)), -1, 8);
      }
    }


    calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
    backproj &= mask;
    RotatedRect trackBox = CamShift(backproj, trackWindow,
                                    TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 10, 1));
    if (trackWindow.area() <= 1) {
      int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5) / 6;
      trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,
                         trackWindow.x + r, trackWindow.y + r) &
                    Rect(0, 0, cols, rows);
    }

	selfie_var.trackingNow=true;
	selfie_var.trackingPercentageX=trackBox.center.x/width;
	selfie_var.trackingPercentageY=trackBox.center.y/height;

    if (backprojMode) {
      cvtColor(backproj, image, COLOR_GRAY2RGB);
    }

    ellipse(image, trackBox, Scalar(0, 0, 255), 3, LINE_AA);
  }

  // Convert back to YUV422, and put it in place of the original image
  // cvtColor(hsv, image, COLOR_HSV2RGB);
  printf("still here for select object? %d %d %d\n", selectObject, selection.width, selection.height);
  if (selectObject && selection.width > 0 && selection.height > 0) {
    printf("Select opject\n");
    Mat roi(image, selection);
    bitwise_not(roi, roi);
  }


//   cvtColor(image, image, COLOR_RGB2GRAY);
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      cv::Vec3b pixelHere = image.at<cv::Vec3b>(row, col);
      img[(row * width + col) * 2 + 1] = 0.299 * pixelHere[0] + 0.587 * pixelHere[1] + 0.114 * pixelHere[2];
      if (col % 2 == 0) { // U
        img[(row * width + col) * 2] = 0.492 * (pixelHere[2] - img[(row * width + col) * 2 + 1] + 127);
      } else { // V
        img[(row * width + col) * 2] = 0.877 * (pixelHere[0] - img[(row * width + col) * 2 + 1] + 127);
      }
    }
  }
  return 0;
}

#include <opencv2/core/core.hpp>
#include <stdio.h>
//#include <opencv2/highgui/highgui.hpp>
using namespace cv;

extern "C" void testImage(void);


void testImage(){
	Mat image(2,2, CV_8UC3, Scalar(0,0,255));
	printf("image width %d",image.rows);
}

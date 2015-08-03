/*
 * Copyright (C) ROland
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
 * @file "modules/read_matrix_serial/read_matrix_serial.c"
 * @author ROland
 * reads from the serial
 */

#include "modules/read_matrix_serial/read_matrix_serial.h"
#include "modules/computer_vision/opticflow_module.h"

#include "subsystems/datalink/telemetry.h"
#include <stdio.h>
#include <sys/fcntl.h>
#include <math.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <inttypes.h>
#include <serial_port.h>
#include "read_matrix_serial.h"
#include "../computer_vision/opticflow/stabilization_opticflow.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_algebra_float.h"
#include "state.h"

speed_t usbInputSpeed = B1000000;

#define PRINT_STUFF 1

uint8_t singleImageColumnCount=6;
uint8_t camerasCount=6;
uint8_t imageHeight=6;
uint8_t imageWidth;

int lengthBytesImage;
int lengthBytesInputArray;

typedef struct ImageProperties{
	int positionImageStart;
	int lineLength;
	int height;
} ImageProperties;

struct termios tty;
uint8_t *READimageBuffer;
uint8_t READimageBuffer_old[36*6];
float butter_old[36*6] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t READimageBuffer_offset[36*6];
uint8_t *serialResponse;
int writeLocationInput=0;
uint8_t SendREADimageBuffer[36];

struct SerialPort *READING_port;
int messageArrayLocation=0;
int widthToSend;

//Variables Kalman filter
float A_kal=1;
float B_kal=0;
float H_kal = 1;
float Q_kal = 0.05;
float R_kal = 2;

float Xest_new_send[36]; 
float K_gain_send = 0;

float Pest_new[36*6];
float Xest_new[36*6];

int size_matrix[3] = {6, 6, 6};

float ref_pitch_angle = 0.2;
uint16_t matrix_sum[6]={0,0,0,0,0,0};
uint16_t matrix_sum2[6]={0,0,0,0,0,0};
uint16_t matrix_sum_treshold = 0;
float matrix_treshold = 7.0;

int32_t cnt_obst[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
float  b_damp = 5.5; 
float  K_goal = 0;
float  K_obst = 20.0;
float  c1 = 0.4;
float  c2 = 0.4;
float  c3 = 3.0;
float  c5 = 0.9;
float  kv = 0.5;
float  epsilon = 0.1;
float  vmax = 0.5;
float  vmin = 0.2;
float  pos_diff = 2;
uint8_t point_index = 0;
float heading_goal_f = 0;

//////////////////////////////////////DEFINED IN STABILIZATION_OPTICFLOW.H/////////////////////////////////////////////////
// int8_t filter_flag = 0;    0 =>no filter 1 =>Kalman filter 2 =>Butterworth filter
// int8_t OA_method_flag = 0; 0 =>ping pong 1=>vector 2=>potentialfield 3=>Safetyzone

static void send_distance_matrix(void) {
	DOWNLINK_SEND_DISTANCE_MATRIX(DefaultChannel, DefaultDevice, &messageArrayLocation,36, SendREADimageBuffer);
}

 static void send_MATRIX_KALMAN(void) {
        DOWNLINK_SEND_MATRIX_KALMAN(DefaultChannel, DefaultDevice, &K_gain_send, 36, Xest_new_send);
 }


void allocateSerialBuffer(int widthOfImage, int heightOfImage)
{
	imageHeight=heightOfImage;
	imageWidth=widthOfImage;

//	lengthBytesInputArray=2*((widthOfImage+8)*heightOfImage+8); // Length of the complete image, including indicator bytes, two times (to make sure we see it)
	lengthBytesInputArray=50000;
	size_t sizeArrays=50000;
	serialResponse=malloc(sizeArrays * sizeof(uint8_t));
	memset(serialResponse, '0', sizeArrays);


	lengthBytesImage=50000;//COMPLETE_MATRIX_WIDTH*MATRIX_ROWS;
	READimageBuffer=malloc(sizeArrays*sizeof(uint8_t));
	memset(READimageBuffer, '0', sizeArrays);

}

/**
 * Checks if the sequence in the array is equal to 255-0-0-171,
 * as this means that this is the end of an image
 */
int isEndOfImage(uint8_t *stack){
	if (stack[0] == 255 && (stack[1] == 0) && (stack[2] == 0) && stack[3]==171){
		return 1;
	}
	return 0;
}

/**
 * Checks if the sequence in the array is equal to 255-0-0-171,
 * as this means a new image is starting from here
 */
int isStartOfImage(uint8_t *stack){
	if (stack[0] == 255 && (stack[1] == 0) && (stack[2] == 0) && stack[3]==175){
		return 1;
	}
	return 0;
}

ImageProperties get_image_properties(uint8_t *raw, int size, int startLocation){
    ImageProperties imageProperties={-1,-1,-1};
    int boolStartCounting=0;
    int startOfLine=0;
    // Search for the startposition of the image, the end of the image,
    // and the width and height of the image
    for (int i=startLocation; i < startLocation+size+10; i++){
    	// Check the first 3 bytes for the pattern 255-0-0, then check what special byte is encoded next
        if ((raw[i] == 255) && (raw[i + 1] == 0) && (raw[i + 2] == 0)){
            if (raw[i + 3] == 171 && imageProperties.positionImageStart >= 0){ // End of image
                break;
            }
            if (raw[i + 3] == 175){ // Start of image
            	imageProperties.positionImageStart = i;
            	boolStartCounting=1;
            	imageProperties.height=0;
            }
            if (raw[i + 3] == 128){ // Start of line
            	startOfLine = i;
			}
            if (raw[i + 3] == 218 && boolStartCounting==1){ // End of line

            	imageProperties.lineLength = i-startOfLine-4; // removed 4 for the indication bit
#if PRINT_STUFF
            	printf("Line length: %d \n", imageProperties.lineLength);
#endif
            	imageProperties.height+=1;
			}
        }
    }

    return imageProperties;
}

void serial_init(void) {
	  //Initalize Kalmann filter
	  for(int tel1=0;tel1<(36*6);tel1++){
	      Pest_new[tel1] = 1;
	      Xest_new[tel1] = 1;
	  }
  
	imageWidth=singleImageColumnCount*camerasCount;
	lengthBytesImage=imageWidth*imageHeight;//camerasAmount*matrixColumns*matrixRows+4+matrixRows*8
	allocateSerialBuffer(imageWidth,imageHeight);

  
	// Open the serial port
	READING_port = serial_port_new();
	int result=serial_port_open_raw(READING_port,"/dev/ttyUSB0",usbInputSpeed);
	register_periodic_telemetry(DefaultPeriodic, "DISTANCE_MATRIX", send_distance_matrix);
	register_periodic_telemetry(DefaultPeriodic, "MATRIX_KALMAN", send_MATRIX_KALMAN);
}		

void printArray(uint8_t *toPrintArray, int totalLength, int width)
{
#if PRINT_STUFF
	for (int x = 0; x < totalLength; x++)
	{

	  printf(" ,%2d ",toPrintArray[x]);
	  if ((x+1)%width==0){
		  printf("line end\n");
	  }
	}
#endif
	for (int x = 0; x < totalLength; x++)
	{
		if(toPrintArray[x]>10){
			//printf(" ,%d ",toPrintArray[x]);
			 //printf("Danger!");
			 // Set to -5 degrees

		}



	}
}	

int isImageReady(int end, int start, int prevStart)
{
   if(start <0)
   {
	   return 0;
   }
   if(end <0)
   {
	   return 0;
   }
   if(start-end > 0)
   {
	   if(prevStart >=0)
	   {
		   return 1;
	   }
   }
   if(end-start>0)
   {
	   return 1;
   }

   return 0;
}

void serial_start(void)
{
	//printf("serial start\n");
}

void serial_update(void) {
	int n=0;
	int timesTriedToRead=0;
	int8_t diparity_offset[6] = {1,1,5,3,1,0};
	//int8_t diparity_offset[6] = {2,?,4,0,8,8}; //offset multigaze 2
	
	//Parameters for Kalman filter
	float Xpred_new[size_matrix[0]*size_matrix[1]*size_matrix[2]];
	float Xest_old[size_matrix[0]*size_matrix[1]*size_matrix[2]];
	float Ppred_new[size_matrix[0]*size_matrix[1]*size_matrix[2]];
	float Pest_old[size_matrix[0]*size_matrix[1]*size_matrix[2]];
	float K_gain[size_matrix[0]*size_matrix[1]*size_matrix[2]];
	
	float butter[size_matrix[0]*size_matrix[1]*size_matrix[2]];
	float A_butter = -0.7265;
	float B_butter[2] = {0.1367, 0.1367}; 
	
	if(filter_flag==2){
	    for (int ifill=0;ifill<(6*36);ifill++){
		READimageBuffer_old[ifill] = READimageBuffer[ifill];
	    }
	 }

	n = read(  READING_port->fd, &serialResponse[writeLocationInput], 10000 );
#if PRINT_STUFF
	printf("Read %d bytes\n",n);
#endif
	timesTriedToRead++;
	int lastRecordedStart=-1;
	int lastRecordedEnd=-1;
	int previousStart =-1;
	if (n > 0){
		// Check if we found the end of the image
		for(int startLocationToSearch=0; startLocationToSearch<writeLocationInput+n;startLocationToSearch++)
		{

				if(isStartOfImage(&serialResponse[startLocationToSearch]))
				{

					previousStart=lastRecordedStart;
					lastRecordedStart=startLocationToSearch;
				}
				if(isEndOfImage(&serialResponse[startLocationToSearch]))
				{
					lastRecordedEnd=startLocationToSearch;
				}
		}
#if PRINT_STUFF
		printf("Last recorded end: %d , last Recorded start: %d previous start %d !\n",lastRecordedEnd, lastRecordedStart, previousStart);
#endif
		if(isImageReady(lastRecordedEnd, lastRecordedStart, previousStart)>0)
		{
#if PRINT_STUFF
			printf("Searhing properties. LastRecorded start: %d lastRecordedEnd: %d previousStart: %d \n",lastRecordedStart,lastRecordedEnd, previousStart);
#endif
			// Find the properties of the image by iterating over the complete image
			ImageProperties imageProperties;
			if (lastRecordedEnd > lastRecordedStart){
				imageProperties= get_image_properties(serialResponse, lastRecordedEnd-lastRecordedStart, lastRecordedStart);
			}
			else
			{
				imageProperties = get_image_properties(serialResponse, lastRecordedEnd-previousStart, previousStart);
			}
#if PRINT_STUFF
			printf("Found image properties, start position: %d , width: %d, height: %d \n", imageProperties.positionImageStart, imageProperties.lineLength, imageProperties.height);
#endif
			if(imageProperties.positionImageStart<0 || imageProperties.height<0||imageProperties.lineLength<0){
				printf("[Read_matrix_serial] serious problem with the image properties");
				return;
			}
			if(imageProperties.height!=imageHeight || imageProperties.lineLength != imageWidth)
			{

				// Because image properties might change (when uploading new code to the multigaze), we need to resize arrays
				// and set the width and height variables
				printf("[Read_matrix_serial] image properties were not as expected");
				imageHeight=imageProperties.height;
				imageWidth=imageProperties.lineLength;
			}
			else
			{

				// Remove all bytes that are indications of start and stop lines
				int imagePixelIndex=0;
				for (int i = imageProperties.positionImageStart; i < imageProperties.positionImageStart+(imageProperties.lineLength+8)*imageProperties.height+8;i++){

					if ((serialResponse[i] == 255) && (serialResponse[i + 1] == 0) && (serialResponse[i + 2] == 0)){
						if (serialResponse[i + 3] == 128){ // Start Of Line

							// Add four to the start and end positions to skip the first four bytes (255-0-0-128)
							int startOfBuf = i + 4;
							int endOfBuf = (i + 4 + imageProperties.lineLength);

							// Copy from the serialResponse in the imagebuffer
							// Hereby removing all bytes that indicate the start of images and lines
							for(int indexInBuffer = startOfBuf; indexInBuffer < endOfBuf; indexInBuffer++){
								READimageBuffer[imagePixelIndex] = serialResponse[indexInBuffer];
								imagePixelIndex++;
							}
						}
					}
				}

				printArray(READimageBuffer,imageProperties.height*imageProperties.lineLength,imageProperties.lineLength);
			}

			// Now move everything after the end of the buffer to the start of the buffer
			int locationNewImage=0;
			for(int toProcess=lastRecordedEnd;toProcess<writeLocationInput+n;toProcess++)
			{
				serialResponse[locationNewImage++]=serialResponse[toProcess];
			}
			writeLocationInput=locationNewImage; // As we found a complete image we will now start writing at the start of the buffer again
		     	
		     	////////////////////////////IMAGE READ, NOW DATA FILTERING/////////////////////////////////
		     	if(filter_flag ==1){
			  
			      //Kallman filter on disparity matrix
			      for (int i_k=0;i_k<(6*36);i_k++){
				    Pest_old[i_k] = Pest_new[i_k];
				    Xest_old[i_k] = Xest_new[i_k];
				    
				    //one step ahead prediction
				    Xpred_new[i_k] = A_kal*Xest_old[i_k];
				    
				    //Covariance matrix of state prediction error
				    Ppred_new[i_k] = A_kal*Pest_old[i_k]*A_kal + Q_kal;
				    
				    //Kalman gain calculation
				    K_gain[i_k] = Ppred_new[i_k]*H_kal*1.0/(H_kal*Ppred_new[i_k]*H_kal + R_kal);
				    
				    //Measurement update
				    Xest_new[i_k] = Xpred_new[i_k] + K_gain[i_k]*(float)(READimageBuffer[i_k] - H_kal*Xpred_new[i_k]);
			      
				    //Covariance matrix of state estimation error
				    Pest_new[i_k] = (1-K_gain[i_k]*H_kal)*Ppred_new[i_k];
			      }	
			} 
			
			if(filter_flag==2){
			     //butterworth filter
			      for (int i_k=0;i_k<(6*36);i_k++){
				
				    if((READimageBuffer_old[i_k] -READimageBuffer[i_k] )<=1 && (READimageBuffer_old[i_k] -READimageBuffer[i_k])>0){
					READimageBuffer[i_k] = READimageBuffer_old[i_k]; 
				      
				    }
				    
				    butter[i_k] = B_butter[0]*(float)READimageBuffer[i_k] + B_butter[1]*(float)READimageBuffer_old[i_k] - A_butter*butter_old[i_k];
				    butter_old[i_k] = butter[i_k];
			      }
			}
			
		       for(int i_fill2=0;i_fill2<size_matrix[2];i_fill2++){
			      for(int i_fill3=0;i_fill3<size_matrix[1];i_fill3++){
				    if(filter_flag==1){
					Xest_new_send[i_fill2*size_matrix[1] + i_fill3] =  Xest_new[0*size_matrix[1]+i_fill2*size_matrix[0]*size_matrix[2] + i_fill3]; 
				    }
				    else if(filter_flag==2){  
				        Xest_new_send[i_fill2*size_matrix[1] + i_fill3] =  butter[0*size_matrix[1]+i_fill2*size_matrix[0]*size_matrix[2] + i_fill3]; 
				   }
			      }
		       }
			
			for (int i_m=0;i_m<size_matrix[0];i_m++){
			    matrix_sum[i_m] = 0;
			    matrix_sum2[i_m] = 0;
			    
			      for(int i_m2=0;i_m2<size_matrix[2];i_m2++){
				    for(int i_m3=0;i_m3<size_matrix[1];i_m3++){
					  READimageBuffer_offset[i_m*size_matrix[1]+i_m2*size_matrix[0]*size_matrix[2] + i_m3] = READimageBuffer[i_m*size_matrix[1]+i_m2*size_matrix[0]*size_matrix[2] + i_m3] + diparity_offset[i_m];
					
					  if(READimageBuffer_offset[i_m*size_matrix[1]+i_m2*size_matrix[0]*size_matrix[2] + i_m3]>matrix_treshold){
					      matrix_sum[i_m] = matrix_sum[i_m] + 1;
					  }	 
  // 					if((READimageBuffer_offset[i_m*size_matrix[1]+i_m2*size_matrix[0]*size_matrix[2] + i_m3]>(matrix_treshold-1))&& (READimageBuffer_offset[i_m*size_matrix[1]+i_m2*size_matrix[0]*size_matrix[2] + i_m3]<=matrix_treshold))
  // 					   matrix_sum2[i_m] = matrix_sum2[i_m] + 1;
				    }
			      }		
			}
			    
			for(int i_fill2=0;i_fill2<size_matrix[2];i_fill2++){
			      for(int i_fill3=0;i_fill3<size_matrix[1];i_fill3++){
				  SendREADimageBuffer[i_fill2*size_matrix[1] + i_fill3] =  READimageBuffer_offset[messageArrayLocation*size_matrix[1]+i_fill2*size_matrix[0]*size_matrix[2] + i_fill3]; 
			      }
			}
			    
			  ////////////////////////////CALCULATE CONTROL ACTION/////////////////////////////////   
		      if(OA_method_flag==0){
			  cal_euler_pingpong();
		      }
		      
		      if(OA_method_flag==1){  
			  nav_cal_heading_stereo();
		      }
		      if(OA_method_flag==2){
			  nav_cal_heading_vector();
		      }
			      
		}
		else
		{
			writeLocationInput+=n;
		}
	}
}
void cal_euler_pingpong(){
  	
	float oa_pitch_angle[6]={0,0,0,0,0,0};
        float oa_roll_angle[6]={0,0,0,0,0,0};

// 		      //define control action
// 		      if (matrix_sum2[0]>matrix_sum_treshold){
// 			    oa_pitch_angle[0] = 0.5*ref_pitch_angle;
// 			    oa_roll_angle[0] = 0; 
// 			} 
// 		      if (matrix_sum2[1]>matrix_sum_treshold){
// 			    oa_pitch_angle[1] = cos((60.0/360.0)*2*M_PI)*0.5*ref_pitch_angle;
// 			    oa_roll_angle[1] = -sin((60.0/360.0)*2*M_PI)*0.5*ref_pitch_angle; 
// 			}
// 		      if (matrix_sum2[2]>matrix_sum_treshold){
// 			    oa_pitch_angle[2] = cos((120.0/360.0)*2*M_PI)*0.5*ref_pitch_angle;
// 			    oa_roll_angle[2] = -sin((120.0/360.0)*2*M_PI)*0.5*ref_pitch_angle; 
// 			}
// 		      if (matrix_sum2[3]>matrix_sum_treshold){
// 			    oa_pitch_angle[3] = -ref_pitch_angle;
// 			    oa_roll_angle[3] = 0; 
// 			}
// 		      if (matrix_sum2[4]>matrix_sum_treshold){
// 			    oa_pitch_angle[4] = cos((240.0/360.0)*2*M_PI)*0.5*ref_pitch_angle;
// 			    oa_roll_angle[4] = -sin((240.0/360.0)*2*M_PI)*0.5*ref_pitch_angle; 
// 			}
// 		      if (matrix_sum2[5]>matrix_sum_treshold){
// 			    oa_pitch_angle[5] = cos((300.0/360.0)*2*M_PI)*0.5*ref_pitch_angle;
// 			    oa_roll_angle[5] = -sin((300.0/360.0)*2*M_PI)*0.5*ref_pitch_angle; 
// 			} 
	    
	
	if (matrix_sum[0]>matrix_sum_treshold){
	      oa_pitch_angle[0] = ref_pitch_angle;
	      oa_roll_angle[0] = 0; 
	  } 
	if (matrix_sum[1]>matrix_sum_treshold){
	      oa_pitch_angle[1] = cos((60.0/360.0)*2*M_PI)*ref_pitch_angle;
	      oa_roll_angle[1] = -sin((60.0/360.0)*2*M_PI)*ref_pitch_angle; 
	  }
	if (matrix_sum[2]>matrix_sum_treshold){
	      oa_pitch_angle[2] = cos((120.0/360.0)*2*M_PI)*ref_pitch_angle;
	      oa_roll_angle[2] = -sin((120.0/360.0)*2*M_PI)*ref_pitch_angle; 
	  }
	if (matrix_sum[3]>matrix_sum_treshold){
	      oa_pitch_angle[3] = -ref_pitch_angle;
	      oa_roll_angle[3] = 0; 
	  }
	if (matrix_sum[4]>matrix_sum_treshold){
	      oa_pitch_angle[4] = cos((240.0/360.0)*2*M_PI)*ref_pitch_angle;
	      oa_roll_angle[4] = -sin((240.0/360.0)*2*M_PI)*ref_pitch_angle; 
	  }
	if (matrix_sum[5]>matrix_sum_treshold){
	      oa_pitch_angle[5] = cos((300.0/360.0)*2*M_PI)*ref_pitch_angle;
	      oa_roll_angle[5] = -sin((300.0/360.0)*2*M_PI)*ref_pitch_angle; 
	  } 
	  
	  //limiter of control action
	  if((oa_pitch_angle[0] + oa_pitch_angle[1] + oa_pitch_angle[2] + oa_pitch_angle[3]+ oa_pitch_angle[4] + oa_pitch_angle[5])>ref_pitch_angle){
	    ref_pitch = ref_pitch_angle;
	  }
	  else if((oa_pitch_angle[0] + oa_pitch_angle[1] + oa_pitch_angle[2] + oa_pitch_angle[3]+ oa_pitch_angle[4] + oa_pitch_angle[5])<-ref_pitch_angle){
	    ref_pitch = -ref_pitch_angle;
	  }
	  else{
	    ref_pitch = oa_pitch_angle[0] + oa_pitch_angle[1] + oa_pitch_angle[2] + oa_pitch_angle[3]+ oa_pitch_angle[4] + oa_pitch_angle[5];
	  }
	  
	  if((oa_roll_angle[0] + oa_roll_angle[1] + oa_roll_angle[2] + oa_roll_angle[3] + oa_roll_angle[4] + oa_roll_angle[5])>ref_pitch_angle)
	    {
	      ref_roll = ref_pitch_angle;
	    }
	    else if((oa_roll_angle[0] + oa_roll_angle[1] + oa_roll_angle[2] + oa_roll_angle[3] + oa_roll_angle[4] + oa_roll_angle[5])<-ref_pitch_angle)
	    {
	      ref_roll = -ref_pitch_angle;
	    }
	  else
	    {
	      ref_roll = oa_roll_angle[0] + oa_roll_angle[1] + oa_roll_angle[2] + oa_roll_angle[3] + oa_roll_angle[4] + oa_roll_angle[5]; 
	    }   
  
}

void nav_cal_heading_stereo(){
	bool_t flag_speed_control = FALSE;
	
	current_heading = atan(OF_Result_Vy/OF_Result_Vx);
	heading_goal_ref = current_heading-heading_goal_f;
	
	if (heading_goal_ref>M_PI){
	  heading_goal_ref = heading_goal_ref - 2*M_PI;
	}
	else if(heading_goal_ref<-M_PI){
	    heading_goal_ref = heading_goal_ref + 2*M_PI;
	}
	
      uint8_t matrix_read[16];
	
	for (int i_m=0;i_m<16;i_m++){
	  matrix_read[i_m] = READimageBuffer[i_m];
	}
	
      //define image size HAS TO BE CHECKED!
      int size_matrix[3] = {1, 4, 4};
      float stereo_fow[2] = {1.0018, 0.7767};//based on FOW of 57.4, by 44.5
      float angle_change;
      float r_o;
      float r_r;
      
      //define potential field variables
      //float r_new;
      float r_old = stateGetBodyRates_f()->r;

      float potential_obst = 0; 
      float potential_obst_integrated = 0;
      
      //define delta t for integration! check response under 0.1 and 1
      float dt = 0.5;
      
      //calculate position angle and angular widht of matrix
      for(int i_o=0;i_o<size_matrix[2];i_o++){ 
	    obst_width[i_o] = stereo_fow[0]/size_matrix[2];
	    obst_angle[i_o] = - 0.5*stereo_fow[0] - stereo_fow[0]/size_matrix[2] + stereo_fow[0]/size_matrix[2]/2 + (stereo_fow[0]/size_matrix[2])*i_o;
	
      }
      
      //calculate position angle and angular widht of obstacles
	for(int i=0;i<size_matrix[2];i++){
	    r_o = (tan(obst_angle[i]+0.5*obst_width[i])*(matrix_read[4+i]+matrix_read[8+i])/2) - (tan(obst_angle[i]-0.5*obst_width[i])*(matrix_read[4+i]+matrix_read[8+i])/2);
	    r_r = 0.3;
	    c5 = M_PI/2 - 2*atan(r_o/(r_o+r_r));
	
	    if (cnt_obst[i]>50 && obst_angle[i] != -(0.5*stereo_fow[0])){
	      potential_obst = potential_obst + K_obst*(-obst_angle[i])*exp(-c3*abs(obst_angle[i]))*(tan(obst_width[i]+c5)-tan(c5));
	      potential_obst_write = potential_obst;
	      potential_obst_integrated = potential_obst_integrated + K_obst*c3*(abs(obst_angle[i])+1)/(c3*c3)*exp(-c3*abs(obst_angle[i]))*(tan(obst_width[i]+c5)-tan(c5));
	      
	    }
	}
	
      //calculate angular accelaration from potential field
	r_dot_new = -b_damp*r_old - K_goal*(heading_goal_ref)*(exp(-c1*pos_diff)+c2) + potential_obst;
	//delta_heading = 0.5*r_dot_new*dt*dt;
	//Integrate using simple intgration CHECK for time step!
	//heading_new = current_heading + 0.5*r_dot_new*dt*dt;

	if(flag_speed_control=FALSE){
	  speed_pot = vmax*exp(-kv*potential_obst_integrated) - epsilon;
	  if(speed_pot>0){
	  //TODO
	  }
	  else if(speed_pot<=0){
	//TODO
	    speed_pot = 0;
	  }
	}
	
}

void nav_cal_heading_vector(){
       //variables that need/can be tuned: 
      float F1 =0.5;
      float F2 =0.5;
      float Cfreq = 0.5;  
      float Ko = 5.0;
      float Kg = 1.0;
      float Dist_offset = -0.125;
      int8_t dis_treshold = 7;	
  
      float fp_angle;
      float total_vel;
      float force_max = 1000;
      float Distance_est;
      float Ca;
      float Cv;

	//define FOW STEREO camera
      float stereo_fow[2] = {1.0018, 0.7767};//based on FOW of 57.4, by 44.5
				
      //Repulsion force, Attracforce and total force with goal
      struct FloatVect3 Repulsionforce_Kan = {0,0,0};
      struct FloatVect3 Total_Kan = {0,0,0};
      struct FloatVect3 Attractforce_goal = {0,0,0};

      //Calculate Attractforce_goal size = 1; 
      Attractforce_goal.x = 1*cos(heading_goal_f);
      Attractforce_goal.y = 1*sin(heading_goal_f);
      Attractforce_goal.z = 0;

      //Variables needed for Holenstein
 
      
      //init angle values in input matrix in body frame
      float angle_hor = - 0.5*stereo_fow[0] - stereo_fow[0]/size_matrix[2] + stereo_fow[0]/size_matrix[2]/2;
      float angle_ver = 0;
      
      total_vel = pow((OF_Result_Vy*OF_Result_Vy + OF_Result_Vx*OF_Result_Vx),0.5);
      
      if (total_vel>vmin){
	  fp_angle = atan2(OF_Result_Vx,OF_Result_Vy);
      }
      else{
	  fp_angle = 0;
      }
      
      heading_goal_ref = fp_angle-heading_goal_f;
      
      if (heading_goal_ref>M_PI){
	heading_goal_ref = heading_goal_ref - 2*M_PI;
      }
      else if(heading_goal_ref<-M_PI){
	  heading_goal_ref = heading_goal_ref + 2*M_PI;
	}
	
      for (int i1=0;i1<size_matrix[0];i1++){
	  for (int i3=0;i3<size_matrix[2];i3++){
	      angle_hor = angle_hor + stereo_fow[0]/size_matrix[2];
	      angle_ver = 0.5*stereo_fow[1] + stereo_fow[1]/size_matrix[1] - stereo_fow[1]/size_matrix[1]/2;
	      for (int i2=0;i2<size_matrix[1];i2++){
		  angle_ver = angle_ver - stereo_fow[1]/size_matrix[1];
		  
		  if(abs((angle_hor-(fp_angle))*Cfreq)<0.5*M_PI){
		      Ca = cos((angle_hor-(fp_angle))*Cfreq);
		  }
		  else{
		      Ca = 0;
		  }
		  Cv = F1 + F2 * total_vel/vmax*Ca;
		  
		  if(READimageBuffer_offset[i1*size_matrix[1]+i3*size_matrix[0]*size_matrix[2] + i2]>dis_treshold){
		      Distance_est = 3-0.2*READimageBuffer_offset[i1*size_matrix[1]+i3*size_matrix[0]*size_matrix[2] + i2];   
		  }
		  else {
		      Distance_est = 2000;
		  }  

		  if(pow(Cv/(READimageBuffer_offset[i1*size_matrix[1]+i3*size_matrix[0]*size_matrix[2] + i2]-0.2),2)<force_max){
		      Repulsionforce_Kan.x = Repulsionforce_Kan.x - pow(Cv/(Distance_est+Dist_offset),2)*sin(angle_hor)*cos(angle_ver);
		      Repulsionforce_Kan.y = Repulsionforce_Kan.y - pow(Cv/(Distance_est+Dist_offset),2)*cos(angle_hor)*cos(angle_ver);
		      Repulsionforce_Kan.z = Repulsionforce_Kan.z - pow(Cv/(Distance_est+Dist_offset),2)*sin(angle_ver);
		  }
		  else{
		      Repulsionforce_Kan.x = Repulsionforce_Kan.x - force_max*sin(angle_hor)*cos(angle_ver);
		      Repulsionforce_Kan.y = Repulsionforce_Kan.y - force_max*cos(angle_hor)*cos(angle_ver);
		      Repulsionforce_Kan.z = Repulsionforce_Kan.z - force_max*sin(angle_ver);
		  }
	      }
	  }
      }

      //Normalize for ammount entries in Matrix
      //Repulsionforce_Kan = Repulsionforce_Kan/(float)(size_matrix[1]*size_matrix[2]);
      VECT3_SMUL(Repulsionforce_Kan, Repulsionforce_Kan, 1.0/(size_matrix[0]*size_matrix[1]*size_matrix[2])); 

      //Repulsionforce_Kan_write[0] = (float)Repulsionforce_Kan.x;
      //Repulsionforce_Kan_write[1] = (float)Repulsionforce_Kan.y;
      //Repulsionforce_Kan_write[2] = (float)Repulsionforce_Kan.z;

      VECT3_SMUL(Repulsionforce_Kan,Repulsionforce_Kan,Ko);
      VECT3_SMUL(Attractforce_goal, Attractforce_goal, Kg);

      //Total force 
      VECT3_ADD(Total_Kan, Repulsionforce_Kan);
      VECT3_ADD(Total_Kan, Attractforce_goal);

      //Total_Kan_write[0] = (float)Total_Kan.x;
      //Total_Kan_write[1] = (float)Total_Kan.y;
      //Total_Kan_write[2] = (float)Total_Kan.z;
      
      //set variable for stabilization_opticflow
      ref_pitch = Total_Kan.x;
      ref_roll = Total_Kan.y;
}

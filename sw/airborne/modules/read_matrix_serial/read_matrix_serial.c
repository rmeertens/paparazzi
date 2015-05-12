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
#include "subsystems/datalink/telemetry.h"
#include <stdio.h>
#include <sys/fcntl.h>
#include <math.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <inttypes.h>

#define toRead 50


int USB_INPUT = 0;
char psResponse[toRead];
double n;
int SIZE_OF_ONE_IMAGE=50;
uint8_t MATRIX_WIDTH = 4;
uint8_t lineBuffer[16];
struct termios tty;



static void send_distance_matrix(void) {
	DOWNLINK_SEND_DISTANCE_MATRIX(DefaultChannel, DefaultDevice, sizeof lineBuffer, lineBuffer);
 }

int search_start_position(int startPosition, int size_of_one_image, uint8_t* raw){
    int sync=0;
    // Search for the startposition
    for (int i=startPosition; i < size_of_one_image; i++){
        if ((raw[i] == 255) && (raw[i + 1] == 0) && (raw[i + 2] == 0)){
	    printf("Possible end of image: %d", i);
            if (raw[i + 3] == 171){
                sync = i;
                break;
	    }
	}
    }
    return sync;
}

void serial_init(void) {
	printf("Init serial\n");
	register_periodic_telemetry(DefaultPeriodic, "DISTANCE_MATRIX", send_distance_matrix);

	USB_INPUT = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);

	memset (&tty, 0, sizeof tty);


	if ( tcgetattr ( USB_INPUT, &tty ) != 0 ) {
		printf("Error XXX from tcgetattr: XXX\n");//	   printf("Error %s" << errno << " from tcgetattr: " << strerror(errno) << std::endl;
	}


	cfsetospeed (&tty, (speed_t)B3000000);
	cfsetispeed (&tty, (speed_t)B3000000);


	tty.c_cflag     &=  ~PARENB;            // Make 8n1
	tty.c_cflag     &=  ~CSTOPB;
	tty.c_cflag     &=  ~CSIZE;
	tty.c_cflag     |=  CS8;

	tty.c_cflag     &=  ~CRTSCTS;           // no flow control
	tty.c_cc[VMIN]   =  1;                  // read doesn't block
	tty.c_cc[VTIME]  =  50;                  // 0.5 seconds read timeout
	tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines


	cfmakeraw(&tty);
//	close(USB_INPUT);

}
void serial_update(void) {

	for (int x = 0; x < sizeof lineBuffer; x++)
	{
	  lineBuffer[x] = 0; 
	}
	
	printf("---Reading read distance matrix-----\n");

/*
	tcflush( USB_INPUT, TCIFLUSH );	//Flush Port, then applies attributes 
	if ( tcsetattr ( USB_INPUT, TCSANOW, &tty ) != 0) {
	   //std::cout << "Error " << errno << " from tcsetattr" << std::endl;
	   printf("Error XXX from tcsetattr XXX\n");
	}
*/
	int n = 0,
	spot = 0;
	char buf = '\0';



	uint8_t response[2*SIZE_OF_ONE_IMAGE];
	memset(response, '\0', sizeof response);

	do {
	   n = read( USB_INPUT, &buf, 1 );
	   sprintf( &response[spot], "%c", buf );
	   spot += n;
	//   printf("-%d->%d<",buf,n);
	//} while( (spot < sizeof response) && (buf != '\r') && (n > 0));
	} while( (spot < sizeof response) && (n > 0));
	//printf("\n---\nDone with this! \n %d %d %d\n---\n", spot < sizeof response, buf != '\r', n > 0);

	int startPos = 0; 

	int arrayIndex = 0;

	for (int x = 0; x < sizeof lineBuffer; x++)
	{
	  lineBuffer[x] = 0; 
	}


	for (int i = startPos; i < SIZE_OF_ONE_IMAGE + startPos;i++){
		if ((response[i] == 255) && (response[i + 1] == 0) && (response[i + 2] == 0)){
		    if (response[i + 3] == 128){
		 	// Start Of Line
		        int startOfBuf = i + 4;
		        int endOfBuf = (i + 4 + MATRIX_WIDTH);
			for(int indexInBuffer = startOfBuf; indexInBuffer < endOfBuf; indexInBuffer++){
		   	    lineBuffer[arrayIndex] = response[indexInBuffer];
			    arrayIndex++;
			}
		    }
		  }
	}

	for (int x = 0; x < sizeof lineBuffer; x++)
	{
	  printf("%d\n",lineBuffer[x]);
	}

}
void serial_start(void)
{
	//printf("serial start\n");
}



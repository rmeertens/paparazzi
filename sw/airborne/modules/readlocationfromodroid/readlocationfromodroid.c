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
 * @file "modules/readlocationfromodroid/readlocationfromodroid.c"
 * @author Roland
 * reads from the odroid
 */

#include "modules/readlocationfromodroid/readlocationfromodroid.h"

#include <fcntl.h>
#include <stropts.h>
#include <termios.h>
#include <serial_port.h>
#include <stdio.h>
#include <inttypes.h>
#include <errno.h>
#include <string.h>
#include "cJSON.h"
struct SerialPort *READING_port;
speed_t usbInputSpeed = B115200;
char *serialResponse;
int writeLocationInput=0;
 void odroid_loc_init() {
	 printf("Loc init\n");

	// Open the serial port
	READING_port = serial_port_new();
	int result=serial_port_open_raw(READING_port,"/dev/ttyUSB0",usbInputSpeed);
	int lengthBytesImage=50000;//COMPLETE_MATRIX_WIDTH*MATRIX_ROWS;
	serialResponse=malloc(lengthBytesImage*sizeof(char));
	memset(serialResponse, '0', lengthBytesImage);


	char* initStuff = "{\"name\": 1}";
	printf("Original json %s\n",initStuff);
	cJSON * root = cJSON_Parse(initStuff);
	printf("result json: \n");
	char* toPrint = "going to print";
	printf(cJSON_Print(root));
	//printf(toPrint);

	int framerate = cJSON_GetObjectItem(root,"name")->valueint;
	//printf("Format: %s",format);
	printf("framerate: %d",framerate);
	printf("^^^^^\n");

 }
 void odroid_loc_periodic() {
	printf("Loc periodic! %d", writeLocationInput);
	char justRead='a';
	int n = read(  READING_port->fd, &serialResponse[writeLocationInput], 100);
	//printf("Read %d bytes\n",n);
	if (n < 0)
	{
		printf(strerror(errno));
	}
	else{
		writeLocationInput+=n;
		serialResponse[writeLocationInput]='\0';
		int index=0;
		printf("Now read in total: %s\n",serialResponse);
		int start=-1;
		for(index=0; index < n; index++){

			printf("%c",serialResponse[index]);
			if(serialResponse[index]=='{'){
				start=index;
			}
			if(serialResponse[index]=='\n' && start >=0)
			{
				printf("Found end of line!!!");
				char subbuf[index];
				memcpy(subbuf,&serialResponse[start],index-start);
				subbuf[index]='\0';
				printf("Read now: %s\n",subbuf);
				if(subbuf[0]=='{'){
					cJSON * root = cJSON_Parse(subbuf);
					printf("result json: \n");
					int xValue = cJSON_GetObjectItem(root,"x")->valueint;
					int yValue = cJSON_GetObjectItem(root,"y")->valueint;
					printf("x: %d\n",xValue);
					printf("y: %d\n",yValue);
					printf("^^^^^\n");
					writeLocationInput=0;
				}
			}

		}
		printf("\n");
	}

	char* toWrite = "sending from paparazzi---";
	write(READING_port->fd,toWrite,24);


 }



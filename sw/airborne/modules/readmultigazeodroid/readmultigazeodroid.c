/*
 * Copyright (C) Roland
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/readmultigazeodroid/readmultigazeodroid.c"
 * @author Roland
 * reads the multigaze data over serial
 */

#include "modules/readmultigazeodroid/readmultigazeodroid.h"
#include "modules/computer_vision/opticflow_module.h"
#include <termios.h>
#include <serial_port.h>
#include <stdio.h>
#include <inttypes.h>
#include <errno.h>
#include <string.h>
#include "/home/roland/paparazzi/sw/ext/cJSON/cJSON.h"


struct SerialPort *READING_port;
#ifndef ODROID_BAUDRATE
	#define ODROID_BAUDRATE B1000000
#endif

speed_t usbInputSpeed = ODROID_BAUDRATE;

int writeLocationInput=0;

#ifndef EXPECTED_WIDTH
	#define EXPECTED_WIDTH 36
#endif

#ifndef EXPECTED_HEIGHT
	#define EXPECTED_HEIGHT 6
#endif
#ifndef LENGTH_READ_BUFFER
	#define LENGTH_READ_BUFFER 50000
#endif
uint8_t lastMultigazeData[EXPECTED_WIDTH * EXPECTED_HEIGHT];
char serialResponse[LENGTH_READ_BUFFER];
void mgread_init() {
	// Open the serial port
	READING_port = serial_port_new();
	char namePort[50];
	for(int portExtension = 0; portExtension < 10; portExtension++){
		sprintf(namePort,"/dev/ttyUSB%d",portExtension);
		int result=serial_port_open_raw(READING_port,namePort,usbInputSpeed);
		if(result >=0){
			break;
		}
	}

	// Initialise the multigaze data to zeros
	int indexMultigazeData;
	for (indexMultigazeData = 0; indexMultigazeData < EXPECTED_WIDTH * EXPECTED_HEIGHT; indexMultigazeData++){
		lastMultigazeData[indexMultigazeData]=0;
	}

}
void mgread_periodic() {
	if (writeLocationInput > 10000)
		writeLocationInput = 0;
	int numberBytesRead = read(  READING_port->fd, &serialResponse[writeLocationInput], 3000);
	if (numberBytesRead < 0)
	{
		printf(strerror(errno));
		printf("\n");
	}
	else{
		writeLocationInput+=numberBytesRead;
		serialResponse[writeLocationInput]='\0';
		int index=0;
		int start=-1;
		for(index=0; index < writeLocationInput; index++){
			if(serialResponse[index]=='{'){
				start=index;
			}
			if(serialResponse[index]=='\n'&&start >=0)
			{
				int lengthMessage=index-start;
				char subbuf[lengthMessage];
				int indexPrint = 0;
				int indexSubbuf=0;

				// Copy to a new array
				for (indexPrint = start;indexPrint < index; indexPrint++){
					subbuf[indexSubbuf++]=serialResponse[indexPrint];
				}
				if(subbuf[0]=='{' && subbuf[lengthMessage-1]=='}'){ // Last check to see if this json message can be parsed

					cJSON * root = cJSON_Parse(subbuf);
					cJSON *array = cJSON_GetObjectItem(root,"mgdata");
					int mgWidth = cJSON_GetObjectItem(root,"mgwidth")->valueint;
					int mgHeight = cJSON_GetObjectItem(root,"mgheight")->valueint;
					if(mgWidth!= EXPECTED_WIDTH){
						printf("ERROR! WIDTH %d IS NOT EXPECTED",mgWidth);
						break;
					}
					if(mgHeight!= EXPECTED_HEIGHT){
						printf("ERROR! HEIGHT %d IS NOT EXPECTED",mgHeight);
						break;
					}
					int lengthArray = cJSON_GetArraySize(array);
					for (indexPrint = 0;indexPrint < lengthArray; indexPrint++){
						lastMultigazeData[indexPrint]=cJSON_GetArrayItem(array,indexPrint)->valueint;
					}
					printf("DONE! FOUND STUTFF\n");
					writeLocationInput=0;
				}

			}
		}
	}
 }






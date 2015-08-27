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
 * reads from the odroid and sends information of the drone to the odroid using JSON messages
 */

#include "modules/readlocationfromodroid/readlocationfromodroid.h"
#include "subsystems/abi.h"
#include "modules/computer_vision/opticflow_module.h"
#include <fcntl.h>
#include <stropts.h>
#include <termios.h>
#include <serial_port.h>
#include <stdio.h>
#include <inttypes.h>
#include <errno.h>
#include <string.h>
#include "state.h"
#include "subsystems/gps.h"
#include "cJSON.h"

#include "subsystems/ins/ins_int.h"
#include "subsystems/datalink/telemetry.h"


#include "firmwares/rotorcraft/autopilot.h"

#include "mcu_periph/uart.h"
#include "subsystems/radio_control.h"
#include "subsystems/commands.h"
#include "subsystems/actuators.h"
#include "subsystems/electrical.h"
#include "subsystems/settings.h"
#include "subsystems/datalink/telemetry.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance.h"

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_none.h"
#include "firmwares/rotorcraft/stabilization/stabilization_rate.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

#include "generated/settings.h"
int mustReset =1;
float test1=23.0;
float test2 = 24.0;
int timesRest = 100;
static void send_odometry(struct transport_tx *trans, struct link_device *dev)
{
    pprz_msg_send_ODOMETRY(trans, dev, AC_ID, &test1,&test2);
    //DOWNLINK_SEND_ODOMETRY (DefaultChannel, DefaultDevice,3.0,4.0);
}

struct SerialPort *READING_port;
speed_t usbInputSpeed = B115200;
char *serialResponse;
int writeLocationInput=0;


void odroid_loc_init() {
  register_periodic_telemetry(DefaultPeriodic, "ODOMETRY", send_odometry);
	// Open the serial port
	READING_port = serial_port_new();
	int result=serial_port_open_raw(READING_port,"/dev/ttyUSB0",usbInputSpeed);
	int lengthBytesImage=50000;//COMPLETE_MATRIX_WIDTH*MATRIX_ROWS;
	serialResponse=malloc(lengthBytesImage*sizeof(char));
	memset(serialResponse, '0', lengthBytesImage);
 }
 void odroid_loc_periodic() {
	printf("Loc periodic! %d", writeLocationInput);
	char justRead='a';
	if (writeLocationInput > 200)
		writeLocationInput = 0;
	int n = read(  READING_port->fd, &serialResponse[writeLocationInput], 100);
	//printf("Read %d bytes\n",n);
	if (n < 0)
	{
		printf(strerror(errno));
		printf("\n");
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
				if(subbuf[0]=='{' && subbuf[index-1]=='}'){
					cJSON * root = cJSON_Parse(subbuf);
					printf("result json: \n");
					float xValue = cJSON_GetObjectItem(root,"x")->valuedouble;
					float yValue = cJSON_GetObjectItem(root,"y")->valuedouble;

					double rot = cJSON_GetObjectItem(root,"rot")->valuedouble;
					printf("x: %d\n",xValue);
					printf("y: %d\n",yValue);
					printf("^^^^^\n");
					test1 = xValue;
					test2 = yValue;
					  gps.lla_pos.lat =519906108+ ((xValue*378.0)/4410.0);
					  gps.lla_pos.lon = 43768274+((yValue*267.3)/2915);

					  gps.lla_pos.alt = 45321;
					  gps.hmsl        = 125;
					  printf("Flow %f - %f\n",vel_x,vel_y);
					  gps.ecef_pos.x = 392433354+((xValue*378.0)/4410.0);
					  gps.ecef_pos.y = 30036451+ ((yValue*267.3)/2915);
					  gps.ecef_pos.z = 500219580;

					  gps.ecef_vel.x = vel_x;
					  gps.ecef_vel.y = vel_y;
					  gps.ecef_vel.z = 0;

					  gps.course = rot*100;
					  gps.num_sv = 11;
					  gps.tow = 0;
					  gps.fix = GPS_FIX_3D;

					  GpsFixValid();
				  // publish new GPS data
				    uint32_t now_ts = get_sys_time_usec();
				    gps.last_msg_ticks = sys_time.nb_sec_rem;
				    gps.last_msg_time = sys_time.nb_sec;
				    if (gps.fix == GPS_FIX_3D) {
				      gps.last_3dfix_ticks = sys_time.nb_sec_rem;
				      gps.last_3dfix_time = sys_time.nb_sec;
				    }
				    AbiSendMsgGPS(GPS_DATALINK_ID, now_ts, &gps);
					/*
					IvySendMsg("0 REMOTE_GPS %d %d %d %d %d %d %d %d %d %d %d %d %d %d", 201,
					      1,                //uint8 Number of markers (sv_num)
					      (int)(xValue),                //int32 ECEF X in CM
					      (int)(yValue),                //int32 ECEF Y in CM
					      (int)(23),                //int32 ECEF Z in CM
					      (int)(0),       //int32 LLA latitude in deg*1e7
					      (int)(0),       //int32 LLA longitude in deg*1e7
					      (int)(123),              //int32 LLA altitude in mm above elipsoid
					      (int)(30),         //int32 HMSL height above mean sea level in mm
					      (int)(20), //int32 ECEF velocity X in cm/s
					      (int)(10), //int32 ECEF velocity Y in cm/s
					      (int)(0), //int32 ECEF velocity Z in cm/s
					      0,
					      (int)(20));             //int32 Course in rad*1e7
*/
					writeLocationInput=0;
				}
			}

		}
		printf("\n");
	}
	cJSON *root, *droneInformation;
	root = cJSON_CreateObject();
	struct Int32Eulers *euler = stateGetNedToBodyEulers_i();

	cJSON_AddItemToObject(root, "droneInformation", droneInformation = cJSON_CreateObject());
	cJSON_AddNumberToObject(droneInformation, "phi", euler->phi);
	cJSON_AddNumberToObject(droneInformation, "theta", euler->theta);
	cJSON_AddNumberToObject(droneInformation, "psi", euler->psi);
	cJSON_AddNumberToObject(droneInformation, "height", state.enu_pos_f.z);
	cJSON_AddNumberToObject(droneInformation, "accel_z", ins_int.ltp_accel.z);
	cJSON_AddNumberToObject(droneInformation, "accel_x", ins_int.ltp_accel.x);
	cJSON_AddNumberToObject(droneInformation, "accel_y", ins_int.ltp_accel.y);

	cJSON_AddNumberToObject(root, "mustReset", mustReset);
	if (timesRest>0){
		timesRest-=1;
	}
	else{
		mustReset = 0;
	}
	char* toWrite = cJSON_PrintUnformatted(root);
	int lengthToWrite= strlen(toWrite);
	printf(toWrite);
	write(READING_port->fd,toWrite,lengthToWrite);
	write(READING_port->fd,"\n",1);

 }



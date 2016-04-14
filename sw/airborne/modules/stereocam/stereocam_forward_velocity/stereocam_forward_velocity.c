/*
 * Copyright (C) Roland
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/follow_me/follow_me.c"
 * @author Roland
 * follows a person on the stereo histogram image.
 * It searches for the highest peak and adjusts its roll and pitch to hover at a nice distance.
 */

#include "modules/stereocam/stereocam_forward_velocity/stereocam_forward_velocity.h"
#include "modules/stereocam/stereocam.h"
#include "state.h"
#include "navigation.h"
#include "subsystems/abi.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "subsystems/datalink/telemetry.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_int.h"
#include "subsystems/radio_control.h"
#include "math/pprz_algebra_int.h"


// for example use the standard horizontal (hover) mode // GUIDANCE_H_MODE_ATTITUDE // GUIDANCE_H_MODE_HOVER
#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_MODULE


// and own guidance_v
#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_MODULE

#define AVERAGE_VELOCITY 0
// Know waypoint numbers and blocks
#include "generated/flight_plan.h"
float ref_pitch=0.0;
float ref_roll=0.0;
#include "firmwares/rotorcraft/stabilization.h"

 struct Gains{
	 float pGain;
	 float dGain;
	 float iGain;
 };
typedef struct Gains gains;

gains stabilisationLateralGains;
gains forwardLateralGains;


int indexTurnFactors=0;


int disparity_velocity_step = 0;
int disparity_velocity_max_time = 500;
int distancesRecorded = 0;
int timeStepsRecorded = 0;

int indexVelocityHistory=0;
float sumVelocities=0.0;

float sumHorizontalVelocities=0.0;
uint8_t GO_FORWARD=0;
uint8_t STABILISE=1;
uint8_t TURN=2;
uint8_t current_state=2;
int totalStabiliseStateCount = 0;
int totalTurningSeenNothing=0;
float previousLateralSpeed = 0.0;
uint8_t detectedWall=0;
float velocityAverageAlpha = 0.65;
float previousHorizontalVelocity = 0.0;
float previousVerticalVelocity = 0.0;
//#define DANGEROUS_CLOSE_DISPARITY 24
int DANGEROUS_CLOSE_DISPARITY=80;
//#define CLOSE_DISPARITY 18
#define INIT_CLOSE_DISP 60
int CLOSE_DISPARITY=INIT_CLOSE_DISP;
//#define LOW_AMOUNT_PIXELS_IN_DROPLET 20
int LOW_AMOUNT_PIXELS_IN_DROPLET=20;
float ref_disparity_to_keep=INIT_CLOSE_DISP;
float pitch_compensation = 0.0;
float roll_compensation=0.0;
int initFastForwardCount = 0;
int goForwardXStages=3;
int counterStab=0;
float previousStabRoll=0.0;
float ref_alt=1.0;
typedef enum{USE_DROPLET,USE_CLOSEST_DISPARITY} something;
demo_type demonstration_type = HORIZONTAL_HOVER;
something wayToDetermineState = USE_CLOSEST_DISPARITY;
float headingStereocamStab=0.0;
float previousStabPitch=0.0;
uint8_t initialisedTurn=0;
uint8_t turnMultiplier=1;
int timeInStableMode=0;
void stereocam_forward_velocity_init()
{
	stabilisationLateralGains.pGain=0.6;
	stabilisationLateralGains.dGain=0.05;
	stabilisationLateralGains.iGain=0.01;
	forwardLateralGains.pGain=0.6;
}

void increase_nav_heading(int32_t *headingToChange, int32_t increment);
void increase_nav_heading(int32_t *headingToChange, int32_t increment)
{
  *headingToChange = *headingToChange + increment;
}

uint8_t dangerousClose(uint8_t closeValue){
	return closeValue>DANGEROUS_CLOSE_DISPARITY || closeValue==0;
}
uint8_t simplyClose(uint8_t closeValue){
	return closeValue>CLOSE_DISPARITY || closeValue==0;
}
int32_t previousThrust = 0;

void stereocam_forward_velocity_periodic()
{


  if (stereocam_data.fresh) {
    stereocam_data.fresh = 0;

    int timeStamp = 0;
    float noiseUs = 0.3f;
	ref_pitch=0.0;
    ref_roll=0.0;
    if((autopilot_mode != AP_MODE_NAV) && (autopilot_mode != AP_MODE_MODULE)){
    	 ref_alt= -state.ned_pos_f.z;
    	 current_state=STABILISE;
    	 headingStereocamStab=ANGLE_FLOAT_OF_BFP(INT32_DEG_OF_RAD(stab_att_sp_euler.psi));
    	 roll_compensation=ANGLE_FLOAT_OF_BFP(stab_att_sp_euler.phi);
    	 pitch_compensation=ANGLE_FLOAT_OF_BFP(stab_att_sp_euler.theta);
    	 previousThrust=stabilization_cmd[COMMAND_THRUST];
    	 totalStabiliseStateCount=0;
    }
    if(stereocam_data.data[1]>0){
    	float diff=stereocam_data.data[1]-128;
    	headingStereocamStab +=3.0*((diff)/128.0);

    }
    uint8_t closest = stereocam_data.data[0];

    while(headingStereocamStab>360.0){
    	headingStereocamStab-=360.0;
    }
    while(headingStereocamStab < 0.0){
    	headingStereocamStab+=360.0;
    }
    nav_set_heading_deg(headingStereocamStab);


    // Now keep distance
    ref_pitch = 0.0;
     ref_roll = 0.0;

     float stab_pitch_pgain = 0.04;
     float pitchDiff = closest - ref_disparity_to_keep;
     float pitchToTake = -0.08;//stab_pitch_pgain*pitchDiff;
     if (dangerousClose(closest)) {
       pitchToTake = 0.8;
     }
     else if (simplyClose(closest)) {
          pitchToTake = 0.20;
        }
     ref_pitch = 0.0;
     float max_roll = 0.25;
     float max_pitch_to_take = 0.25;

       if (pitchToTake > max_pitch_to_take) {
         ref_pitch = max_pitch_to_take;
       } else if (pitchToTake < -1 * max_pitch_to_take) {
         ref_pitch = -1 * max_pitch_to_take;
       } else {
         ref_pitch = pitchToTake;
       }



    ref_pitch += pitch_compensation;
    ref_roll += roll_compensation;
    float maxRefPitch = 0.25;
    if(ref_pitch>maxRefPitch){
		ref_pitch=maxRefPitch;
	}
	else if (ref_pitch<-1*maxRefPitch){
		ref_pitch=-1*maxRefPitch;
	}
  }
}















/**
 * Initialization of horizontal guidance module.
 */
void guidance_h_module_init(void)
{

}

/**
 * Horizontal guidance mode enter resets the errors
 * and starts the controller.
 */
void guidance_h_module_enter(void)
{

}

/**
 * Read the RC commands
 */
void guidance_h_module_read_rc(void)
{
  // TODO: change the desired vx/vy
}

/**
 * Main guidance loop
 * @param[in] in_flight Whether we are in flight or not
 */
void guidance_h_module_run(bool_t in_flight)
{
	struct Int32Eulers command;
	 float addedRollJoystick = ((float)radio_control.values[RADIO_ROLL])/((float)MAX_PPRZ);//RADIO_CONTROL_NB_CHANNEL
	    if(addedRollJoystick>0.25){
	    	addedRollJoystick=0.25;
		}
		else if (addedRollJoystick<-0.25){
			addedRollJoystick=-0.25;
		}


	    float addedPitchJoystick = ((float)radio_control.values[RADIO_PITCH])/((float)MAX_PPRZ);//RADIO_CONTROL_NB_CHANNEL
	       if(addedPitchJoystick>0.25){
	    	   addedPitchJoystick=0.25;
	   	}
	   	else if (addedPitchJoystick<-0.25){
	   		addedPitchJoystick=-0.25;
	   	}

	command.phi=ANGLE_BFP_OF_REAL(ref_roll+addedRollJoystick);
	command.theta=ANGLE_BFP_OF_REAL(ref_pitch+addedPitchJoystick);
	command.psi=INT32_RAD_OF_DEG(ANGLE_BFP_OF_REAL(headingStereocamStab));
	  /* Update the setpoint */
	  stabilization_attitude_set_rpy_setpoint_i(&command);

	  /* Run the default attitude stabilization */
  stabilization_attitude_run(in_flight);
}



// Implement own Horizontal loops
 void guidance_v_module_init(void){

 }
 void guidance_v_module_enter(void){

 }
 void guidance_v_module_read_rc(void){

 }
void guidance_v_module_run(bool_t in_flight){
	int32_t thrust = 0.80 * MAX_PPRZ;
	stabilization_cmd[COMMAND_THRUST] = previousThrust;
}


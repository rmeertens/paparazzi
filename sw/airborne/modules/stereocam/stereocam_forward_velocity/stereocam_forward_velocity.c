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
#include "generated/flight_plan.h"

#define AVERAGE_VELOCITY 0
#define DANGEROUS_CLOSE_DISPARITY 40
#define CLOSE_DISPARITY 33
#define DANGEROUS_CLOSE_DISTANCE 1.0
#define CLOSE_DISTANCE 2.0
#define LOW_AMOUNT_PIXELS_IN_DROPLET 30


float ref_pitch=0.0;
float ref_roll=0.0;

struct Gains{
	 float pGain;
	 float dGain;
	 float iGain;
 };
typedef struct Gains gains;

gains stabilisationLateralGains;
gains stabilisationForwardGains;
gains forwardLateralGains;


// Forward velocity estimation
#define LENGTH_VELOCITY_HISTORY 6
int disparity_velocity_step = 0;
int disparity_velocity_max_time = 500;
int distancesRecorded = 0;
int timeStepsRecorded = 0;
int velocity_disparity_outliers = 0;
float distancesHistory[500];
float timeStepHistory[500];
float velocityHistory[LENGTH_VELOCITY_HISTORY];
int indexVelocityHistory=0;
float sumVelocities=0.0;

float sumHorizontalVelocities=0.0;

avoidance_phase current_state=STABILISE;
demonstration_type demo_type = HORIZONTAL_STABLE;

int totalStabiliseStateCount = 0;
int totalTurningSeenNothing=0;
float previousLateralSpeed = 0.0;
uint8_t detectedWall=0;
float velocityAverageAlpha = 0.65;
float previousHorizontalVelocity = 0.0;

float ref_disparity_to_keep=25.0;
float pitch_compensation = 0.0;
float roll_compensation=0.0;
int initFastForwardCount = 0;
int goForwardXStages=3;
int counterStab=0;
float previousStabRoll=0.0;
float ref_alt=1.0;
typedef enum{USE_DROPLET,USE_CLOSEST_DISPARITY} avoid_strategy_type;
avoid_strategy_type avoidStrategy = USE_DROPLET;
float headingStereocamStab=0.0;
float someGainWhenDoingNothing=0.0;
const float max_roll_stab = 0.25;
float somePitchGainWhenDoingNothing=0.0;
float previousStabPitch=0.0;
int stabPositionCount=0;
float max_roll_forward=0.25;
float disparityDiffSum=0.0;
int timeInStabMode=0;
uint8_t initialisedTurnPhase=0;
int turnDirection=1;
void stereocam_forward_velocity_init()
{
	stabilisationLateralGains.pGain=0.5;
	stabilisationLateralGains.dGain=0.05;
	stabilisationLateralGains.iGain=0.01;
	forwardLateralGains.pGain=0.6;
	stabilisationForwardGains.pGain=0.3;
	stabilisationForwardGains.iGain=0.001;
}

void array_pop(float *array, int lengthArray);
void array_pop(float *array, int lengthArray)
{
  int index;
  for (index = 1; index < lengthArray; index++) {
    array[index - 1] = array[index];
  }
}

float calculateForwardVelocity(float distance,float alpha,int MAX_SUBSEQUENT_OUTLIERS,int n_steps_velocity);
float calculateForwardVelocity(float distance,float alpha,int MAX_SUBSEQUENT_OUTLIERS,int n_steps_velocity)
{
	    disparity_velocity_step += 1;
	    float new_dist = 0.0;
	    if (distancesRecorded > 0) {
	      new_dist = alpha * distancesHistory[distancesRecorded - 1] + (1 - alpha) * distance;
	    }
	    // Deal with outliers:
	    // Single outliers are discarded, while persisting outliers will lead to an array reset:
	    if (distancesRecorded > 0 && fabs(new_dist - distancesHistory[distancesRecorded - 1]) > 0.5) {
	      velocity_disparity_outliers += 1;
	      if (velocity_disparity_outliers >= MAX_SUBSEQUENT_OUTLIERS) {
	        // The drone has probably turned in a new direction
	        distancesHistory[0] = new_dist;
	        distancesRecorded = 1;

	        timeStepHistory[0] = disparity_velocity_step;
	        timeStepsRecorded = 1;
	        velocity_disparity_outliers = 0;
	      }
	    } else {
	        //append
	      velocity_disparity_outliers = 0;
	      timeStepHistory[timeStepsRecorded] = disparity_velocity_step;
	      distancesHistory[distancesRecorded] = new_dist;
	      distancesRecorded++;
	      timeStepsRecorded++;
	    }

	    //determine velocity (very simple method):
	    float velocityFound = 0.0;
	    if (distancesRecorded > n_steps_velocity) {
	      velocityFound = distancesHistory[distancesRecorded - n_steps_velocity] - distancesHistory[distancesRecorded - 1];
	    }
	    // keep maximum array size:
	    if (distancesRecorded > disparity_velocity_max_time) {
	    	array_pop(distancesHistory, disparity_velocity_max_time);
	    }
	    if (timeStepsRecorded > disparity_velocity_max_time) {
	    	array_pop(timeStepHistory, disparity_velocity_max_time);
	    }
	    return velocityFound;
}
void increase_nav_heading(int32_t *headingToChange, int32_t increment);
void increase_nav_heading(int32_t *headingToChange, int32_t increment)
{
  *headingToChange = *headingToChange + increment;
}

void boundAngle(float *angle, float maxAngle);
void boundAngle(float *angle, float maxAngle){
	if ((*angle) > maxAngle) {
		(*angle) = maxAngle;
	}
	else if ((*angle) < (-1.0 * maxAngle)) {
		(*angle) = -(1.0 * maxAngle);
	}
}

void forwardFunction(uint8_t closest, float dist, int disparitiesInDroplet,float guidoVelocityHor);
void forwardFunction(uint8_t closest, float dist, int disparitiesInDroplet,float guidoVelocityHor) {
	ref_pitch=-0.1;
	if(avoidStrategy==USE_CLOSEST_DISPARITY){
		if(dist < DANGEROUS_CLOSE_DISTANCE){
			ref_pitch=0.2;
			detectedWall=1;
		}
		else if(dist < CLOSE_DISTANCE){
			ref_pitch=0.1;
			detectedWall=1;
		}
	}
	else{
		if(disparitiesInDroplet>LOW_AMOUNT_PIXELS_IN_DROPLET){
			ref_pitch=0.1;
			detectedWall=1;
		}
	}

	float rollToTake = forwardLateralGains.pGain * guidoVelocityHor;
	rollToTake*=-1;
	if(counterStab%4==0){
		boundAngle(&rollToTake,max_roll_forward);
		ref_roll=rollToTake;
	}

	if(closest < DANGEROUS_CLOSE_DISPARITY && detectedWall&& closest>0){
		totalTurningSeenNothing=0;
		current_state=STABILISE;
		detectedWall=0;
		stabPositionCount=0;
	}
}

void turnFunction(uint8_t closest, int disparitiesInDroplet, uint8_t disparityLeft, uint8_t disparityRight);
void turnFunction(uint8_t closest, int disparitiesInDroplet, uint8_t disparityLeft, uint8_t disparityRight) {
	ref_pitch=0.0;
	ref_roll=0.0;
	if(!initialisedTurnPhase){
		if(disparityLeft>disparityRight){
			turnDirection =1;
		}
		else{
			turnDirection=-1;
		}
		initialisedTurnPhase=1;
		headingStereocamStab += turnDirection*35.0;
	}
	if(demo_type==DROPLET){
		headingStereocamStab += 180.0;
		current_state=INIT_FORWARD;
	}
	else{
		headingStereocamStab += turnDirection*5.0;

	}
	if (headingStereocamStab > 360.0){
	  headingStereocamStab -= 360.0;
  }

	if(avoidStrategy==USE_CLOSEST_DISPARITY){
		if(closest<CLOSE_DISPARITY && closest>0){
			totalTurningSeenNothing++;
			if(totalTurningSeenNothing>2){
				current_state=GO_FORWARD;
				detectedWall=0;
			}
		}
	}
	else{
		if(disparitiesInDroplet<LOW_AMOUNT_PIXELS_IN_DROPLET){
			totalTurningSeenNothing++;
			current_state=GO_FORWARD;
			detectedWall=0;
		}
	}
}

uint8_t isFartherThanGoal(uint8_t closest,float ref_disparity_to_keep){
	return closest > 0 && closest < ref_disparity_to_keep;
}
void stabilisationFunction(uint8_t closest, float guidoVelocityHor);
void stabilisationFunction(uint8_t closest, float guidoVelocityHor) {
	timeInStabMode++;
	float pitchDiff = closest - ref_disparity_to_keep;
	float pitchToTake = stabilisationForwardGains.pGain * pitchDiff;
	float rollToTake = stabilisationLateralGains.pGain * guidoVelocityHor;
	rollToTake *= -1;

	if (counterStab % 5 == 0) {
		boundAngle(&rollToTake,max_roll_stab);
		ref_roll=rollToTake;
		boundAngle(&pitchToTake,0.12);
		ref_pitch=pitchToTake;

		previousStabRoll = ref_roll;
		previousStabPitch = ref_pitch;
		someGainWhenDoingNothing += 0.1 * ref_roll;
		somePitchGainWhenDoingNothing += 0.1 * ref_pitch;
	} else {
		ref_pitch = 0.8 * previousStabPitch;
		ref_roll = 0.4 * previousStabRoll;
	}
	if (abs(closest - ref_disparity_to_keep) < 5) {
		stabPositionCount++;
	}
	else {
		stabPositionCount = 0;
	}

	if (demo_type == DROPLET){
		if(timeInStabMode>5){
			disparityDiffSum=0.0;
			current_state = TURN;
			stabPositionCount = 0;
			timeInStabMode=0;
		}
	}
	// Maybe we can start turning
	if (demo_type != HORIZONTAL_STABLE) {

		// Maybe we can go to turn phase
		if ((guidoVelocityHor < 0.25 && guidoVelocityHor > -0.25)||isFartherThanGoal( closest, ref_disparity_to_keep)){
			if (stabPositionCount > 10 || isFartherThanGoal( closest, ref_disparity_to_keep) || timeInStabMode>50) {
				disparityDiffSum=0.0;
				stabPositionCount = 0;
				timeInStabMode=0;
				// Set new mode
				current_state = TURN;
				initialisedTurnPhase=0;

			}
		}
	}
}

void stereocam_forward_velocity_periodic()
{

  if (stereocam_data.fresh && stereocam_data.len>20) {
    stereocam_data.fresh = 0;
	uint8_t closest = stereocam_data.data[4];

	uint8_t disparitiesInDroplet = stereocam_data.data[5];
    int horizontalVelocity = stereocam_data.data[8]-127;
    int upDownVelocity = stereocam_data.data[9] -127;
    uint8_t disparityLeft = stereocam_data.data[10] ;
    uint8_t disparityRight = stereocam_data.data[11];

    float  BASELINE_STEREO_MM = 60.0;
    float BRANDSPUNTSAFSTAND_STEREO = 118.0 * 6.0 * 2.0;
	float dist = 5.0;
	if (closest > 0) {
	  dist = ((BASELINE_STEREO_MM * BRANDSPUNTSAFSTAND_STEREO / (float)closest)) / 1000;
	}
	float velocityFound = calculateForwardVelocity(dist,0.65, 5,5);

    float guidoVelocityHorStereoboard = horizontalVelocity/100.0;
    float guidoVelocityHor = 0.0;

    // Set the velocity to either the average of the last few velocities, or take the current velocity with alpha times the previous one
    guidoVelocityHor = guidoVelocityHorStereoboard*velocityAverageAlpha + (1-velocityAverageAlpha)*previousHorizontalVelocity;
    sumHorizontalVelocities+=guidoVelocityHor;
    previousHorizontalVelocity= guidoVelocityHorStereoboard;

	ref_pitch=0.0;
    ref_roll=0.0;
    if(autopilot_mode != AP_MODE_NAV){
    	 ref_alt= -state.ned_pos_f.z;
    	 ref_disparity_to_keep=20.0;
    	 disparityDiffSum=0.0;
    	 initialisedTurnPhase=0;
    	 if(demo_type==HORIZONTAL_STABLE){
    		 current_state=STABILISE;
    	 }
    	 else{
    		 current_state=TURN;
    	 }
    	 headingStereocamStab=ANGLE_FLOAT_OF_BFP(INT32_DEG_OF_RAD(stab_att_sp_euler.psi));
    	 roll_compensation=ANGLE_FLOAT_OF_BFP(stab_att_sp_euler.phi);
    	 pitch_compensation=ANGLE_FLOAT_OF_BFP(stab_att_sp_euler.theta);
    }
    counterStab++;
    if(current_state==GO_FORWARD){
		forwardFunction(closest, dist,disparitiesInDroplet,guidoVelocityHor);
    }
    else if(current_state==STABILISE){
    	stabilisationFunction(closest, guidoVelocityHor);

     }
    else if(current_state==TURN){
    	turnFunction( closest, disparitiesInDroplet,disparityLeft,disparityRight);
    }
    else if(current_state==INIT_FORWARD){

    	ref_pitch=0.0;
        ref_roll=0.0;
    	// wait till turn completed
    	float currentHeading=ANGLE_FLOAT_OF_BFP(INT32_DEG_OF_RAD(stab_att_sp_euler.psi));
    	if(fabs(currentHeading-headingStereocamStab)<10){
    		current_state=GO_FORWARD;
    	}
    }
    else{
    	current_state=GO_FORWARD;
    }


    nav_set_heading_deg(headingStereocamStab);
    ref_pitch += pitch_compensation;
    ref_roll += roll_compensation;
    boundAngle(&ref_roll,0.2); boundAngle(&ref_pitch,0.2);
    DOWNLINK_SEND_STEREO_VELOCITY(DefaultChannel, DefaultDevice, &closest, &disparitiesInDroplet,&dist, &velocityFound,&guidoVelocityHor,&ref_disparity_to_keep,&current_state,&timeInStabMode,&disparityLeft,&disparityRight);
    DOWNLINK_SEND_REFROLLPITCH(DefaultChannel, DefaultDevice, &ref_roll,&ref_pitch);
  }
}

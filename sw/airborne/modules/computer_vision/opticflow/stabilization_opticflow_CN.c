/*
 * Copyright (C) 2014 Hann Woei Ho
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/computer_vision/opticflow/stabilization_opticflow.c
 * @brief Optical-flow based control for Linux based systems
 *
 * Control loops for optic flow based hovering.
 * Computes setpoint for the lower level attitude stabilization to control horizontal velocity.
 */

// Own Header
#include "stabilization_opticflow.h"

// Stabilization
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "autopilot.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/datalink/telemetry.h"
#include "subsystems/electrical.h"
#include "std.h"

//#ifndef CMD_OF_SAT
#define CMD_OF_SAT  1500 // 40 deg = 2859.1851
//#endif

#ifndef VISION_PHI_PGAIN
#define VISION_PHI_PGAIN 400
#endif
PRINT_CONFIG_VAR(VISION_PHI_PGAIN)

#ifndef VISION_PHI_IGAIN
#define VISION_PHI_IGAIN 20
#endif
PRINT_CONFIG_VAR(VISION_PHI_IGAIN)

#ifndef VISION_THETA_PGAIN
#define VISION_THETA_PGAIN 400
#endif
PRINT_CONFIG_VAR(VISION_THETA_PGAIN)

#ifndef VISION_THETA_IGAIN
#define VISION_THETA_IGAIN 20
#endif
PRINT_CONFIG_VAR(VISION_THETA_IGAIN)

#ifndef VISION_DESIRED_VX
#define VISION_DESIRED_VX 0
#endif
PRINT_CONFIG_VAR(VISION_DESIRED_VX)

#ifndef VISION_DESIRED_VY
#define VISION_DESIRED_VY 0
#endif
PRINT_CONFIG_VAR(VISION_DESIRED_VY)

/* Check the control gains */
#if (VISION_PHI_PGAIN < 0)      ||  \
  (VISION_PHI_IGAIN < 0)        ||  \
  (VISION_THETA_PGAIN < 0)      ||  \
  (VISION_THETA_IGAIN < 0)
#error "ALL control gains have to be positive!!!"
#endif

/* Initialize the default gains and settings */
struct opticflow_stab_t opticflow_stab = {
  .phi_pgain = VISION_PHI_PGAIN,
  .phi_igain = VISION_PHI_IGAIN,
  .theta_pgain = VISION_THETA_PGAIN,
  .theta_igain = VISION_THETA_IGAIN,
  .desired_vx = VISION_DESIRED_VX,
  .desired_vy = VISION_DESIRED_VY
};

float r_dot_new = 0;
float desired_vx = 0;
float desired_vy = 0;
float ref_pitch = 0;
float ref_roll = 0;
float speed_pot = 0;
float yaw_diff = 0;
float alpha_fil = 0.1;
float heading_target = 0;
float new_heading = 0;
float v_desired = 0.0;

int32_t yaw_rate = 0;
float yaw_rate_write = 0;
float yaw_ref_write = 0;
int32_t keep_yaw_rate = 0;
int32_t keep_turning = 0;

int8_t filter_flag = 0;    //0 =>no filter 1 =>Kalman filter 2 =>Butterworth filter
int8_t OA_method_flag = 4; //0 =>ping pong 2=>vector 1=>potentialfield 3=>Safetyzone 4=>No OA only opticflow

float err_vx = 0;
float err_vy = 0;

static void send_INPUT_CONTROL(void) {
  DOWNLINK_SEND_INPUT_CONTROL(DefaultChannel, DefaultDevice, &err_vx, &err_vy);
 }

/**
 * Horizontal guidance mode enter resets the errors
 * and starts the controller.
 */
void guidance_h_module_enter(void)
{
  /* Reset the integrated errors */
  opticflow_stab.err_vx_int = 0;
  opticflow_stab.err_vy_int = 0;

  /* Set rool/pitch to 0 degrees and psi to current heading */
  opticflow_stab.cmd.phi = 0;
  opticflow_stab.cmd.theta = 0;
  opticflow_stab.cmd.psi = stateGetNedToBodyEulers_i()->psi;
  
  new_heading = 0;
  
  register_periodic_telemetry(DefaultPeriodic, "INPUT_CONTROL", send_INPUT_CONTROL);
  
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
  int vsupply_scaled = electrical.vsupply*10;


  /* Update the setpoint */
  stabilization_attitude_set_rpy_setpoint_i(&opticflow_stab.cmd);

  /* Run the default attitude stabilization */
  stabilization_attitude_run(in_flight);
}

/**
 * Update the controls based on a vision result
 * @param[in] *result The opticflow calculation result used for control
 */
void stabilization_opticflow_update(struct opticflow_result_t *result)
{
      /* Check if we are in the correct AP_MODE before setting commands */
      if (autopilot_mode != AP_MODE_MODULE) {
	return;
      }
      
      if(OA_method_flag==0){
	    opticflow_stab.cmd.phi = ANGLE_BFP_OF_REAL(ref_roll);
	    opticflow_stab.cmd.theta = ANGLE_BFP_OF_REAL(ref_pitch);
      }
      else if(OA_method_flag==1){
	//TODO CHECK potential field method
		    if(TRUE){
	      //calculate new heading
	      //heading_temp = atan(result->vel_y/result->vel_x);
	      
	      //yaw_diff = alpha_fil*r_dot_new;
	      //new_heading = heading_target + yaw_diff; 
	      
	      new_heading = new_heading + alpha_fil*(r_dot_new-new_heading);
	      
	      desired_vx = sin(new_heading)*v_desired*100;
	      desired_vy = cos(new_heading)*v_desired*100;
	      opticflow_stab.desired_vx = sin(new_heading)*v_desired*100;
	      opticflow_stab.desired_vy = cos(new_heading)*v_desired*100;
	    }
	    else{
	      opticflow_stab.desired_vx = v_desired*100;
	      opticflow_stab.desired_vy = 0;
	    }
	    
	    /* Calculate the error if we have enough flow */
	    
	    float err_vx = 0;
	    float err_vy = 0;
	    if (result->tracked_cnt > 0) {
	      err_vx = opticflow_stab.desired_vx - result->vel_x;
	      err_vy = opticflow_stab.desired_vy - result->vel_y;
	    }

	    /* Calculate the integrated errors (TODO: bound??) */
	    opticflow_stab.err_vx_int += err_vx / 100;
	    opticflow_stab.err_vy_int += err_vy / 100;

	    /* Calculate the commands */
	    opticflow_stab.cmd.phi   = opticflow_stab.phi_pgain * err_vx / 100
				      + opticflow_stab.phi_igain * opticflow_stab.err_vx_int;
	    opticflow_stab.cmd.theta = -(opticflow_stab.theta_pgain * err_vy / 100
					+ opticflow_stab.theta_igain * opticflow_stab.err_vy_int);

	    /* Bound the roll and pitch commands */
	    BoundAbs(opticflow_stab.cmd.phi, CMD_OF_SAT);
	    BoundAbs(opticflow_stab.cmd.theta, CMD_OF_SAT);
      }
      else if(OA_method_flag==2){
	      float Total_Kan_x = ref_pitch;
	      float Total_Kan_y = ref_roll;

	      if((Total_Kan_x*Total_Kan_x+Total_Kan_y*Total_Kan_y)>1){
		  opticflow_stab.desired_vx = alpha_fil*Total_Kan_y + result->vel_x;
		  opticflow_stab.desired_vy = alpha_fil*Total_Kan_x + result->vel_y;
	      }
	      else{
		  opticflow_stab.desired_vx = 0;
		  opticflow_stab.desired_vy = 0;
	      }
	    /* Calculate the error if we have enough flow */

	    //alpha_fil needs to be tuned!
	    if (result->tracked_cnt > 0) {
	      err_vx = opticflow_stab.desired_vx - result->vel_x;
	      err_vy = opticflow_stab.desired_vy - result->vel_y;
	    }

	    /* Calculate the integrated errors (TODO: bound??) */
	    opticflow_stab.err_vx_int += err_vx / 100;
	    opticflow_stab.err_vy_int += err_vy / 100;

	    /* Calculate the commands */
	    opticflow_stab.cmd.phi   = opticflow_stab.phi_pgain * err_vx / 100
				      + opticflow_stab.phi_igain * opticflow_stab.err_vx_int;
	    opticflow_stab.cmd.theta = -(opticflow_stab.theta_pgain * err_vy / 100
					+ opticflow_stab.theta_igain * opticflow_stab.err_vy_int);

	    /* Bound the roll and pitch commands */
	    BoundAbs(opticflow_stab.cmd.phi, CMD_OF_SAT);
	    BoundAbs(opticflow_stab.cmd.theta, CMD_OF_SAT);
	
	
      }
      else if(OA_method_flag==3){
	//TODO safetyzone method
      }
      else if(OA_method_flag==4){
	    
	    /* Check if we are in the correct AP_MODE before setting commands */
	    if (autopilot_mode != AP_MODE_MODULE) {
	      return;
	    }

	    /* Calculate the error if we have enough flow */
	    //float err_vx = 0;
	    //float err_vy = 0;
	    if (result->tracked_cnt > 0) {
	      err_vx = opticflow_stab.desired_vx - result->vel_x;
	      err_vy = opticflow_stab.desired_vy - result->vel_y;
	    }

	    /* Calculate the integrated errors (TODO: bound??) */
	    opticflow_stab.err_vx_int += err_vx / 100;
	    opticflow_stab.err_vy_int += err_vy / 100;

	    /* Calculate the commands */
	    opticflow_stab.cmd.phi   = opticflow_stab.phi_pgain * err_vx / 100
				      + opticflow_stab.phi_igain * opticflow_stab.err_vx_int;
	    opticflow_stab.cmd.theta = -(opticflow_stab.theta_pgain * err_vy / 100
					+ opticflow_stab.theta_igain * opticflow_stab.err_vy_int);

	    /* Bound the roll and pitch commands */
	    BoundAbs(opticflow_stab.cmd.phi, CMD_OF_SAT);
	    BoundAbs(opticflow_stab.cmd.theta, CMD_OF_SAT);

      }
}

/*
 * Copyright (C) Ewoud Smeur <ewoud_smeur@msn.com>
 * MAVLab Delft University of Technology
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file stabilization_attitude_quat_indi.c
 * MAVLab Delft University of Technology
 * This control algorithm is Incremental Nonlinear Dynamic Inversion (INDI)
 *
 * This is a simplified implementation of the (soon to be) publication in the
 * journal of Control Guidance and Dynamics: Adaptive Incremental Nonlinear
 * Dynamic Inversion for Attitude Control of Micro Aerial Vehicles
 */

#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"

void stabilization_attitude_init(void)
{
  stabilization_indi_init();
}

void stabilization_attitude_enter(void)
{
  stabilization_indi_enter();
}

void stabilization_attitude_set_failsafe_setpoint(void)
{
  stabilization_indi_set_failsafe_setpoint();
}

void stabilization_attitude_set_rpy_setpoint_i(struct Int32Eulers *rpy)
{
  stabilization_indi_set_rpy_setpoint_i(rpy);
}

void stabilization_attitude_set_earth_cmd_i(struct Int32Vect2 *cmd, int32_t heading)
{
  // stab_att_sp_euler.psi still used in ref..
  stab_att_sp_euler.psi = heading;

  // compute sp_euler phi/theta for debugging/telemetry
  /* Rotate horizontal commands to body frame by psi */
  int32_t psi = stateGetNedToBodyEulers_i()->psi;
  int32_t s_psi, c_psi;
  PPRZ_ITRIG_SIN(s_psi, psi);
  PPRZ_ITRIG_COS(c_psi, psi);
  stab_att_sp_euler.phi = (-s_psi * cmd->x + c_psi * cmd->y) >> INT32_TRIG_FRAC;
  stab_att_sp_euler.theta = -(c_psi * cmd->x + s_psi * cmd->y) >> INT32_TRIG_FRAC;

  quat_from_earth_cmd_i(&stab_att_sp_quat, cmd, heading);
}

static void attitude_run_indi(int32_t indi_commands[], struct Int32Quat *att_err, bool_t in_flight __attribute__((unused)))
{
  //Calculate required angular acceleration
  struct FloatRates *body_rate = stateGetBodyRates_f();
#if STABILIZATION_INDI_FILTER_ROLL_RATE
  indi.angular_accel_ref.p = reference_acceleration.err_p * QUAT1_FLOAT_OF_BFP(att_err->qx)
                             - reference_acceleration.rate_p * indi.filtered_rate.p;
#else
  indi.angular_accel_ref.p = reference_acceleration.err_p * QUAT1_FLOAT_OF_BFP(att_err->qx)
                             - reference_acceleration.rate_p * body_rate->p;
#endif
#if STABILIZATION_INDI_FILTER_PITCH_RATE
  indi.angular_accel_ref.q = reference_acceleration.err_q * QUAT1_FLOAT_OF_BFP(att_err->qy)
                             - reference_acceleration.rate_q * indi.filtered_rate.q;
#else
  indi.angular_accel_ref.q = reference_acceleration.err_q * QUAT1_FLOAT_OF_BFP(att_err->qy)
                             - reference_acceleration.rate_q * body_rate->q;
#endif
#if STABILIZATION_INDI_FILTER_YAW_RATE
  indi.angular_accel_ref.r = reference_acceleration.err_r * QUAT1_FLOAT_OF_BFP(att_err->qz)
                             - reference_acceleration.rate_r * indi.filtered_rate.r;
#else
  float rate_ref_r = reference_acceleration.err_r * QUAT1_FLOAT_OF_BFP(att_err->qz)/reference_acceleration.rate_r;
  Bound(rate_ref_r, -RadOfDeg(STABILIZATION_INDI_MAX_R), RadOfDeg(STABILIZATION_INDI_MAX_R));

  indi.angular_accel_ref.r = reference_acceleration.rate_r * (rate_ref_r - body_rate->r);
#endif

  //Incremented in angular acceleration requires increment in control input
  //G1 is the actuator effectiveness. In the yaw axis, we need something additional: G2.
  //It takes care of the angular acceleration caused by the change in rotation rate of the propellers
  //(they have significant inertia, see the paper mentioned in the header for more explanation)
  indi.du.p = 1.0 / g1.p * (indi.angular_accel_ref.p - indi.filtered_rate_deriv.p);
  indi.du.q = 1.0 / g1.q * (indi.angular_accel_ref.q - indi.filtered_rate_deriv.q);
  indi.du.r = 1.0 / (g1.r + g2) * (indi.angular_accel_ref.r - indi.filtered_rate_deriv.r + g2 * indi.du.r);

  //add the increment to the total control input
  indi.u_in.p = indi.u.p + indi.du.p;
  indi.u_in.q = indi.u.q + indi.du.q;
  indi.u_in.r = indi.u.r + indi.du.r;

  //bound the total control input
  Bound(indi.u_in.p, -4500, 4500);
  Bound(indi.u_in.q, -4500, 4500);
  Bound(indi.u_in.r, -4500, 4500);

  //Propagate input filters
  //first order actuator dynamics
  indi.u_act_dyn.p = indi.u_act_dyn.p + STABILIZATION_INDI_ACT_DYN_P * (indi.u_in.p - indi.u_act_dyn.p);
  indi.u_act_dyn.q = indi.u_act_dyn.q + STABILIZATION_INDI_ACT_DYN_Q * (indi.u_in.q - indi.u_act_dyn.q);
  indi.u_act_dyn.r = indi.u_act_dyn.r + STABILIZATION_INDI_ACT_DYN_R * (indi.u_in.r - indi.u_act_dyn.r);

  //sensor filter
  stabilization_indi_second_order_filter(&indi.u_act_dyn, &indi.udotdot, &indi.udot, &indi.u,
                                         STABILIZATION_INDI_FILT_OMEGA, STABILIZATION_INDI_FILT_ZETA, STABILIZATION_INDI_FILT_OMEGA_R);

  //Don't increment if thrust is off
  if (stabilization_cmd[COMMAND_THRUST] < 300) {
    FLOAT_RATES_ZERO(indi.u);
    FLOAT_RATES_ZERO(indi.du);
    FLOAT_RATES_ZERO(indi.u_act_dyn);
    FLOAT_RATES_ZERO(indi.u_in);
    FLOAT_RATES_ZERO(indi.udot);
    FLOAT_RATES_ZERO(indi.udotdot);
  } else {
    lms_estimation();
  }

  /*  INDI feedback */
  indi_commands[COMMAND_ROLL] = indi.u_in.p;
  indi_commands[COMMAND_PITCH] = indi.u_in.q;
  indi_commands[COMMAND_YAW] = indi.u_in.r;
}

void stabilization_attitude_run(bool_t enable_integrator)
{
  stabilization_indi_run(enable_integrator, FALSE);
}

void stabilization_attitude_read_rc(bool_t in_flight, bool_t in_carefree, bool_t coordinated_turn)
{
  stabilization_indi_read_rc(in_flight, in_carefree, coordinated_turn);
}

/*
 * Copyright (C) Roland Meertens
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
 * @file "modules/replay_commands/replay_commands.c"
 * @author Roland Meertens
 * replays commands when switching from att to nav mode
 */

#include "modules/replay_commands/replay_commands.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_int.h"

// by default remember 40 seconds of command
#ifndef COMMANDS_MEMORY_LENGTH
#define COMMANDS_MEMORY_LENGTH 512*40
#endif

struct Int32Eulers  sp_memory[COMMANDS_MEMORY_LENGTH];
struct Int32Eulers to_set;
int record_position = 0;
int replay_position = 0;
replay_state_t current_state;

void replay_commands_init() {
	current_state=NONE;
}
void replay_commands_periodic() {
	if(current_state == RECORDING){
		if(record_position < COMMANDS_MEMORY_LENGTH){
			sp_memory[record_position++]=stab_att_sp_euler;

		}
		else{
			// what to do when buffer is full
		}
	}
	else if(current_state == REPLAYING){
		if(replay_position<COMMANDS_MEMORY_LENGTH && replay_position < record_position){
			to_set = sp_memory[replay_position++];


		}
		else{
			// what to do when replaying last part??
		}
	}
}

void guidance_h_module_init(void){

}
 void guidance_h_module_enter(void){
 }
 void guidance_h_module_read_rc(void){
 }
 void guidance_h_module_run(bool in_flight){
	 if(in_flight){
		 stabilization_attitude_set_rpy_setpoint_i(&to_set);
	 }
 }




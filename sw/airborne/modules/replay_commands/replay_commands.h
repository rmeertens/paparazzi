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
 * @file "modules/replay_commands/replay_commands.h"
 * @author Roland Meertens
 * replays commands when switching from att to nav mode
 */
#include "firmwares/rotorcraft/guidance/guidance_module.h"

#ifndef REPLAY_COMMANDS_H
#define REPLAY_COMMANDS_H
typedef enum{RECORDING,REPLAYING,NONE} replay_state_t;
extern replay_state_t current_state;
extern void replay_commands_init(void);
extern void replay_commands_periodic(void);
void guidance_h_module_init(void);
 void guidance_h_module_enter(void);
 void guidance_h_module_read_rc(void);
 void guidance_h_module_run(bool);


#endif


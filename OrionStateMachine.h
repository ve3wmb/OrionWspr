#ifndef ORIONSTATEMACHINE_H
#define ORIONSTATEMACHINE_H
/*
    OrionStateMachine.h - Definitions for control of Orion WSPR Beacon

   Copyright (C) 2018-2019 Michael Babineau <mbabineau.ve3wmb@gmail.com>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
enum OrionState {POWERUP_ST, STARTUP_CALIBRATE_ST, CALIBRATE_ST, WAIT_TELEMETRY_ST, TELEMETRY_ST, WAIT_TX_PRIMARY_WSPR_ST, 
                 TX_PRIMARY_WSPR_ST, WAIT_TX_SECONDARY_WSPR_ST, TX_SECONDARY_WSPR_ST};
                 
enum OrionEvent {NO_EV,SETUP_DONE_EV, CALIBRATION_FAIL_EV, CALIBRATION_DONE_EV, TELEMETRY_TIME_EV, TELEMETRY_DONE_EV,
                 PRIMARY_WSPR_TX_TIME_EV, PRIMARY_WSPR_TX_DONE_EV, SECONDARY_WSPR_TX_TIME_EV, SECONDARY_WSPR_TX_DONE_EV};

enum OrionAction {NO_ACTION, CALIBRATION_ACTION, GET_TELEMETRY_ACTION, TX_WSPR_MSG1_ACTION, TX_WSPR_MSG2_ACTION, STARTUP_CALIBRATION_ACTION, WSPR_TX_INT_SETUP_ACTION}; 

void orion_sm_begin();

OrionState orion_sm_get_current_state();
OrionAction orion_state_machine(OrionEvent event);                                 
#endif

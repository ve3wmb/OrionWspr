/*
   OrionStateMachine.cpp - Control Logic for the Orion WSPR Beacon
   This file implements a state machine.

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
#include "OrionSerialMonitor.h"
#include "OrionStateMachine.h"


OrionState g_current_orion_state = POWERUP_ST;
OrionState g_previous_orion_state = POWERUP_ST;
OrionEvent g_current_orion_event = NO_EV;
OrionEvent g_previous_orion_event = NO_EV;


void orion_sm_no_op () {
  // Placeholder
}

// State Machine Initializationto be called once in setup()
void orion_sm_begin() {
  // Start out in the initial state
  g_current_orion_state = POWERUP_ST;
  g_previous_orion_state = POWERUP_ST;
}


void orion_sm_change_state( OrionState new_state) {
  g_previous_orion_state = g_current_orion_state;
  g_current_orion_state = new_state;
}


// This is the event processor that implements the core of the Orion State Machine
// It returns an Action of type OrionAction to trigger work. 
OrionAction orion_state_machine(OrionEvent event) {
  
  OrionAction next_action = NO_ACTION; // Always default to NO_ACTION
  g_current_orion_event = event;

  // Pre_sm trace logging
  orion_sm_trace_pre(byte(g_current_orion_state), event); 
  
  switch (g_current_orion_state) {

    case  POWERUP_ST : { //executing setup()

        if (event == SETUP_DONE_EV) {
          // Done setup now wait until it is time to TX the Primary WSPR Msg
          orion_sm_change_state(WAIT_TX_PRIMARY_WSPR_ST);
          next_action = NO_ACTION;
        }
        else
          swerr(1, event); // This event is not supported in this state
        break;
      }

    /*
      case  WAIT_CALIBRATE_ST : // waiting for next calibration cycle
             orion_sm_no_op();

      case  CALIBRATE_ST :  // calibrating Si5351a and ATMega328p clocks
             orion_sm_no_op();

      case   WAIT_TELEMETRY_ST : // Waiting for telemetry gathering cycle
             orion_sm_no_op();

      case  TELEMETRY_ST :  // gathering/calculating telemetry data
             orion_sm_no_op();
    */

    case  WAIT_TX_PRIMARY_WSPR_ST : { // waiting for Primary WSPR Msg TX Window

        if (event == PRIMARY_WSPR_TX_TIME_EV) { // Time to send Primary WSPR MSG
          orion_sm_change_state(TX_PRIMARY_WSPR_ST);
          next_action = TX_WSPR_MSG1_ACTION; // Start Transmitting the Primary WSPR Message
        }
        else
         // swerr(2, event); // This event is not supported in this state
        break;
      }

    case  TX_PRIMARY_WSPR_ST : { // transmitting Primary WSPR Msg
      
        if (event == PRIMARY_WSPR_TX_DONE_EV) { // Primary WSPR Transmission Complete
          orion_sm_change_state(WAIT_TX_PRIMARY_WSPR_ST);
          next_action = NO_ACTION; 
        }
        else
          orion_sm_no_op();
        break;
      }

    /*
      case  WAIT_TX_SECONDARY_WSPR_ST : // waiting for Secondary WSPR Msg TX Window
              orion_sm_no_op();

      case  TX_SECONDARY_WSPR_ST : // transmitting Secondary WSPR Msg
              orion_sm_no_op();
    */

    default : orion_sm_no_op(); // If we end up here it is an error as we have and unimplemented state.

  } // end switch

  g_previous_orion_event = event;
  g_current_orion_event = NO_EV;

  // Post_sm trace logging 
  orion_sm_trace_post(byte(g_current_orion_state), event, next_action);

  return next_action; 

}

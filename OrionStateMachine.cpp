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
#include "OrionBoardConfig.h"


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

OrionState orion_sm_get_current_state(){
  return g_current_orion_state;
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

    case  POWERUP_ST :  //executing setup()
              
        if (event == WAIT_VOLTAGE_EV){
          orion_sm_change_state(WAIT_OP_VOLTAGE_ST);
          next_action = OP_VOLT_WAITLOOP_ACTION;
        }
        else 
          swerr(1, event); // This event is not supported in this state
        
        break;

    case  CALIBRATE_ST :  // calibrating Si5351a clock
    
      if ( (event == CALIBRATION_DONE_EV) || (event == CALIBRATION_FAIL_EV) ) {
        orion_sm_change_state(WAIT_TELEMETRY_ST);
        next_action = NO_ACTION;
      }
      else { 
        if ( (event == GPS_LOS_TIMEOUT_EV) || (event == STARTUP_CALIBRATION_FAIL_EV)  ) { 
          // We exceeded the guard time for GPS LOS or we failed startup calibration so we switch over to QRSS transmissions
          orion_sm_change_state(QRSS_TX_ST);
          next_action = QRSS_TX_ACTION; // Initiate QRSS Transmission Mode until GPS AOS  
        }
        else
        
          swerr(9, event); // Unexpected event in this state
      }
        
      break;


    case   WAIT_TELEMETRY_ST :  // Waiting for telemetry gathering cycle
      if (event == TELEMETRY_TIME_EV) {
        // Time to get the Telemetery info
        orion_sm_change_state(TELEMETRY_ST);
        next_action = GET_TELEMETRY_ACTION;
      }
      else
        swerr(2, event); // This event is not supported in this state
      break;



    case  TELEMETRY_ST :  // done gathering/calculating telemetry data
      if (event == TELEMETRY_DONE_EV) {
        orion_sm_change_state(WAIT_TX_PRIMARY_WSPR_ST);
        next_action = NO_ACTION;
      }
      else if (event == LOW_VOLTAGE_EV) { // Measure VCC below operating threshhold
        orion_sm_change_state(SHUTDOWN_ST);
        next_action = INITIATE_SHUTDOWN_ACTION;
      }
      else
        swerr(3, event); // This event is not supported in this state
      break;


    case  WAIT_TX_PRIMARY_WSPR_ST :  // waiting for Primary WSPR Msg TX Window

      if (event == PRIMARY_WSPR_TX_TIME_EV) { // Time to send Primary WSPR MSG
        orion_sm_change_state(TX_PRIMARY_WSPR_ST);
        next_action = TX_WSPR_MSG1_ACTION; // Start Transmitting the Primary WSPR Message
      }
      else
        swerr(4, event); // This event is not supported in this state
      break;


    case  TX_PRIMARY_WSPR_ST : // transmitting Primary WSPR Msg
      if (event == PRIMARY_WSPR_TX_DONE_EV) { // Primary WSPR Transmission Complete
          orion_sm_change_state(WAIT_TX_SECONDARY_WSPR_ST);
          next_action = NO_ACTION;    
      }
      else
        swerr(13, event); // This event is not supported in this state
      break;
          
   
      case  WAIT_TX_SECONDARY_WSPR_ST : // waiting for Secondary WSPR Msg TX Window
      
              switch (event) {
                case WSPR_TX_TIME_MIN02_EV :
                  orion_sm_change_state(TX_SECONDARY_WSPR_ST);
                  next_action = TX_WSPR_MIN02_ACTION;
                  break;
                
                case WSPR_TX_TIME_MIN12_EV :
                  orion_sm_change_state(TX_SECONDARY_WSPR_ST);
                  next_action = TX_WSPR_MIN12_ACTION;
                  break;
                
                case WSPR_TX_TIME_MIN22_EV :
                  orion_sm_change_state(TX_SECONDARY_WSPR_ST);
                  next_action = TX_WSPR_MIN22_ACTION;
                  break;
                  
                case WSPR_TX_TIME_MIN32_EV :                   
                  orion_sm_change_state(TX_SECONDARY_WSPR_ST);
                  next_action = TX_WSPR_MIN32_ACTION;
                  break;
                  
                case WSPR_TX_TIME_MIN42_EV :
                  orion_sm_change_state(TX_SECONDARY_WSPR_ST);
                  next_action = TX_WSPR_MIN42_ACTION;
                  break;
                  
                case WSPR_TX_TIME_MIN52_EV :
                  orion_sm_change_state(TX_SECONDARY_WSPR_ST);
                  next_action = TX_WSPR_MIN52_ACTION;
                  break;
                
                default : 
                  swerr(12, event); // This event is not supported in this state
         
              } // end switch(event)
              break;   
           

      case  TX_SECONDARY_WSPR_ST : // transmitting Secondary WSPR Msg
       if (event == SECONDARY_WSPR_TX_DONE_EV) { // Secondary WSPR Transmission Complete

          if ((SI5351_SELF_CALIBRATION_SUPPORTED == true) && (is_selfcalibration_on())) {
            // Now we initiate a four minute Calibration Cycle for the next transmission
            orion_sm_change_state(CALIBRATE_ST);
            next_action = CALIBRATION_ACTION;
          }
          else {
            // If we don't support self Calibration then skip to telemetry
            orion_sm_change_state(WAIT_TELEMETRY_ST);
            next_action = NO_ACTION;
          }

        }
        else
          swerr(5, event); // This event is not supported in this state
        break;

    case WAIT_OP_VOLTAGE_ST : // Waiting to reach operational voltage 
       if (event == SETUP_DONE_EV) {

          if ((SI5351_SELF_CALIBRATION_SUPPORTED == true) && (is_selfcalibration_on())) {
            // Done setup now do intial calibration
            orion_sm_change_state(CALIBRATE_ST);
            next_action = STARTUP_CALIBRATION_ACTION;
          }
          else {
            // If we don't support self Calibration then skip to telemetry
            orion_sm_change_state(WAIT_TELEMETRY_ST);
            
            // Since we can't rely on the post-calibration cleanup to setup and 
            // start the WS TX Interrupt we trigger it using an action. 
            next_action = WSPR_TX_INT_SETUP_ACTION;
          }
          
        } // SETUP_DONE_EV 
        else
          swerr(15, event);
        break;
        
    case QRSS_TX_ST :
      if (event == QRSS_TX_DONE_EV) {
        orion_sm_change_state(CALIBRATE_ST);
        next_action = CALIBRATION_ACTION;  
      }
      break; 
      
    default : 
        swerr(6, g_current_orion_state); // If we end up here it is an error as we have and unimplemented state.
        break;
    

  } // end switch

  g_previous_orion_event = event;
  g_current_orion_event = NO_EV;

  // Post_sm trace logging
  orion_sm_trace_post(byte(g_current_orion_state), event, next_action);

  return next_action;

}

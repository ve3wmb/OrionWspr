/*
 * OrionSerialMonitor.cpp - Simple Software Serial system monitor for Orion WSPR Beacon
 *
 * Copyright 2019 Michael Babineau, VE3WMB <mbabineau.ve3wmb@gmail.com>
 *                          
 * This sketch  is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License.
 * If not, see <http://www.gnu.org/licenses/>.
*/
#include <Arduino.h>
#include "OrionSerialMonitor.h"
#include "OrionXConfig.h"
#include "OrionBoardConfig.h"
#include <TimeLib.h>


#if defined (DEBUG_USES_SW_SERIAL)
  #include <NeoSWSerial.h>
#endif

// This is needed if an application used both PinChangeInterrupts and NeoSWSerial which we do.
// Note that you must modify this to define an ISR for the correct PCINTx_vect 
// according to what pins you are using for SOFT_SERIAL_RX_PIN and SOFT_SERIAL_TX_PIN
//
// Port B (digital pin 8 to 13)
// Port C (all analog input pins)
// Port D (digital pins 0 to 7) 
//
// ISR (PCINT0_vect) handles pin change interrupt for D8 to D13, (Port B, PCINT0 - PCINT7)
// ISR (PCINT1_vect) handles pin change interrupt for A0 to A5, (Port C, PCINT8 - PCINT14)
// ISR (PCINT2_vect) handles pin change interrupt for D0 to D7, (Port D, PCINT16 - PCINT23)

// ISR(PCINT0_vect){}    // Port B, PCINT0 - PCINT7
// ISR(PCINT1_vect){}    // Port C, PCINT8 - PCINT14
// ISR(PCINT2_vect){}    // Port D, PCINT16 - PCINT23
//  PCINT0_vect must call NeoSWSerial::rxISR(PINB);
//  PCINT1_vect must call NeoSWSerial::rxISR(PINC);
//  PCINT2_vect must call NeoSWSerial::rxISR(PIND);
// Also note that you must uncomment the third last line in NeoSWSerial.h
// which is #define NEOSWSERIAL_EXTERNAL_PCINT, othewise you will have linking errors

#if defined (NEOSWSERIAL_EXTERNAL_PCINT)
  ISR (PCINT0_vect)
  {
    // handle pin change interrupt for D8 to D13 here. 
    // Note SOFT_SERIAL_TX_PIN (MOSI) = 11 and SOFT_SERIAL_RX_PIN =  = 12  (MISO - PB4/PCINT4)for U3S Clones
    // 
    NeoSWSerial::rxISR(PINB);
  }  // end of PCINT0_vect

  ISR (PCINT2_vect)
  {
    NeoSWSerial::rxISR(PIND);
  }  // end of PCINT2_vect
#endif

static bool g_debug_on_off = OFF; 
static bool g_txlog_on_off = ON;
static bool g_info_log_on_off = ON;
static bool g_qrm_avoidance_on_off = ON;
static bool g_selfcalibration_on_off = ON;
 
#if defined (DEBUG_USES_SW_SERIAL)  
  NeoSWSerial debugSerial(SOFT_SERIAL_RX_PIN, SOFT_SERIAL_TX_PIN);  // RX, TX
#else
  #define debugSerial Serial
#endif

void enable_qrm_avoidance() {
  g_qrm_avoidance_on_off = ON;
}

void disable_qrm_avoidance(){
  g_qrm_avoidance_on_off = OFF;
}

bool is_selfcalibration_on(){
  if (g_selfcalibration_on_off == OFF)
    return false;
  else
    return true;
}

bool is_qrm_avoidance_on(){
  if (g_qrm_avoidance_on_off == OFF)
    return false;
  else
    return true;
}
void print_monitor_prompt(){
  debugSerial.print(F("> "));
}

void print_date_time() {
  debugSerial.print(year());
  debugSerial.print(F("-"));
  debugSerial.print(month());
  debugSerial.print(F("-"));
  debugSerial.print(day());
  debugSerial.print(F(" "));
  debugSerial.print(hour());
  debugSerial.print(F(":"));
  debugSerial.print(minute());
  debugSerial.print(F(":"));
  debugSerial.print(second());
  debugSerial.print(F(" "));

}

bool toggle_on_off(bool flag){
  bool return_flag = ON;
  
  if (flag == ON){
    return_flag = OFF;
    debugSerial.println(F(" OFF"));
  }
  else{
    debugSerial.println(F(" ON"));
  }
  return return_flag;
}

// Log a software error. 
// swerr_num is unique number 1-255 assigned sequentially (should be unique for each call to swerr). 
void swerr(byte swerr_num, int data){
  print_date_time();
  debugSerial.print(F("***SWERR: "));
  debugSerial.print(swerr_num);
  debugSerial.print(F(" data dump in hex: "));
  debugSerial.println(data, HEX); 
  print_monitor_prompt(); 
    
}
void println_cmd_list(){
  debugSerial.println(F("cmds: v = f/w version, d = debug trace on/off, l = TX log on/off, i= info on/off, q = qrm avoidance on/off, ? = cmd list"));
} 



void orion_sm_trace_pre(byte state, byte event){
  
  if (g_debug_on_off == OFF) return;
  
  print_date_time();
  debugSerial.print(F(">> orion PRE sm trace: "));
  debugSerial.print(F("curr_state: "));
  debugSerial.print(state);
  debugSerial.print(F(" curr_event: "));
  debugSerial.println(event);
  print_monitor_prompt();  
}

void orion_sm_trace_post(byte state, byte processed_event,  byte resulting_action){
  
  if (g_debug_on_off == OFF) return;
  
  print_date_time();
  debugSerial.print(F("<< orion POST sm trace: "));
  debugSerial.print(F("curr_state: "));
  debugSerial.print(state);
  debugSerial.print(F(" event_just_processed: "));
  debugSerial.print(processed_event);
  debugSerial.print(F(" action: "));
  debugSerial.println(resulting_action);
  print_monitor_prompt();
    
}

void orion_log_wspr_tx(OrionWsprMsgType msgType, char grid[], unsigned long freq_hz, uint8_t pwr_dbm){

  // If either txlog is turned on or info logs are turned on then log the TX
  if ((g_txlog_on_off == OFF) && (g_info_log_on_off == OFF)) return; 
  
  print_date_time();

  switch (msgType) {
    
    case PRIMARY_WSPR_MSG :
      debugSerial.print(F("Primary WSPR TX - Grid: "));
      debugSerial.print(grid);
      break;

    case ALTITUDE_TELEM_MSG :
      debugSerial.print(F("Telemetry WSPR TX - ALT-> "));
      break;
      
    case TEMPERATURE_TELEM_MSG :
      debugSerial.print(F("Telemetry WSPR TX - TEMP-> "));
      break;
      
    case VOLTAGE_TELEM_MSG :
      debugSerial.print(F("Telemetry WSPR TX - VOLT-> "));
      break;

    default :
      break; 
    
  }
    
  debugSerial.print(F(" Pwr/dBm field:"));
  debugSerial.print(pwr_dbm);
  debugSerial.print(F(", Freq Hz: "));
  debugSerial.println(freq_hz);
  print_monitor_prompt();
  
}

void orion_log_telemetry (struct OrionTxData *data) {
  
  // If either txlog is turned on or info logs are turned on then log the TX
  if ((g_txlog_on_off == OFF) && (g_info_log_on_off == OFF)) return; 
  
  print_date_time();
  debugSerial.print(F("Telem Grid:"));
  debugSerial.print( (*data).grid_sq_6char );
  debugSerial.print(F(", alt_m:"));
  debugSerial.print( (*data).altitude_m);
  debugSerial.print(F(", spd_kn:"));
  debugSerial.print( (*data).speed_kn);
  debugSerial.print(F(", num_sats:"));
  debugSerial.print( (*data).number_of_sats);
  debugSerial.print(F(", gps_stat:"));
  debugSerial.print( (*data).gps_status);
  //debugSerial.print(F(", gps_fix: "));
  //debugSerial.print( (*data).gps_3d_fix_ok_bool);
  debugSerial.print(F(", batt_v_x10:"));
  debugSerial.print( (*data).battery_voltage_v_x10);
  debugSerial.print(F(", ptemp_c:"));
  debugSerial.print( (*data).processor_temperature_c);
  debugSerial.print(F(", temp_c:"));
  debugSerial.println( (*data).temperature_c);
  
  
  print_monitor_prompt();

}

void log_debug_Timer1_info(byte it,int ofCount, int t_count){
  
  if (g_debug_on_off == OFF) return; // Do nothing if debug disabled.
  
  debugSerial.print(F(" iteration: "));
  debugSerial.print(it);
  debugSerial.print(F(" overflowCounter: "));
  debugSerial.print(ofCount);
  debugSerial.print(F(" TCNT1: "));
  debugSerial.println(t_count);
  print_monitor_prompt();
}

// This is intended as a ligthweight notification of the start of calibration without all
// of the gory details that log calibration provides. It is only logged if txlog is ON
void log_calibration_start() {
  if (g_txlog_on_off == OFF) return; 
  
  print_date_time();
  debugSerial.println(F(" ** running CALIBRATION for 4 minutes **"));
  print_monitor_prompt();
} 

void log_calibration(uint64_t sampled_freq, int32_t o_cal_factor, int32_t n_cal_factor)
{ 
  if (g_info_log_on_off == OFF) return;
  
  uint32_t low = sampled_freq % 0xFFFFFFFF; 
  uint32_t high = (sampled_freq >> 32) % 0xFFFFFFFF;

  print_date_time();
  debugSerial.print(F(" Sampled Freq Hz x 100 : "));
  debugSerial.print(high);
  debugSerial.print(low);
  debugSerial.print(F(" Old Corr factor : "));
  debugSerial.print(o_cal_factor);
  debugSerial.print(F(" New Corr factor : "));
  debugSerial.println(n_cal_factor);
  print_monitor_prompt();
  
}
void log_time_set(){
  // If info logs are turned on then log the timeset
  if  (g_info_log_on_off == OFF) return;
  
  print_date_time();
  debugSerial.println(F(" ** Info: System Time set from GPS ** "));
  print_monitor_prompt();  
}

void log_shutdown(uint8_t voltagex10) {
  // If either txlog is turned on or info logs are turned on then log the shutdown
  if ((g_txlog_on_off == OFF) && (g_info_log_on_off == OFF)) return; 


  print_date_time();
  debugSerial.print(F(" ****** Controlled System Shutdown - low VCC(x10): "));
  debugSerial.println(voltagex10);
}

void log_qrss_tx_start(QrssMode mode, QrssSpeed spd){
   if ((g_txlog_on_off == OFF) && (g_info_log_on_off == OFF)) return;
   
   print_date_time();
   debugSerial.print(F("QRSS TX Start - QrssMode: "));
   debugSerial.print(mode);  
   debugSerial.print(F(" Speed: "));
   debugSerial.println(spd);
   print_monitor_prompt();  
}

void log_qrss_tx_end(){
  if ((g_txlog_on_off == OFF) && (g_info_log_on_off == OFF)) return; 

   print_date_time();
   debugSerial.println(F("QRSS TX Complete "));
   print_monitor_prompt();
 }

 void log_calibration_fail() {
  if (g_info_log_on_off == OFF) return;

  print_date_time();
  debugSerial.println(F(" ** Info: Calibration Fail ** "));
  print_monitor_prompt();
  
 }
/**********************
/* Serial Monitor code 
/**********************/
void print_board_and_version(){
  debugSerial.print(F("Orion firmware version: "));
  debugSerial.print(ORION_FW_VERSION);
  debugSerial.println(BOARDNAME);
}

void serial_monitor_begin(){
  
  // Start software serial port 
  debugSerial.begin(MONITOR_SERIAL_BAUD);
  debugSerial.print(F("Initialising Orion Serial Monitor.... "));
  print_board_and_version();
  println_cmd_list();
  print_monitor_prompt();
  delay(500);
}

static void flush_input(void){

  while (debugSerial.available() > 0)
    debugSerial.read();
}

void serial_monitor_interface(){
 
  if (debugSerial.available() > 0){
    char c = debugSerial.read();
    
    debugSerial.println(c); // echo the typed character
    
    switch (c) {
      
      case 'v' :
        flush_input();
        print_board_and_version();
        break;

       case 'd' : // toggle debug flag
       flush_input();
       debugSerial.print(F("Orion debug tracing is : "));
       g_debug_on_off = toggle_on_off(g_debug_on_off);
       break;

       case 'c' : // toggle calibration flag
       flush_input();
       debugSerial.print(F("Orion self calibration is : "));
       g_selfcalibration_on_off = toggle_on_off(g_selfcalibration_on_off);
       break;

       case 'l' : // toggle TX log flag
       flush_input();
       debugSerial.print(F("Orion TX log is : "));
       g_txlog_on_off = toggle_on_off(g_txlog_on_off);
       break;

       case 'h': case '?' : // list commands
       flush_input();
       println_cmd_list();
       break;

       case'q' :
       flush_input();
       debugSerial.print(F("Orion QRM avoidance is : "));
       g_qrm_avoidance_on_off = toggle_on_off(g_qrm_avoidance_on_off);
       break;

       case'i' :
       flush_input();
       debugSerial.print(F("Orion info logs are : "));
       g_info_log_on_off = toggle_on_off(g_info_log_on_off);
       break;
               
      default:
        flush_input();
        debugSerial.println(F(" -- unrecognized command"));
        println_cmd_list();
       
        // Do nothing
    }
    print_monitor_prompt();
  }
 
}

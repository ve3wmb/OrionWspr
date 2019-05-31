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
#define OFF false
#define ON true
#if defined (DEBUG_USES_SW_SERIAL)
  #include <NeoSWSerial.h>
#endif

static bool g_debug_on_off = OFF;
static bool g_txlog_on_off = OFF;
static bool g_qrm_avoidance_on_off = ON;
 
#if defined (DEBUG_USES_SW_SERIAL)  
  NeoSWSerial debugSerial(SOFT_SERIAL_RX_PIN, SOFT_SERIAL_TX_PIN);  // RX, TX
#else
  #define debugSerial Serial
#endif

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
  debugSerial.println(F("cmds: v = f/w version, d = toggle debug trace on/off, l = toggle TX log on/off, q = toggle qrm avoidance on/off, ? = cmd list"));
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

void orion_log_wspr_tx(OrionWsprMsgType msgType, char grid[], unsigned long freq_hz){
  if (g_txlog_on_off == OFF) return; 
  
  print_date_time();
  debugSerial.print(F("WSPR TX Complete - GRIDSQ: "));
  debugSerial.print(grid);
  debugSerial.print(F(" Freq Hz: "));
  debugSerial.println(freq_hz);
  print_monitor_prompt();
  
}

/**********************
/* Serial Monitor code 
/**********************/

void serial_monitor_begin(){
  
  // Start software serial port 
  debugSerial.begin(MONITOR_SERIAL_BAUD);
  debugSerial.println(F("Initialising Orion Serial Monitor...."));
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
        debugSerial.print(F("Orion firmware version: "));
        debugSerial.print(ORION_FW_VERSION);
        debugSerial.println(BOARDNAME);
        break;

       case 'd' : // toggle debug flag
       flush_input();
       debugSerial.print(F("Orion debug tracing is : "));
       g_debug_on_off = toggle_on_off(g_debug_on_off);
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
               
      default:
        flush_input();
        debugSerial.println(F(" -- unrecognized command"));
        println_cmd_list();
       
        // Do nothing
    }
    print_monitor_prompt();
  }
 
}

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
#include <NeoSWSerial.h>
#include "OrionSerialMonitor.h"
#include "OrionXConfig.h""
#include <TimeLib.h>
#define OFF false
#define ON true

static bool g_debug_on_off = OFF;
static bool g_txlog_on_off = OFF;
static bool g_qrm_avoidance_on_off = ON;
 
NeoSWSerial softSerial(12, 11);  // RX, TX  -- Arduino Pro Mini pins 12 and 11 (aka MISO MOSI of six pin ICSP header on DL6OW boards)

bool is_qrm_avoidance_on(){
  if (g_qrm_avoidance_on_off == OFF)
    return false;
  else
    return true;
}

void print_monitor_prompt(){
  softSerial.print(F("> "));
}

void print_date_time() {
  softSerial.print(year());
  softSerial.print(F("-"));
  softSerial.print(month());
  softSerial.print(F("-"));
  softSerial.print(day());
  softSerial.print(F(" "));
  softSerial.print(hour());
  softSerial.print(F(":"));
  softSerial.print(minute());
  softSerial.print(F(":"));
  softSerial.print(second());
  softSerial.print(F(" "));

}

bool toggle_on_off(bool flag){
  bool return_flag = ON;
  
  if (flag == ON){
    return_flag = OFF;
    softSerial.println(F(" OFF"));
  }
  else{
    softSerial.println(F(" ON"));
  }
  return return_flag;
}

// Log a software error. 
// swerr_num is unique number 1-255 assigned sequentially (should be unique for each call to swerr). 
void swerr(byte swerr_num, int data){
  print_date_time();
  softSerial.print(F("***SWERR: "));
  softSerial.print(swerr_num);
  softSerial.print(F(" data dump in hex: "));
  softSerial.println(data, HEX); 
  print_monitor_prompt(); 
    
}
void println_cmd_list(){
  softSerial.println(F("cmds: v = f/w version, d = toggle debug trace on/off, l = toggle TX log on/off, q = toggle qrm avoidance on/off, ? = cmd list"));
}



void orion_sm_trace_pre(byte state, byte event){
  
  if (g_debug_on_off == OFF) return;
  
  print_date_time();
  softSerial.print(F(">> orion PRE sm trace: "));
  softSerial.print(F("curr_state: "));
  softSerial.print(state);
  softSerial.print(F(" curr_event: "));
  softSerial.println(event);
  print_monitor_prompt();  
}

void orion_sm_trace_post(byte state, byte processed_event,  byte resulting_action){
  
  if (g_debug_on_off == OFF) return;
  
  print_date_time();
  softSerial.print(F("<< orion POST sm trace: "));
  softSerial.print(F("curr_state: "));
  softSerial.print(state);
  softSerial.print(F(" event_just_processed: "));
  softSerial.print(processed_event);
  softSerial.print(F(" action: "));
  softSerial.println(resulting_action);
  print_monitor_prompt();
    
}

void orion_log_wspr_tx(OrionWsprMsgType msgType, char grid[], unsigned long freq_hz){
  if (g_txlog_on_off == OFF) return; 
  
  print_date_time();
  softSerial.print(F("WSPR TX Complete - GRIDSQ: "));
  softSerial.print(grid);
  softSerial.print(F(" Freq Hz: "));
  softSerial.println(freq_hz);
  print_monitor_prompt();
  
}

/**********************
/* Serial Monitor code 
/**********************/

void serial_monitor_begin(){
  
  // Start software serial port 
  softSerial.begin(MONITOR_SERIAL_BAUD);
  softSerial.println(F("Initialising Orion Serial Monitor...."));
  println_cmd_list();
  print_monitor_prompt();
  delay(500);
}

static void flush_input(void){

  while (softSerial.available() > 0)
    softSerial.read();
}

void serial_monitor_interface(){
 
  if (softSerial.available() > 0){
    char c = softSerial.read();
    
    softSerial.println(c); // echo the typed character
    
    switch (c) {
      case 'v' :
        flush_input();
        softSerial.print(F("Orion firmware version: "));
        softSerial.println(ORION_FW_VERSION);
        break;

       case 'd' : // toggle debug flag
       flush_input();
       softSerial.print(F("Orion debug tracing is : "));
       g_debug_on_off = toggle_on_off(g_debug_on_off);
       break;

       case 'l' : // toggle TX log flag
       flush_input();
       softSerial.print(F("Orion TX log is : "));
       g_txlog_on_off = toggle_on_off(g_txlog_on_off);
       break;

       case 'h': case '?' : // list commands
       flush_input();
       println_cmd_list();
       break;

       case'q' :
       flush_input();
       softSerial.print(F("Orion QRM avoidance is : "));
       g_qrm_avoidance_on_off = toggle_on_off(g_qrm_avoidance_on_off);
       break;
               
      default:
        flush_input();
        softSerial.println(F(" -- unrecognized command"));
        println_cmd_list();
       
        // Do nothing
    }
    print_monitor_prompt();
  }
 
}

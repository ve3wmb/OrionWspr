#ifndef ORIONXCONFIG_H
#define ORIONXCONFIG_H
/*
   OrionXConfig.h - Orion WSPR Beacon for pico-Balloon payloads for Arduino

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
#include <Arduino.h>

// THIS FILE CONTAINS THE USER MODIFIABLE #DEFINES TO CONFIGURE THE ORION WSPR BEACON


/***********************************************************
   USER SPECIFIED PARAMETERS FOR WSPR
 ***********************************************************/
// Beacon frequency  - hardcoded on a single frequency for now
//#define BEACON_FREQ_HZ          10140110UL     // Base Beacon Frequency In Hz.
#define BEACON_FREQ_HZ          14097010UL

// Configuration parameters for Primary WSPR Message (i.e. Callsign, 4 character grid square and power out in dBm)
#define BEACON_CALLSIGN_6CHAR   "VE3WMB"      // Your beacon Callsign, maximum of 6 characters
#define BEACON_GRID_SQ_4CHAR    "AA01"        // Your hardcoded 4 character Grid Square - this is temporary until we add Grid square derived from GPS Coordinates
#define BEACON_TX_PWR_DBM          7          // Beacon Power Output in dBm (5mW = 7dBm)


/***********************************************************
   Hardware Pin Assignments
 ***********************************************************/
// Arduino Hardware Pin Configurations - change these to match your specific hardware
//#define TX_LED_PIN              4             // TX LED on D4. Make this 0 if you don't have a pin allocated for TX_LED
//#define SYNC_LED_PIN            7             // LED on PIN D7 indicates GPS time synchronization. Make this 0 if you don't have a pin allocated for TX_LED
// Note the most Arduino Boards have a built-in LED that can be used for either of the above purposes referred to as LED_BUILTIN
#define TX_LED_PIN 0
#define SYNC_LED_PIN 0

/***********************************************************
   Si5351a Configuration Parameters
 ***********************************************************/
#define SI5351A_PARK_CLK_NUM    1              // The Si5351a Clock Number output used to mimic the QRP Labs U3S Park feature. This needs to be an unused clk port.
                                               // I recommend terminating this port to ground via a 47 to 56 ohm resistor.
#define SI5351A_CAL_CLK_NUM     2              // Calibration Clock Number                                
#define SI5351A_WSPRTX_CLK_NUM  0              // The Si5351a Clock Number output used for the WSPR Beacon Transmission

/*********************************************************************************************************************** 
*  You need to calibrate your Si5351a and substitute the your correction value for SI5351A_CLK_FREQ_CORRECTION below.
*  See OrionSi5351_calibration.ino sketch. You may also need to modify SI5351BX_XTALPF
*  in OrionSi5351.h is you need a crystal load capacitance other that 8 pf.
************************************************************************************************************************/
//#define SI5351A_CLK_FREQ_CORRECTION  4300     // Correction Value for VE3WMB Adafruit Si5351 Breakout board (Mickey Prototype)
//#define SI5351A_CLK_FREQ_CORRECTION  -13292    // Correction Value for VE3WMB Adafruit Si5351 Breakout board (Donald Prototype)
#define SI5351A_CLK_FREQ_CORRECTION  -6813      // Correction value for Stella9.1 board
                                               

/**************************************************************************************
   GPS CONFIGURATION Parameters - IT ASSUMES THE USE OF A HARWARE SERIAL PORT
 **************************************************************************************/
// GPS Serial port Baud rate - For now only a hardware serial connection to the GPS is supported.
#define GPS_SERIAL_BAUD         9600          // Baudrate for the GPS Serial port

#define MONITOR_SERIAL_BAUD     9600          // Baudrate for Orion Serial Monitor 

// This defines how often we reset the Arduino Clock to the current GPS time 
#define TIME_SET_INTERVAL_MS   600000         // 60,000 ms / minute x 10 = 10 minutes         


/***********************************************************
   Parameters dependant on Processor CPU Speed
 ***********************************************************/
// The following value WSPR_CTC is used for Timer1 that generates a 1.46 Hz interrupt.

// THE CURRENT VALUE for WSPR_CTC ASSUMES AN 8 MHZ PROCESSOR CLOCK !

// If you are using most Arduinos (i.e Nano, UNO etc) they use a 16 Mhz clock and the value 10672 must be substituted.
// ie. #define WSPR_CTC                10672       // CTC value for WSPR on Arduino using 16 Mhz clock (i.e. Nano, Uno etc)
// The formula to calculate WSPR_CTC is: 1.4648 = CPU_CLOCK_SPEED_HZ / (PRESCALE_VALUE) x (WSPR_CTC + 1)
#define WSPR_CTC                5336               // CTC value for WSPR on Arduino using an 8 Mhz clock (i.e. Arduino Pro Mini 3.3v 8 Mhz)


// Type Definitions
enum OrionWsprMsgType {PRIMARY_WSPR_MSG, SECONDARY_WSPR_MSG};
struct OrionTelemetryData {
  long altitude_m;
  long speed_mps;
  byte temperature_c;
  byte battery_voltage_vX100;
  byte number_of_sats;
  bool gps_3d_fix_ok;
};

#endif

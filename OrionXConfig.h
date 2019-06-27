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

#define ORION_FW_VERSION "v0.14a"  // Whole numbers are for released versions. (i.e. 1.0, 2.0 etc.)
                                  // Numbers to the right of the decimal are allocated consecutively, one per GITHUB submission.(i.e. 0.01, 0.02 etc)
                                  // a = alpha b=beta, r=release
                                  
/***********************************************************
   USER SPECIFIED PARAMETERS FOR WSPR
 ***********************************************************/
//#define BEACON_FREQ_HZ          10140110UL     // Base Beacon Frequency In Hz for 30m. 

#define BEACON_FREQ_HZ            14097010UL    //  Base Frequency In Hz for use when QRM Avoidance is enabled. Actual Tx frequency is BEACON_FREQ_HZ + a random offset.
#define FIXED_BEACON_FREQ_HZ      14097070UL    //  Beacon Frequency In Hz for use when QRM Avoidance is disabled.
#define PARK_FREQ_HZ              108000000ULL  // Use this on clk SI5351A_PARK_CLK_NUM to keep the SI5351a warm to avoid thermal drift during WSPR transmissions. Max 109 Mhz.

// Configuration parameters for Primary WSPR Message (i.e. Callsign, 4 character grid square and power out in dBm)
#define BEACON_CALLSIGN_6CHAR   "VE3WMB"      // Your beacon Callsign, maximum of 6 characters
#define BEACON_GRID_SQ_4CHAR    "AA01"        // Your hardcoded 4 character Grid Square - this will be overwritten with GPS derived Grid
#define BEACON_TX_PWR_DBM          7          // Beacon Power Output in dBm (5mW = 7dBm)

// This defines how often we reset the Arduino Clock to the current GPS time 
#define TIME_SET_INTERVAL_MS   600000         // 60,000 ms / minute x 10 = 10 minutes         

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

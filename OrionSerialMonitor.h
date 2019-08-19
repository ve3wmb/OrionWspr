#ifndef ORIONSERIALMONITOR_H
#define ORIONSERIALMONITOR_H
/*
    OrionSerialMonitor.h - Definitions for Orion Debug Monitor

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
#include "OrionXConfig.h"

// For use in info logging
enum OrionWsprMsgType {PRIMARY_WSPR_MSG, ALTITUDE_TELEM_MSG, TEMPERATURE_TELEM_MSG, VOLTAGE_TELEM_MSG};

void swerr(byte swerr_num, int data);
void serial_monitor_begin();
void serial_monitor_interface();
void orion_log_telemetry(struct OrionTxData *data);
void orion_log_wspr_tx(OrionWsprMsgType msgType, char grid[], unsigned long freq_hz, uint8_t pwr_dbm);
void orion_sm_trace_pre(byte state, byte event);
void orion_sm_trace_post(byte state, byte processed_event,  byte resulting_action);
bool is_qrm_avoidance_on();
bool is_selfcalibration_on();  
void log_debug_Timer1_info(byte i, int ofCount, int t_count);
void log_calibration(uint64_t sampled_freq, int32_t o_cal_factor, int32_t n_cal_factor );
void log_calibration_start(); 
void log_time_set(); 
#endif

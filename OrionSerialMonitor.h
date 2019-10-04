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
#include "OrionXConfig.h"

void swerr(byte swerr_num, int data);
void info(byte info_num, int data);
void serial_monitor_begin();
void serial_monitor_interface();
void orion_log_telemetry(struct OrionTxData *data);
void orion_log_wspr_tx(OrionWsprMsgType msgType, char grid[], unsigned long freq_hz, uint8_t pwr_dbm);
void orion_sm_trace_pre(byte state, byte event);
void orion_sm_trace_post(byte state, byte processed_event,  byte resulting_action);
bool is_qrm_avoidance_on();
void enable_qrm_avoidance();
void disable_qrm_avoidance();
bool is_selfcalibration_on();
void log_debug_Timer1_info(byte i, int ofCount, int t_count);
void log_calibration(uint64_t sampled_freq, int32_t o_cal_factor, int32_t n_cal_factor );
void log_calibration_start();
void log_time_set();
void log_shutdown(uint8_t voltagex10);
void log_qrss_tx_start(QrssMode mode, QrssSpeed speed);
void log_qrss_tx_end();
void log_calibration_fail(OrionCalibrationResult fail_reason);

#endif

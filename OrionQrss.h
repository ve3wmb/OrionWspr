#ifndef ORIONQRSS_H
#define ORIONQRSS_H
/*
    OrionQrss.h - Definitions for Orion QRSS Beacon Code  
   (software I2C assumed for compatibiltiy with the QRP Labs U3S and clones).

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
// Configuration Parameters for QRSS Beacon
#define QRSS_BEACON_FREQ_HZ       14096095UL
#define QRSS_BEACON_FSK_OFFSET_HZ 5            // DITS will be transmitted at QRSS_BEACON_FREQ_HZ and DAHS QRSS_BEACON_FSK_OFFSET_HZ higher for FSKCW
#define QRSS_MESSAGE "VE3WMB "                // Message - put your callsign here, in capital letters. Remember to put a space at the end
                                              // Space as used as the message terminator.  

// Delay after QRSS Transmission before attempting self_calibration (3 minutes)
#define POST_QRSS_TX_DELAY_MS   180000  // - Since we wait about 2 minutes for calibration this gives about 5 minutes between QRSS transmissions
#define FSK_HIGH  QRSS_BEACON_FSK_OFFSET_HZ 
#define FSK_LOW 0

// Mode NONE means ouput disabled
// QRSS mode (i.e. ordinary CW)
// FSKCW mode
// DFCW mode
enum QrssMode {MODE_NONE, MODE_QRSS, MODE_FSKCW, MODE_DFCW};
enum QrssSpeed {s12wpm, QRSS3, QRSS6, QRSS10};

void qrss_beacon();

 #endif

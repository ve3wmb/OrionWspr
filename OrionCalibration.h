#ifndef ORIONCALIBRATION_H
#define ORIONCALIBRATION_H
/*
    OrionCalibbration.h - Definitions for Orion SI5351 Self Calibration

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

#define FINE_CORRECTION_STEP   10     // 0.1 HZ step
#define COARSE_CORRECTION_STEP 100   // 1 Hz step

void setup_calibration();
void reset_for_calibration();
void do_calibration(unsigned long calibration_step);
#endif

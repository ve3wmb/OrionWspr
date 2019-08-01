#ifndef ORIONTELEMETRY_H
#define ORIONTELEMETRY_H
/*
   OrionTelemetry.h - Orion WSPR Beacon for pico-Balloon payloads for Arduino

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
int read_voltage_v_x10 ();
int read_DS1820_temperature();
int read_processor_temperature();
uint8_t encode_temperature (int temperature_c);
uint8_t encode_voltage (int voltage_v_x10);
uint8_t encode_altitude (int altitude_m);

#endif

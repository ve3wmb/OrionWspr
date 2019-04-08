#ifndef ORIONSI5351_H
#define ORIONSI5351_H
/*
    OrionSi5351.h - Definitions for control of Si5351a chip via I2C 
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
#include <int.h>

// Definitions for Software I2C. Remove the following if using the Wire.h library for hardware I2C
#define SCL_PIN 1  //PB1
#define SCL_PORT PORTB
#define SDA_PIN 2 //PD2
#define SDA_PORT PORTD

// Macros used by the KE7ER Si5351 Code 
#define BB0(x) ((uint8_t)x)             // Bust int32 into Bytes
#define BB1(x) ((uint8_t)(x>>8))
#define BB2(x) ((uint8_t)(x>>16))

#define SI5351BX_ADDR 0x60              // I2C address of Si5351   (typical)
#define SI5351BX_XTALPF 2               // 1:6pf  2:8pf  3:10pf - assuming 8 pF, otherwise change

// If using 27mhz crystal, set XTAL=27000000, MSA=33.  Then vco=891mhz
#define SI5351BX_XTAL 2500000000ULL      // Crystal freq in centi Hz 
#define SI5351BX_MSA  35                // VCOA is at 25mhz*35 = 875mhz
#define RFRAC_DENOM 1000000ULL
#define SI5351_CLK_ON true
#define SI5351_CLK_OFF false

// Turn the specified clock number on or off. 
void si5351bx_enable_clk(uint8_t clk_num, bool on_off);

// Initialize the Si5351
void si5351bx_init();

// Set the frequency for the specified clock number
// Note that fout is in hertz x 100 (i.e. hundredths of hertz). 
// Frequency range must be between 500 Khz and 109 Mhz
void si5351bx_setfreq(uint8_t clknum, uint64_t fout);

// Write a single 8 bit value to an Si5351a register addres
void i2cWrite(uint8_t reg, uint8_t val);

// Write an array of 8bit values to an Si5351a register address
void i2cWriten(uint8_t reg, uint8_t *vals, uint8_t vcnt);

 #endif

#ifndef ORIONSI5351_H
#define ORIONSI5351_H
/*
    OrionSi5351.h - Definitions for control of Si5351a chip via I2C

   Both software and hardware I2C communication are supported via conditional
   compile.

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

// Macros used by the KE7ER Si5351 Code
#define BB0(x) ((uint8_t)x)             // Bust int32 into Bytes
#define BB1(x) ((uint8_t)(x>>8))
#define BB2(x) ((uint8_t)(x>>16))

#define SI5351BX_ADDR 0x60              // I2C address of Si5351   (typical)

#define RFRAC_DENOM 1000000ULL
#define SI5351_CLK_ON true
#define SI5351_CLK_OFF false

// Turn the specified clock number on or off.
void si5351bx_enable_clk(uint8_t clk_num, bool on_off);

// Initialize the Si5351
void si5351bx_init();

// Set the correction factor for the Si5351a clock.
// This is used for self-calibration
void si5351bx_set_correction(int32_t corr);

// Set the frequency for the specified clock number
// Note that fout is in hertz x 100 (i.e. hundredths of hertz).
// Frequency range must be between 500 Khz and 109 Mhz
// Boolean tx_on specifies whether clock is enabled after frequency
// change. 
void si5351bx_setfreq(uint8_t clknum, uint64_t fout, bool tx_on);

#endif

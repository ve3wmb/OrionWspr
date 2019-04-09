/*
 * OrionSi5351.cpp - Control functions for the Si5351a chip via I2C 
 * (software I2C assumed for compatibiltiy with the QRP Labs U3S and clones).

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
#include "OrionXConfig.h"
#include "OrionSi5351.h"
#include <SoftWire.h>  // Needed for Software I2C otherwise include <Wire.h>

uint64_t si5351bx_vcoa = (SI5351BX_XTAL*SI5351BX_MSA);  // 25mhzXtal calibrate
int32_t si5351_correction = SI5351A_CLK_FREQ_CORRECTION;  //Frequency correction factor calculated using OrionSi5351_calibration sketch
uint8_t  si5351bx_rdiv = 0;             // 0-7, CLK pin sees fout/(2**rdiv) // Note that 0 means divide by 1
uint8_t  si5351bx_drive[3] = {3, 3, 3}; // 0=2ma 1=4ma 2=6ma 3=8ma for CLK 0,1,2 - Set CLK 0,1,2 to 8ma
uint8_t  si5351bx_clken = 0xFF;         // Private, all CLK output drivers off

// Create an instance of Softwire named Wire. If you are using Hardware I2C then create an instance of Wire instead.
SoftWire Wire = SoftWire();

/** *************  SI5315 routines - (tks Jerry Gaffke, KE7ER)   ***********************
   A minimalist standalone set of Si5351 routines originally written by Jerry Gaffke, KE7ER
   but modified by VE3WMB for use with Software I2C and to provide sub-Hz resolution for WSPR
   transmissions. .
   
   VCOA is fixed at 875mhz, VCOB not used.
   The output msynth dividers are used to generate 3 independent clocks
   with 1hz resolution to any frequency between 4khz and 109mhz.
   
   Usage:
   Call si5351bx_init() once at startup with no args;
   
   Call si5351bx_setfreq(clknum, freq) each time one of the
   three output CLK pins is to be updated to a new frequency.
   
   A freq of 0 serves to shut down that output clock or alternately a
   call to si5351bx_enable_clk(uint8_t clk_num, bool on_off)
   
   The global variable si5351bx_vcoa starts out equal to the nominal VCOA
   frequency of 25mhz*35 = 875000000 Hz.  To correct for 25mhz crystal errors,
   the user can adjust this value.  The vco frequency will not change but
   the number used for the (a+b/c) output msynth calculations is affected.
   Example:  We call for a 5mhz signal, but it measures to be 5.001mhz.
   So the actual vcoa frequency is 875mhz*5.001/5.000 = 875175000 Hz,
   To correct for this error:     si5351bx_vcoa=875175000;
   
   Most users will never need to generate clocks below 500khz.
   But it is possible to do so by loading a value between 0 and 7 into
   the global variable si5351bx_rdiv, be sure to return it to a value of 0
   before setting some other CLK output pin.  The affected clock will be
   divided down by a power of two defined by  2**si5351_rdiv
   
   A value of zero gives a divide factor of 1, a value of 7 divides by 128.
   This lightweight method is a reasonable compromise for a seldom used feature.
*/

// Turn the specified clock number on or off. 
void si5351bx_enable_clk(uint8_t clk_num, bool on_off) {
    if (on_off == SI5351_CLK_OFF ) { // Off Disable ClK 
      si5351bx_clken |= 1 << clk_num;      //  Set Bit to shut down the clock
    }
    else {  // Enable CLK
      si5351bx_clken &= ~(1 << clk_num);   // Clear bit to enable clock
    }
   i2cWrite(3, si5351bx_clken);   
}

// Initialize the Si5351a 
void si5351bx_init() {                  // Call once at power-up, start PLLA
  uint8_t reg;  uint32_t msxp1;
  Wire.begin();
  i2cWrite(149, 0);                     // SpreadSpectrum off
  i2cWrite(3, si5351bx_clken);          // Disable all CLK output drivers
  i2cWrite(183, ((SI5351BX_XTALPF << 6) | 0x12)); // Set 25mhz crystal load capacitance (tks Daniel KB3MUN)
  msxp1 = 128 * SI5351BX_MSA - 512;     // and msxp2=0, msxp3=1, not fractional
  uint8_t  vals[8] = {0, 1, BB2(msxp1), BB1(msxp1), BB0(msxp1), 0, 0, 0};
  i2cWriten(26, vals, 8);               // Write to 8 PLLA msynth regs
  i2cWrite(177, 0x20);                  // Reset PLLA  (0x80 resets PLLB)
}

// Set the frequency for the specified clock number
// Note that fout is in hertz x 100 (i.e. hundredths of hertz). 
// Frequency range must be between 500 Khz and 109 Mhz
// An fout value of 0 will shutdown the specified clock.

void si5351bx_setfreq(uint8_t clknum, uint64_t fout)
{
  // Note that I am not being lazy here in naming variables. If you refer to SiLabs 
  // application note AN619 - "Manually Generating an Si5351 Register Map", the formulas
  // within refer to calculating values named a,b,c and p1, p2, p3. 
  // For consistency I continue to use the same notation, even though the calculations appear
  // a bit cryptic. 
  uint64_t a,b,c, ref_freq;
  uint32_t p1, p2, p3;
  uint8_t vals[8];

  if ((fout < 50000000) || (fout > 10900000000)) {  // If clock freq out of range 500 Khz to 109 Mhz
    si5351bx_clken |= 1 << clknum;      //  shut down the clock
  }
  
  else {
    
    // Determine the integer part of feedback equation
    ref_freq = si5351bx_vcoa;
    ref_freq = ref_freq + (int32_t)((((((int64_t)si5351_correction) << 31) / 1000000000LL) * ref_freq) >> 31);
    a = ref_freq / fout;
    b = (ref_freq % fout * RFRAC_DENOM) / fout;
    c = b ? RFRAC_DENOM : 1;
  
    p1 = 128 * a + ((128 * b) / c) - 512;
    p2 = 128 * b - c * ((128 * b) / c);
    p3 = c;

    // Setup the bytes to be sent to the Si5351a register
    vals[0] = (p3 & 0x0000FF00) >> 8;
    vals[1] = p3 & 0x000000FF;
    vals[2] = (p1 & 0x00030000) >> 16;
    vals[3] = (p1 & 0x0000FF00) >> 8;
    vals[4]= p1 & 0x000000FF;
    vals[5] = (((p3 & 0x000F0000) >> 12) | ((p2 & 0x000F0000) >> 16));
    vals[6] = (p2 & 0x0000FF00) >> 8;
    vals[7] = p2 & 0x000000FF;
    i2cWriten(42 + (clknum * 8), vals, 8); // Write to 8 msynth regs
    i2cWrite(16 + clknum, 0x0C | si5351bx_drive[clknum]); // use local msynth
    si5351bx_clken &= ~(1 << clknum);   // Clear bit to enable clock
    
  }
  
  i2cWrite(3, si5351bx_clken);        // Enable/disable clock

}

// Write a single 8 bit value to an Si5351a register address
void i2cWrite(uint8_t reg, uint8_t val) {   // write reg via i2c
  Wire.beginTransmission(SI5351BX_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Write an array of 8bit values to an Si5351a register address
void i2cWriten(uint8_t reg, uint8_t *vals, uint8_t vcnt) {  // write array
  Wire.beginTransmission(SI5351BX_ADDR);
  Wire.write(reg);
  while (vcnt--) Wire.write(*vals++);
  Wire.endTransmission();
}

// *********** End of Jerry's si5315bx routines *********************************************************

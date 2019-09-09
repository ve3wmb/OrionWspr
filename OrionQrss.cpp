/*
   OrionQrss.cpp - Orion QRSS Beacon (slow speed CW)
   This provides a fallback QRSS TX mode that will become active
   when a GPS_LOS_TIME_EV occurs, indicating a long term GPS LOS (loss of signal).

   This code is derived from the QRSS/FSKCW/DFCW Beacon Keyer by Hans Summers G0UPL, 2012 (copyright)
   and used with permission by Hans for this derivitive work.

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
#include "OrionBoardConfig.h"
#include "OrionSi5351.h"
#include "OrionQrss.h"
#include <Chrono.h>
#include "OrionSerialMonitor.h"

const char msg[] = QRSS_MESSAGE; // Defined in OrionQrss.h

// This array is indexed by a parameter of type QrssSpeed, defined in OrionQrss.h
const unsigned int speeds[] = {1, 30, 60, 100};   // Speeds for: s12wpm, QRSS3, QRSS6, QRSS10

//
// This function returns the encoded CW pattern for the character passed in.
// Binary encoding is left-padded with ones.
// Processing left to right, first 0 (discarded) indicates that pattern starts with next bit.
// Pattern encoding is 0 = DIT, 1 = DAH.
// So 'A' = B11111001, which is 1 1 1 1 1 (padding bits) 0 (start bit)  0 1 (dit, dah)

byte charCode(char c)
{
  switch (c)
  {
    case 'A':  return B11111001; break;    // A  .-
    case 'B':  return B11101000; break;    // B  -...
    case 'C':  return B11101010; break;    // C  -.-.
    case 'D':  return B11110100; break;    // D  -..
    case 'E':  return B11111100; break;    // E  .
    case 'F':  return B11100010; break;    // F  ..-.
    case 'G':  return B11110110; break;    // G  --.
    case 'H':  return B11100000; break;    // H  ....
    case 'I':  return B11111000; break;    // I  ..
    case 'J':  return B11100111; break;    // J  .---
    case 'K':  return B11110101; break;    // K  -.-
    case 'L':  return B11100100; break;    // L  .-..
    case 'M':  return B11111011; break;    // M  --
    case 'N':  return B11111010; break;    // N  -.
    case 'O':  return B11110111; break;    // O  ---
    case 'P':  return B11100110; break;    // P  .--.
    case 'Q':  return B11101101; break;    // Q  --.-
    case 'R':  return B11110010; break;    // R  .-.
    case 'S':  return B11110000; break;    // S  ...
    case 'T':  return B11111101; break;    // T  -
    case 'U':  return B11110001; break;    // U  ..-
    case 'V':  return B11100001; break;    // V  ...-
    case 'W':  return B11110011; break;    // W  .--
    case 'X':  return B11101001; break;    // X  -..-
    case 'Y':  return B11101011; break;    // Y  -.--
    case 'Z':  return B11101100; break;    // Z  --..
    case '0':  return B11011111; break;    // 0  -----
    case '1':  return B11001111; break;    // 1  .----
    case '2':  return B11000111; break;    // 2  ..---
    case '3':  return B11000011; break;    // 3  ...--
    case '4':  return B11000001; break;    // 4  ....-
    case '5':  return B11000000; break;    // 5  .....
    case '6':  return B11010000; break;    // 6  -....
    case '7':  return B11011000; break;    // 7  --...
    case '8':  return B11011100; break;    // 8  ---..
    case '9':  return B11011110; break;    // 9  ----.
    case ' ':  return B11101111; break;    // Space - equal to 4 dah lengths
    case '/':  return B11010010; break;    // /  -..-.
    default: return charCode(' ');
  }
}



void setRfFsk(boolean rf_on, boolean setFSK_high)
{
  uint8_t fsk_value;

  if (setFSK_high == true) {
    fsk_value = FSK_HIGH;
  }
  else {
    fsk_value = FSK_LOW;
  }


  if (rf_on == true) {
    si5351bx_setfreq(SI5351A_WSPRTX_CLK_NUM, ((QRSS_BEACON_FREQ_HZ + fsk_value) * 100ULL), SI5351_CLK_ON );
  }
  else {
    si5351bx_enable_clk(SI5351A_WSPRTX_CLK_NUM, SI5351_CLK_OFF); // Disable the TX clock
  }
}


//
bool qrss_transmit(QrssMode mode, QrssSpeed ditSpeed)
{
  static byte timerCounter;              // Counter to get to divide by 100 to get 10Hz
  static int ditCounter;                 // Counter to time the length of each dit
  static byte pause;                     // Generates the pause between characters
  static byte msgIndex = 255;            // Index into the message
  static byte character;                 // Bit pattern for the character being sent
  static byte key;                       // State of the key
  static byte charBit;                   // Which bit of the bit pattern is being sent

  static boolean dah;                    // True when a dah is being sent
  byte divisor;                          // Divide 1kHz by 100 normally, but by 33 when sending DFCW)
  bool transmission_done = false;

  // Set Divisor based on Mode
  if (mode == MODE_DFCW)                 // Divisor is 33 for DFCW, to get the correct timing
    divisor = 33;                        // (inter-symbol gap is 1/3 of a dit)
  else
    divisor = 100;                      // For ever other mode it is one dit length
    

  timerCounter++;                        // 1000Hz at this point

  if (timerCounter == divisor)           // Divides by 100 (or 33 for DFCW)
  {
    timerCounter = 0;                    // 10 Hz here (30Hz for DFCW)
    ditCounter++;                        // Generates the correct dit-length
    
    if (ditCounter >= speeds[ditSpeed]){ // We have counted the duration of a dit
    
      ditCounter = 0;

      if (!pause) {
       // Pause is set to 2 after the last element of the character has been sent
        key--;                         // This generates the correct pause between characters (3 dits)
        if ((!key) && (!charBit)){
        
          if (mode == MODE_DFCW)
            pause = 3;                 // DFCW needs an extra delay to make it 4/3 dit-length
          else
            pause = 2;
        }
      } // end if (!pause) 
      else
        pause--;
        
      
      // Key becomes 255 when the last element (dit or dah) of the character has been sent
      if (key == 255) {
        // Done sending the last element (dit or dah) in the character
        // If the last symbol of the character has been sent, get the next character
        
        if (!charBit){
          // Increment the message character index
          msgIndex++;
          
          // Reset to the start of the message when the end is reached
          if (!msg[msgIndex]) msgIndex = 0;
          
          // Get the encoded bit pattern for the morse character
          character = charCode(msg[msgIndex]);
          // Start at the 7'th (leftmost) bit of the bit pattern
          charBit = 7;
          
          // Loop through bits looking for a 0, signifying start of coding bits
          while (character & (1 << charBit)) charBit--;

        } // end if (!charBit)

        charBit--;                     // Move to the next rightermost bit of the pattern, this is the first element

        // Special case for the space character, we use this to designate the end of message.
        if (character == charCode(' ')) {
          key = 0;
          dah = false;
          transmission_done = true; 
        }
        else {
          // Get the state of the current bit in the pattern
          key = character & (1 << charBit);

          if (key) {                     // If it's a 1, set this to a dah
            key = 3;
            dah = true;
          }
          else {                        // otherwise it's a dit
            if (mode == MODE_DFCW)     // Special case for DFCW - dit's and dah's are both
              key = 3;                 // the same length.
            else
              key = 1;

            dah = false;
          }
        }
      } // end if (key == 255 )

      if (!key) dah = false;

      
      // 
      if (mode == MODE_FSKCW)
      {
        //setRF(true);                    // in FSK/CW mode, the RF output is always ON
        //setFSK(key);                    // and the FSK depends on the key state
        setRfFsk(true, key);
      }
      else if (mode == MODE_QRSS)
      {
        //setRF(key);                     // in QRSS mode, the RF output is keyed
        //setFSK(false);                  // and the FSK is always off
        setRfFsk(key, false);
      }
      else if (mode == MODE_DFCW)
      {
        //setRF(key);                     // in DFCW mode, the RF output is keyed (ON during a dit or a dah)
        //setFSK(dah);                    // and the FSK depends on the key state
        setRfFsk(key, dah);
      }
      else
        setRfFsk(false, false);

    } // end if (ditcounter >= speeds[ditspeed];
    
  } // end if (timercounter == divisor)


  if (transmission_done == true){
    // reset the static variables for the next transmission 
    timerCounter = 0;             
    ditCounter = 0;                 
    pause = 0;                     
    msgIndex = 255;                 
    character = 0;                 
    key = 0;                       
    charBit = 0;                   
    dah = false; 
  }
  
  return transmission_done; 
  
} // end qrss_transmit function


void qrss_beacon(){

  static unsigned long milliPrev;        // Static variable stores previous millisecond count
  unsigned long milliNow;
  bool done_transmission = false;

  
  // Turn off the PARK clock
  si5351bx_enable_clk(SI5351A_PARK_CLK_NUM, SI5351_CLK_OFF);

  log_qrss_tx_start(MODE_DFCW, QRSS10);
  
  while (!done_transmission) {
    milliNow = millis();                   // Get millisecond counter value

    if (milliNow != milliPrev)             // If one millisecond has elapsed, call the beacon() function
    {
      milliPrev = milliNow;
      done_transmission = qrss_transmit(MODE_QRSS, QRSS10); // This gets called once per millisecond (i.e 1000 times per second)
    }
  } // end while (!done)

  // Ensure that the Si5351a TX clock is shutdown
  si5351bx_enable_clk(SI5351A_WSPRTX_CLK_NUM, SI5351_CLK_OFF);

  // Re-enable the Park Clock
  si5351bx_setfreq(SI5351A_PARK_CLK_NUM, (PARK_FREQ_HZ * 100ULL), SI5351_CLK_ON);

  log_qrss_tx_end();

  delay(POST_QRSS_TX_DELAY_MS); // Kill some time before attempting calibration 
  
} // end qrss_beacon()

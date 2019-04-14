// OrionWspr.ino - Orion WSPR Beacon for pico-Balloon payloads using Arduino
//
// This code implements a very low power Arduino HF WSPR Beacon using a Silicon Labs Si5351a clock chip 
// as the beacon transmitter and a GPS receiver module for time and location fix.
//
// This work is dedicated to the memory of my very dear friend Ken Louks, WA8REI(SK)
// - Michael Babineau, VE3WMB - March 4, 2019.
//
// Why is it called Orion?
// Serendipity! I was trying to think of a reasonable short name for this code one evening
// while sitting on the couch watching Netflix. I happened to look out at the night sky thorough
// the window and there was the constellation Orion staring back at me, so Orion it is.
//
// I wish that I could say that I came up with this code all on my own, but for the most part
// it is based on excellent work done by people far more clever than I am.
//
// It was derived from the Simple WSPR beacon for Arduino Uno, with the Etherkit Si5351A Breakout
// Board, by Jason Milldrum NT7S whose original code was itself based on Feld Hell beacon for Arduino by Mark
// Vandewettering K6HX and adapted for the Si5351A by Robert Liesenfeld AK6L <ak6l@ak6l.org>.
// Timer setup code by Thomas Knutsen LA3PNA. Time code adapted from the TimeSerial.ino example from the Time library.
//
// The original Si5351 Library code (from Etherkit .. aka NT7S) was replaced by self-contained SI5315 routines 
// written Jerry Gaffke, KE7ER. The KE7ER code was modified by VE3WMB to use 64 bit precision in the calculations to
// enable the sub-Hz resolution needed for WSPR and to allow Software I2C usage via the inclusion of <SoftWire.h>.
//
// Code modified and added by Michael Babineau, VE3WMB for Project Aries pico-Balloon WSPR Beacon (2018/2019)
// This additional code is Copyright (C) 2018-2019 Michael Babineau <mbabineau.ve3wmb@gmail.com>


// Hardware Requirements
// ---------------------
// This firmware must be run on an Arduino or an AVR microcontroller with the Arduino Bootloader installed.
// Pay special attention, the Timer1 Interrupt code is highly dependant on processor clock speed.
//
// I have tried to keep the AVR/Arduino port usage in the implementation configurable via #defines in OrionXConfig.h so this
// code can more easily be setup to run on different beacon boards such as those designed by DL6OW and N2NXZ and in future
// the QRP Labs U3B. This was also the motivation for moving to a software I2C solution using SoftWire.h. The SoftWire interface
// mimics the Wire library so switching back to using hardware-enable I2C is simple. 
//
// Required Libraries
// ------------------
// Etherkit JTEncode (Library Manager)  https://github.com/etherkit/JTEncode
// Time (Library Manager)   https://github.com/PaulStoffregen/Time
// SoftWire (https://github.com/stevemarple/SoftWire) - (assumes that you are using Software I2C otherwise substitute Wire.h)
// TinyGPS (Library Manager) http://arduiniana.org/libraries/TinyGPS/
//
// License
// -------
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
// ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
// CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//

#include <JTEncode.h>
#include <int.h>
#include <TimeLib.h>
#include <TinyGPS.h>
//#include <Wire.h>     / Only uncomment this if you are using Hardware I2C to talk to the Si5351a
#include "OrionXConfig.h"
#include "OrionSi5351.h"

// NOTE THAT ALL #DEFINES THAT ARE INTENDED TO BE USER CONFIGURABLE ARE LOCATED IN OrionXConfig.h.
// DON'T TOUCH ANYTHING DEFINED IN THIS FILE WITHOUT SOME VERY CAREFUL CONSIDERATION.

// WSPR specific defines. DO NOT CHANGE THESE VALUES, EVER!
#define TONE_SPACING            146                 // ~1.46 Hz
#define SYMBOL_COUNT            WSPR_SYMBOL_COUNT

// Globals
JTEncode jtencode;
TinyGPS gps;

unsigned long g_beacon_freq_hz = BEACON_FREQ_HZ;      // The Beacon Frequency in Hz

// Use this on clk SI5351A_PARK_CLK_NUM to keep the SI5351a warm to avoid thermal drift during WSPR transmissions
uint64_t g_park_freq_hz = 108000000ULL;            // 108 MHz,in hertz

char g_beacon_callsign[7] = BEACON_CALLSIGN_6CHAR;

// Grid Square is hardcoded for now until the code is written to derive it from GPS Coordinates
char g_grid_loc[5] = BEACON_GRID_SQ_4CHAR;
uint8_t g_tx_pwr_dbm = BEACON_TX_PWR_DBM;
uint8_t g_tx_buffer[SYMBOL_COUNT];

// Global variables used in ISRs
volatile bool g_proceed = false;

// Timer interrupt vector.  This toggles the variable g_proceed which we use to gate
// each column of output to ensure accurate timing.  This ISR is called whenever
// Timer1 hits the WSPR_CTC value used below in setup().
ISR(TIMER1_COMPA_vect)
{
  g_proceed = true;
}

// Transmit the Primary WSPR Message
// Loop through the transmit buffer, transmitting one character at a time.
void encode_and_tx_wspr_msg1()
{
  uint8_t i;
  
  // Encode the primary message paramters into the TX Buffer
  jtencode.wspr_encode(g_beacon_callsign, g_grid_loc, g_tx_pwr_dbm, g_tx_buffer);

  // Reset the tone to 0 and turn on the TX output
  si5351bx_setfreq(SI5351A_WSPRTX_CLK_NUM,(g_beacon_freq_hz * 100ULL));

  // Turn off the PARK clock
  si5351bx_enable_clk(SI5351A_PARK_CLK_NUM, SI5351_CLK_OFF);

  if (TX_LED_PIN != 0) { // If we are using the TX LED turn it on
    digitalWrite(TX_LED_PIN, HIGH);
  }
  // Now send the rest of the message
  for (i = 0; i < SYMBOL_COUNT; i++)
  {
    si5351bx_setfreq(SI5351A_WSPRTX_CLK_NUM, (g_beacon_freq_hz * 100ULL) + (g_tx_buffer[i] * TONE_SPACING));
    g_proceed = false;

    // We spin our wheels in TX here, waiting until the Timer1 Interrupt sets the g_proceed flag
    // Then we can go back to the top of the for loop to start sending the next symbol
    while (!g_proceed);
  }

  // Turn off the WSPR TX clock output, we are done sending the message
  si5351bx_enable_clk(SI5351A_WSPRTX_CLK_NUM, SI5351_CLK_OFF);

  // Re-enable the Park Clock
  si5351bx_enable_clk(SI5351A_PARK_CLK_NUM, SI5351_CLK_ON); 

  if (TX_LED_PIN != 0) { // If we are using the TX LED turn it off
    digitalWrite(TX_LED_PIN, LOW);
  }
} // end of encode_and_tx_wspr_msg1()

void setup()
{
  // Use the TX_LED_PIN as a transmit indicator, but only if it is non-zero
  if (TX_LED_PIN != 0) {
    pinMode(TX_LED_PIN, OUTPUT);
    digitalWrite(TX_LED_PIN, LOW);
  }


  // Use the SYNC_LED_PIN to indicate the beacon has valid time from the GPS, but only if it is non-zero
  if (SYNC_LED_PIN != 0) {
    pinMode(SYNC_LED_PIN, OUTPUT);
    digitalWrite(SYNC_LED_PIN, LOW);
  }

  // Start Hardware serial communications with the GPS
  Serial.begin(GPS_SERIAL_BAUD);

  // Initialize the Si5351
  si5351bx_init();
  
  // Setup WSPR TX output
  si5351bx_setfreq(SI5351A_WSPRTX_CLK_NUM,(g_beacon_freq_hz * 100ULL));
  si5351bx_enable_clk(SI5351A_WSPRTX_CLK_NUM, SI5351_CLK_OFF); // Disable the clock initially
  
  // Set PARK CLK Output - Note that we leave SI5351A_PARK_CLK_NUM running at 108 Mhz to keep the SI5351 temperature more constant
  // This minimizes thermal induced drift during WSPR transmissions. The idea is borrowed from G0UPL's PARK feature on the QRP Labs U3S
  si5351bx_setfreq(SI5351A_PARK_CLK_NUM,(g_park_freq_hz * 100ULL)); // Turn on Park Clock

  // Set up Timer1 for interrupts every symbol period (i.e 1.46 Hz)
  // The formula to calculate this is CPU_CLOCK_SPEED_HZ / (PRESCALE_VALUE) x (WSPR_CTC + 1)
  // In this case we are using a prescale of 1024.
  noInterrupts();          // Turn off interrupts.
  TCCR1A = 0;              // Set entire TCCR1A register to 0; disconnects
  //   interrupt output pins, sets normal waveform
  //   mode.  We're just using Timer1 as a counter.
  TCNT1  = 0;              // Initialize counter value to 0.
  TCCR1B = (1 << CS12) |   // Set CS12 and CS10 bit to set prescale
           (1 << CS10) |   //   to /1024
           (1 << WGM12);   //   turn on CTC
  //   which gives, 64 us ticks
  TIMSK1 = (1 << OCIE1A);  // Enable timer compare interrupt.

  // Note the the OCR1A value is processor clock speed dependant.
  // WSPR_CTC must be defined as 5336 for an 8Mhz processor clock or 10672 for a 16 Mhz Clock
  OCR1A = WSPR_CTC;       // Set up interrupt trigger count.

  interrupts();            // Re-enable interrupts.
} // end setup()


void loop()
{

  // Get the time from the GPS when it is ready
  while (Serial.available()) {
    if (gps.encode(Serial.read())) {

      // process gps messages when TinyGPS reports new data...
      unsigned long age;
      int Year;
      byte Month, Day, Hour, Minute, Second;

      gps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, NULL, &age);

      if (age < 500) {
        // set the Time to the latest GPS reading
        // Setting the clock this frequently seems like overkill but it works.
        // This strategy will be revisited as the code is expanded and updated with new functionality
        setTime(Hour, Minute, Second, Day, Month, Year);
      }
    }
  }

  if (SYNC_LED_PIN != 0) {               // If we are using the SYNC_LED
    if (timeStatus() == timeSet) {
      digitalWrite(SYNC_LED_PIN, HIGH); // Turn LED on if the time is synced
    }
    else {
      digitalWrite(SYNC_LED_PIN, LOW);  // LED off if time needs refresh
    }
  }

  // We beacon every 10th minute of the hour so we use minute()modulo 10 (i.e. on 00, 10, 20, 30, 40, 50)
  // WSPR should start on the 1st second of the minute, but there's a slight delay
  // in this code because it is limited to 1 second resolution.
  if ((timeStatus() == timeSet) && ((minute() % 10) == 0) && (second() == 0)) {
    encode_and_tx_wspr_msg1();
    delay(1000); // Delay one second
  }

} // end loop ()

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
// I have tried to keep the AVR/Arduino port usage in the implementation configurable via #defines in OrionBoardConfig.h so that this
// code can more easily be setup to run on different beacon boards such as those designed by DL6OW and N2NXZ and in future
// the QRP Labs U3B. This was also the motivation for moving to a software I2C solution using SoftWire.h. The SoftWire interface
// mimics the Wire library so switching back to using hardware-enable I2C is simple.
//
// Required Libraries
// ------------------
// Etherkit JTEncode (Library Manager)  https://github.com/etherkit/JTEncode
// Time (Library Manager)   https://github.com/PaulStoffregen/Time - This provides a Unix-like System Time capability
// SoftI2CMaster (Software I2C with SoftWire wrapper) https://github.com/felias-fogg/SoftI2CMaster/blob/master/SoftI2CMaster.h - (assumes that you are using Software I2C otherwise Wire.h)
// NeoGps (https://github.com/SlashDevin/NeoGPS) - NMEA and uBlox GPS parser using Nominal Configuration : date, time, lat/lon, altitude, speed, heading,
//  number of satellites, HDOP, GPRMC and GPGGA messages.
// NeoSWSerial (https://github.com/SlashDevin/NeoSWSerial)
// LightChrono (https://github.com/SofaPirate/Chrono) - Simple Chronometer Library
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
#include <NMEAGPS.h>  // NeoGps
#include <TimeLib.h>
#include <LightChrono.h>
#include "OrionXConfig.h"
#include "OrionBoardConfig.h"
#include "OrionSi5351.h"
#include "OrionStateMachine.h"
#include "OrionSerialMonitor.h"
#include "OrionCalibration.h"
#include "OrionTelemetry.h"

// NOTE THAT ALL #DEFINES THAT ARE INTENDED TO BE USER CONFIGURABLE ARE LOCATED IN OrionXConfig.h and OrionBoardConfig.h
// DON'T TOUCH ANYTHING DEFINED IN THIS FILE WITHOUT SOME VERY CAREFUL CONSIDERATION.

// Port Definitions for NeoGps
// We support hardware-based Serial, or software serial communications with the GPS using NeoSWSerial using conditional compilation
// based on the #define GPS_USES_HW_SERIAL. For software serial comment out this #define in OrionBoardConfig.h
#if defined (GPS_USES_HW_SERIAL)
#define gpsPort Serial
#define GPS_PORT_NAME "Serial"
#else
#include <NeoSWSerial.h>
#define GPS_PORT_NAME "NeoSWSerial"
#endif

#define GPS_STATUS_STD 4  //This needs to match the definition in NeoGPS for STATUS_STD 


// WSPR specific defines. DO NOT CHANGE THESE VALUES, EVER!
#define TONE_SPACING            146                 // ~1.46 Hz
#define SYMBOL_COUNT            WSPR_SYMBOL_COUNT

// Globals
JTEncode jtencode;

// GPS related
static NMEAGPS gps;
static gps_fix fix;

//Time related
LightChrono g_chrono;

// If we are using software serial to talk to the GPS then we need to create an instance of NeoSWSerial and
// provide the RX and TX Pin numbers.
#if !defined (GPS_USES_HW_SERIAL)
NeoSWSerial gpsPort(SOFT_SERIAL_RX_PIN, SOFT_SERIAL_TX_PIN);  // RX, TX
#endif

// We initialize this to 61 so the first time through the scheduler we can't possibly match the current second
// This forces the scheduler to run on its very first call.
byte g_last_second = 61;

unsigned long g_beacon_freq_hz = FIXED_BEACON_FREQ_HZ;      // The Beacon Frequency in Hz

// Raw Telemetry data types define in OrionTelemetry.h
// This contains the last validated Raw telemetry for use during GPS LOS (i.e. we use the last valid data if current data is missing)
struct OrionTelemetryData g_last_valid_telemetry = {0, 0, 0, 0, 0, 0, 0, 0, 0};
struct OrionTelemetryData g_orion_current_telemetry {0, 0, 0, 0, 0, 0, 0, 0, 0};

// This differs from OrionTelemetryData mostly in that the values are reformatted with the correct units (i.e altitude in metres vs cm etc.)
// It is used to populate the global values below prior to encoding the WSPR message for TX
struct OrionTxData g_tx_data  = {'0', 0, 0, 0, 0, 0, 0, 0};

// The following values are used in the encoding and transmission of the WSPR Type 1 messages.
// They are populated from g_tx_data according to the implemented Telemetry encoding rules.
char g_beacon_callsign[7] = BEACON_CALLSIGN_6CHAR;
char g_grid_loc[5] = BEACON_GRID_SQ_4CHAR; // Grid Square defaults to hardcoded value it is over-written with a value derived from GPS Coordinates
uint8_t g_tx_pwr_dbm = BEACON_TX_PWR_DBM;  // This value is overwritten to encode telemetry data.
uint8_t g_tx_buffer[SYMBOL_COUNT];

// Globals used by the Orion Scheduler
OrionAction g_current_action = NO_ACTION;

// Global variables used in ISRs
volatile bool g_proceed = false;

// Timer interrupt vector.  This toggles the variable g_proceed which we use to gate
// each column of output to ensure accurate timing.  This ISR is called whenever
// Timer1 hits the WSPR_CTC value used below in setup().
ISR(TIMER1_COMPA_vect)
{
  g_proceed = true;
}

// ------- Functions ------------------

unsigned long get_tx_frequency() {
  /********************************************************************************
    Get the frequency for the next transmission cycle
    This function also implements the QRM Avoidance feature
    which applies a pseudo-random offset of 0 to 100 hz to the base TX frequency
  ********************************************************************************/
  if (is_qrm_avoidance_on() == false)
    return FIXED_BEACON_FREQ_HZ;
  else {
    // QRM Avoidance - add a random number in range of 0 to 180 to the base TX frequency to help avoid QRM
    return (BEACON_FREQ_HZ + random(181)); // base freq + 0 to 180 hz random offset
  }

}

// -- Telemetry -------

void calculate_gridsquare_6char(float lat, float lon) {
  /***************************************************************************************
    Calculate the 6 Character Maidenhead Gridsquare from the Lat and Long Coordinates
    We follow the convention that West and South are negative Lat/Long.
  ***************************************************************************************/
  // This puts the calculated 6 character Maidenhead Grid square into the field grid_sq_6char[]
  // of g_tx_data

  // Temporary variables for calculation
  int o1, o2, o3;
  int a1, a2, a3;
  float remainder;

  // longitude
  remainder = lon + 180.0;
  o1 = (int)(remainder / 20.0);
  remainder = remainder - (float)o1 * 20.0;
  o2 = (int)(remainder / 2.0);
  remainder = remainder - 2.0 * (float)o2;
  o3 = (int)(12.0 * remainder);

  // latitude
  remainder = lat + 90.0;
  a1 = (int)(remainder / 10.0);
  remainder = remainder - (float)a1 * 10.0;
  a2 = (int)(remainder);
  remainder = remainder - (float)a2;
  a3 = (int)(24.0 * remainder);

  // Generate the 6 character Grid Square
  g_tx_data.grid_sq_6char[0] = (char)o1 + 'A';
  g_tx_data.grid_sq_6char[1] = (char)a1 + 'A';
  g_tx_data.grid_sq_6char[2] = (char)o2 + '0';
  g_tx_data.grid_sq_6char[3] = (char)a2 + '0';
  g_tx_data.grid_sq_6char[4] = (char)o3 + 'A';
  g_tx_data.grid_sq_6char[5] = (char)a3 + 'A';
  g_tx_data.grid_sq_6char[6] = (char)0;
} // calculate_gridsquare_6char


void get_telemetry_data() {

  // GPS Location
  if (fix.valid.location) {
    // We have a valid location so save the lat/long to current_telemetry
    g_orion_current_telemetry.latitude = fix.latitude();
    g_orion_current_telemetry.longitude = fix.longitude();

    // Copy the lat/long to last_valid_telemetry so we can use it if our fix is invalid (GPS LOS?) next time.
    g_last_valid_telemetry.latitude = g_orion_current_telemetry.latitude;
    g_last_valid_telemetry.longitude = g_orion_current_telemetry.longitude;
  }
  else {
    // Our position fix isn't valid so use the position information from the last_valid_telemetry, not ideal but better than nothing
    g_orion_current_telemetry.latitude = g_last_valid_telemetry.latitude;
    g_orion_current_telemetry.longitude =  g_last_valid_telemetry.longitude;
  }

  // GPS Altitude
  if (fix.valid.altitude) {
    g_orion_current_telemetry.altitude_cm = fix.altitude_cm();
    g_last_valid_telemetry.altitude_cm = g_orion_current_telemetry.altitude_cm;
  }
  else {
    // Our altitude fix isn't valid so use the last valid data
    g_orion_current_telemetry.altitude_cm  = g_last_valid_telemetry.altitude_cm;
  }

  // GpS Speed
  if (fix.valid.speed) {
    g_orion_current_telemetry.speed_mkn = fix.speed_mkn();
    g_last_valid_telemetry.speed_mkn = g_orion_current_telemetry.speed_mkn;
  }
  else {
    // Our speed fix isn't valid so use the last valid data
    g_orion_current_telemetry.speed_mkn = g_last_valid_telemetry.speed_mkn;
  }

  // GPS number of sats
  if (fix.valid.satellites) {
    g_orion_current_telemetry.number_of_sats = fix.satellites;
    g_last_valid_telemetry.number_of_sats = g_orion_current_telemetry.number_of_sats;
  }
  else {
    g_orion_current_telemetry.number_of_sats = g_last_valid_telemetry.number_of_sats;
  }

  // Overall GPS status.
  if (fix.valid.status) {
    g_orion_current_telemetry.gps_status = fix.status;
  }
  else {
    // Assume that if we don't have a valid Status fix that the GPS status is not OK
    g_orion_current_telemetry.gps_status = 0;

  }

  // Get the remaining non-GPS derived telemetry values.
  // Since these are not reliant on the GPS we assume that we will always be able to get valid values for these.
#if defined (DS1820_TEMP_SENSOR_PRESENT)
  g_orion_current_telemetry.temperature_c = read_DS1820_temperature();
#elif defined (TMP36_TEMP_SENSOR_PRESENT)
  g_orion_current_telemetry.temperature_c = read_TEMP36_temperature();
#else
  // Note that if neither of the supported external temperature sensors are present 
  // we will encode the temperature later into the Pwr/dBm field using the internal processor temp
  g_orion_current_telemetry.temperature_c = 0;
#endif

  // Note that if neither of the supported external temperature sensors are present we will encode the temperature later into the Pwr/dBm field using the internal processor temp
  g_orion_current_telemetry.processor_temperature_c = read_processor_temperature();
  g_orion_current_telemetry.battery_voltage_v_x10 = read_voltage_v_x10();

}

// This function populates g_tx_data from the current_telemetry data, calculates the 6 char grid square and does unit conversions for most of the GPS data
void set_tx_data() {
  byte i;

  // Calculate the 6 character grid square and put it into g_tx_data.grid_sq_6char[]
  calculate_gridsquare_6char(g_orion_current_telemetry.latitude, g_orion_current_telemetry.longitude);

  // Copy the first four characters of the Grid Square to g_grid_loc[] for use in the Primary Type 1 WSPR Message
  for (i = 0; i < 4; i++ ) g_grid_loc[i] = g_tx_data.grid_sq_6char[i];
  g_grid_loc[i] = (char) 0; // g_grid_loc[4]

  // Set the transmit frequency
  g_beacon_freq_hz = get_tx_frequency();

  g_tx_data.altitude_m = g_orion_current_telemetry.altitude_cm / 100; // convert from cm to metres;
  g_tx_data.speed_kn = g_orion_current_telemetry.speed_mkn / 1000;   // covert from thousandths of a knot to knots
  g_tx_data.number_of_sats = g_orion_current_telemetry.number_of_sats;
  g_tx_data.gps_status = g_orion_current_telemetry.gps_status;
  //g_tx_data.gps_3d_fix_ok_bool = (bool)g_orion_current_telemetry.gps_status_ok;
  g_tx_data.temperature_c = g_orion_current_telemetry.temperature_c;
  g_tx_data.processor_temperature_c = g_orion_current_telemetry.processor_temperature_c;
  g_tx_data.battery_voltage_v_x10 = g_orion_current_telemetry.battery_voltage_v_x10;

  orion_log_telemetry (&g_tx_data);  // Pass a pointer to the g_tx_data structure
}

void prepare_telemetry()
{
  get_telemetry_data();

  set_tx_data();

  // We encode the 5th and 6th characters of the Grid square into the PWR/dBm field of the WSPR message.
  g_tx_pwr_dbm = encode_gridloc_char5_char6(g_tx_data.grid_sq_6char[4], g_tx_data.grid_sq_6char[5]);

} //end prepare_telemetry

void encode_and_tx_wspr_msg() {
  /**************************************************************************
    Transmit the Primary WSPR Message
    Loop through the transmit buffer, transmitting one character at a time.
  * ************************************************************************/
  uint8_t i;

  // Encode the primary message paramters into the TX Buffer
  jtencode.wspr_encode(g_beacon_callsign, g_grid_loc, g_tx_pwr_dbm, g_tx_buffer);

  // Reset the tone to 0 and turn on the TX output
  si5351bx_setfreq(SI5351A_WSPRTX_CLK_NUM, (g_beacon_freq_hz * 100ULL));

  // Turn off the PARK clock
  si5351bx_enable_clk(SI5351A_PARK_CLK_NUM, SI5351_CLK_OFF);

  // If we are using the TX LED turn it on
#if defined(TX_LED_PRESENT)
  digitalWrite(TX_LED_PIN, HIGH);
#endif

  // We need to synchronize the 1.46 second Timer/Counter-1 interrupt to the start of WSPR transmission as it is free-running.
  // We reset the counts to zero so we ensure that the first symbol is not truncated (i.e we get a full 1.46 seconds before the interrupt handler sets
  // the g_proceed flag).
  noInterrupts();
  TCNT1 = 0; // Clear the count for Timer/Counter-1
  GTCCR |= (1 << PSRSYNC); // Do a reset on the pre-scaler. Note that we are not using Timer 0, it shares a prescaler so it would also be impacted.
  interrupts();
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
  si5351bx_setfreq(SI5351A_PARK_CLK_NUM, (PARK_FREQ_HZ * 100ULL)); // Turn on Park Clock


  // If we are using the TX LED turn it off
#if defined (TX_LED_PRESENT)
  digitalWrite(TX_LED_PIN, LOW);
#endif

  delay(1000); // Delay one second
} // end of encode_and_tx_wspr_msg()


void get_gps_fix_and_time() {
  /*********************************************
    Get the latest GPS Fix
  * ********************************************/
  while (gps.available(gpsPort)) {
    fix = gps.read();

    if (fix.valid.time) {

      // If we have a valid fix, set the Time on the Arduino if needed
      if ( timeStatus() == timeNotSet ) { // System date/time isn't set so set it
        setTime(fix.dateTime.hours, fix.dateTime.minutes, fix.dateTime.seconds, fix.dateTime.date, fix.dateTime.month, fix.dateTime.year);
        g_chrono.start();

        // If we are using the SYNC_LED
#if defined (SYNC_LED_PRESENT)
        if (timeStatus() == timeSet)
          digitalWrite(SYNC_LED_PIN, HIGH); // Turn LED on if the time is synced
        else
          digitalWrite(SYNC_LED_PIN, LOW); // Turn LED off
#endif
      }


      if (g_chrono.hasPassed(TIME_SET_INTERVAL_MS, true)) { // When the time set interval has passed, restart the Chronometer set system time again from GPS
        setTime(fix.dateTime.hours, fix.dateTime.minutes, fix.dateTime.seconds, fix.dateTime.date, fix.dateTime.month, fix.dateTime.year);

        // If we are using the SYNC_LED
#if defined (SYNC_LED_PRESENT)
        if (timeStatus() == timeSet)
          digitalWrite(SYNC_LED_PIN, HIGH); // Turn LED on if the time is synced
        else
          digitalWrite(SYNC_LED_PIN, LOW); // Turn LED off
#endif
      }


    } // end if fix.valid.time
  } // end while

}  // end get_gps_fix_and_time()



OrionAction process_orion_sm_action (OrionAction action) {
  /****************************************************************************************************
    This is where all of the work gets triggered by processing Actions returned by the state machine.
    We don't need to worry about state we just do what we are told.
  ****************************************************************************************************/
  OrionAction returned_action = NO_ACTION;

  switch (action) {


    case NO_ACTION :
      returned_action = NO_ACTION;
      break;


    case WSPR_TX_INT_SETUP_ACTION :
      // This action is used to handle the case where self calibration is not supported by the board in use.
      // Usually we rely on the post-calibration clean-up to setup and enable the WSPR TX Interrupt
      // so this is no longer done in setup(). In the case where we skip the Startup self-calibration,
      // because self-calibration isn't suported, we need a mechanism to setup this timer Interrupt, so
      // here it is.

      // Setup 1.46 sec Timer1 interrupt for WSPR Transmission
      wspr_tx_interrupt_setup();
      returned_action = NO_ACTION;
      break;


    case CALIBRATION_ACTION :

      // re-initialize Interrupts for calibration
      reset_for_calibration();

      //TODO This should be modified with a boolean return code so we can handle calibration fail.
      do_calibration(FINE_CORRECTION_STEP);

      // Restore Timer1 interrupt for WSPR Transmission
      wspr_tx_interrupt_setup();
      returned_action = orion_state_machine(CALIBRATION_DONE_EV);
      break;

    case STARTUP_CALIBRATION_ACTION :

      // Initialize Interrupts for Initial calibration
      setup_calibration();

      //TODO This should be modified with a boolean return code so we can handle calibration fail.
      do_calibration(COARSE_CORRECTION_STEP); // Initial calibration with 1 Hz correction step

      // Reset the Timer1 interrupt for WSPR transmission
      wspr_tx_interrupt_setup();

      returned_action = orion_state_machine(CALIBRATION_DONE_EV);
      break;


    case GET_TELEMETRY_ACTION :
      prepare_telemetry();

      // Tell the Orion state machine that we have the telemetry
      returned_action = orion_state_machine(TELEMETRY_DONE_EV);
      break;


    case TX_WSPR_MSG1_ACTION :

      // Encode the 5th and 6th Maidenhead Grid characters into the pwr/dBm field
      // g_tx_pwr_dbm = encode_gridloc_char5_char6();
      // Note that this is actually done as part of the Telemetry gathering so no need to redo it.
      // It is the same story for :
      //  g_beacon_freq_hz = get_tx_frequency(); .. this is already covered for the Primary Msg in the Telemetry Phase. 

      // Encode and transmit the Primary WSPR Message
      encode_and_tx_wspr_msg();

      orion_log_wspr_tx(PRIMARY_WSPR_MSG, g_tx_data.grid_sq_6char, g_beacon_freq_hz, g_tx_pwr_dbm); // If TX Logging is enabled then ouput a log

      // Tell the Orion state machine that we are done tranmitting the Primary WSPR message and update the current_action
      returned_action = orion_state_machine(PRIMARY_WSPR_TX_DONE_EV);
      break;


    case TX_WSPR_MIN02_ACTION :
    case TX_WSPR_MIN22_ACTION :
    case TX_WSPR_MIN42_ACTION :  // TX altitude Telemetry

      // At  hh:02, hh:22, hh:42 encode and transmit the Secondary WSPR Message with altitude encoded into the pwr/dBm field
      g_tx_pwr_dbm = encode_altitude(g_tx_data.altitude_m);
      
      // Set the transmit frequency. This will ensure that QRM Avoidance is utilized for Telemetry Messages as well (i.e different TX frequency than Primary WSPR Msg)
      g_beacon_freq_hz = get_tx_frequency();
      
      encode_and_tx_wspr_msg();
      orion_log_wspr_tx(ALTITUDE_TELEM_MSG, g_tx_data.grid_sq_6char, g_beacon_freq_hz, g_tx_pwr_dbm); // If TX Logging is enabled then ouput a log

      // Tell the Orion state machine that we are done tranmitting the Telemetry WSPR message and update the current_action
      returned_action = orion_state_machine(SECONDARY_WSPR_TX_DONE_EV);
      break;

    case TX_WSPR_MIN12_ACTION :
    case TX_WSPR_MIN52_ACTION :  //  -  TX voltage Telemetry

      // At hh:12, hh:22, hh:52 encode and transmit the Secondary WSPR Message with voltage encoded into the pwr/dBm field
      g_tx_pwr_dbm = encode_voltage(g_tx_data.battery_voltage_v_x10); 

      // Set the transmit frequency. This will ensure that QRM Avoidance is utilized for Telemetry Messages as well (i.e different TX frequency than Primary WSPR Msg)
      g_beacon_freq_hz = get_tx_frequency();
      
      encode_and_tx_wspr_msg();
      orion_log_wspr_tx(VOLTAGE_TELEM_MSG, g_tx_data.grid_sq_6char, g_beacon_freq_hz, g_tx_pwr_dbm); // If TX Logging is enabled then ouput a log

      // Tell the Orion state machine that we are done tranmitting the Telemetry WSPR message and update the current_action
      returned_action = orion_state_machine(SECONDARY_WSPR_TX_DONE_EV);
      break;


    case TX_WSPR_MIN32_ACTION : //  -  TX temperature Telemetry
      // At hh:32 encode and transmit the Secondary WSPR Message with temperature encoded into the pwr/dBm field
#if defined (DS1820_TEMP_SENSOR_PRESENT) | defined (TMP36_TEMP_SENSOR_PRESENT )
      g_tx_pwr_dbm = encode_temperature(g_tx_data.temperature_c); // Use Sensor data
#else
      g_tx_pwr_dbm = encode_temperature(g_tx_data.processor_temperature_c); // Use internal processor temperature
#endif
      // Set the transmit frequency. This will ensure that QRM Avoidance is utilized for Telemetry Messages as well (i.e different TX frequency than Primary WSPR Msg)
      g_beacon_freq_hz = get_tx_frequency();
      
      encode_and_tx_wspr_msg();
      orion_log_wspr_tx(TEMPERATURE_TELEM_MSG, g_tx_data.grid_sq_6char, g_beacon_freq_hz, g_tx_pwr_dbm); // If TX Logging is enabled then ouput a log

      // Tell the Orion state machine that we are done transmitting the Telemetry WSPR message and update the current_action
      returned_action = orion_state_machine(SECONDARY_WSPR_TX_DONE_EV);
      break;


    default :
      returned_action = NO_ACTION;
      break;


  } // end switch (action)

  return returned_action;

} //  process_orion_sm_action


OrionAction orion_scheduler() {
  /*********************************************************************
    This is the scheduler code that determines the Orion Beacon schedule
  **********************************************************************/
  byte Second; // The current second
  byte Minute; // The current minute
  byte i;
  OrionAction returned_action = NO_ACTION;


  if (timeStatus() == timeSet) { // We have valid time from the GPS otherwise do nothing

    // The scheduler will get called many times per second but we only want it to run once per second,
    // otherwise we might generate multiples of the same time event on these extra passes through the scheduler.
    Second = second();

    // If we are on the same second as the last time we went through here then we bail-out with NO_ACTION returned.
    // This prevents us from sending time events more than once as our time resolution is only one second but we might
    // make it back here in less than 1 second.
    if (Second == g_last_second) return returned_action;

    // If the current second is different from the last time, then it has been updated so we run.

    g_last_second = Second; // Remember what second we are currently on for the next time the scheduler is called

    Minute = minute();

    if (Second == 1) { // To simplify things we trigger everything at the one second mark if we are on the correct minute

      switch (Minute) {

        // Primary WSPR Transmission Triggers every 10th minute of the hour on the first second
        case 0  :
        case 10 :
        case 20 :
        case 30 :
        case 40 :
        case 50 :
          // Primary WSPR transmission should start on the 1st second of the minute, but there's a slight delay
          // in this code because it is limited to 1 second resolution.
          return (orion_state_machine(PRIMARY_WSPR_TX_TIME_EV)); // This is a bit time critical so we try to minimize any extra processing

        // These are also time critical as they trigger Telemetry messages so we try to minimize any extra processing by returning directly
        // Telemetry is sent in the next even minute slot after the Primary message
        case 2  : return (orion_state_machine(WSPR_TX_TIME_MIN02_EV));
        case 12 : return (orion_state_machine(WSPR_TX_TIME_MIN12_EV));
        case 22 : return (orion_state_machine(WSPR_TX_TIME_MIN22_EV));
        case 32 : return (orion_state_machine(WSPR_TX_TIME_MIN32_EV));
        case 42 : return (orion_state_machine(WSPR_TX_TIME_MIN42_EV));
        case 52 : return (orion_state_machine(WSPR_TX_TIME_MIN52_EV));

        case 9  :
        case 19 :
        case 29 :
        case 39 :
        case 49 :
        case 59 :
          // If are one minute prior to a scheduled Beacon Transmission, trigger collection of new telemetry info
          returned_action =  (orion_state_machine(TELEMETRY_TIME_EV));
          break;

        default :
          break;


      } // end switch (Minute)


    } // end if (Second == 1)

    /*
       // We beacon every 10th minute of the hour so we use minute() modulo 10 (i.e. on 00, 10, 20, 30, 40, 50)
       if (((Minute % 10) == 0) && (Second == 1)) {
         // Primary WSPR transmission should start on the 1st second of the minute, but there's a slight delay
         // in this code because it is limited to 1 second resolution.
         returned_action = orion_state_machine(PRIMARY_WSPR_TX_TIME_EV);
       }
       else if (Second == 30) { // Note the choice of a second value of 30 is arbitrary

         // check once per minute at 30 seconds after the minute to see if it is time to collect new telemetry data
         for (i = 9; i < 60; i = i + 10) {
           // i.e. i is 9, 19, 29, 39, 49, 59 which is one minute prior to transmission time

           if (Minute == i) {
             // If are about thirty seconds prior to a scheduled Beacon Transmission collect new telemetry info
             returned_action = orion_state_machine(TELEMETRY_TIME_EV);
             break;
           }
         } // end for

       } // end else if

    */

  } // end if timestatus == timeset

  return returned_action;
} // end orion_scheduler()

void wspr_tx_interrupt_setup() {

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
}

void setup() {

  // Ensure that GPS is powered up if power disable feature supported
#if defined(GPS_POWER_DISABLE_SUPPORTED)
  pinMode(GPS_POWER_DISABLE_PIN, OUTPUT);
  digitalWrite(GPS_POWER_DISABLE_PIN, LOW);
#endif

  // Ensure that Si5351a is powered up if power disable feature supported
#if defined(SI5351_POWER_DISABLE_SUPPORTED)
  pinMode(TX_POWER_DISABLE_PIN, OUTPUT);
  digitalWrite(TX_POWER_DISABLE_PIN, LOW);
#endif

  // Use the TX_LED_PIN as a transmit indicator if it is present
#if defined (TX_LED_PRESENT)
  pinMode(TX_LED_PIN, OUTPUT);
  digitalWrite(TX_LED_PIN, LOW);
#endif

  // Use the SYNC_LED_PIN to indicate the beacon has valid time from the GPS, but only if it present
#if defined (SYNC_LED_PRESENT)
  pinMode(SYNC_LED_PIN, OUTPUT);
  digitalWrite(SYNC_LED_PIN, LOW);
#endif

  pinMode(CAL_FREQ_IN_PIN, INPUT); // This is the frequency input must be D5 to use Timer1 as a counter
  pinMode(GPS_PPS_PIN, INPUT);

  // Start Hardware serial communications with the GPS
  gpsPort.begin(GPS_SERIAL_BAUD);

  // Initialize the Si5351
  si5351bx_init();

  // Setup WSPR TX output
  si5351bx_setfreq(SI5351A_WSPRTX_CLK_NUM, (g_beacon_freq_hz * 100ULL));
  si5351bx_enable_clk(SI5351A_WSPRTX_CLK_NUM, SI5351_CLK_OFF); // Disable the TX clock initially

  // Set PARK CLK Output - Note that we leave SI5351A_PARK_CLK_NUM running at 108 Mhz to keep the SI5351 temperature more constant
  // This minimizes thermal induced drift during WSPR transmissions. The idea is borrowed from G0UPL's PARK feature on the QRP Labs U3S
  si5351bx_setfreq(SI5351A_PARK_CLK_NUM, (PARK_FREQ_HZ * 100ULL)); // Turn on Park Clock

  // Setup the software serial port for the serial monitor interface
  serial_monitor_begin();

  // Set the intial state for the Orion Beacon State Machine
  orion_sm_begin();

  // Read unused analog pin (not connected) to generate a random seed for QRM avoidance feature
  randomSeed(analogRead(ANALOG_PIN_FOR_RNG_SEED));

  // Tell the state machine that we are done SETUP
  g_current_action = orion_state_machine(SETUP_DONE_EV);

} // end setup()


void loop() {
  /****************************************************************************************************************************
    This loop constantly updates the system time from the GPS and calls the scheduler for the operation of the Orion Beacon
  ****************************************************************************************************************************/
  // We need to ensure that the GPS is up and running and we have valid time before we proceed
  // This should only get invoked on system cold start. It ensures that the scheduler and logging will properly function

  /*if (orion_sm_get_current_state()== CALIBRATE_ST) {
    while (timeStatus() == timeNotSet ) { // System date/time isn't set yet
     get_gps_fix_and_time(); // Try to get a GPS fix
     delay(1000); // Wait one second
    }

    }
  */
  // This triggers actual work when the state machine returns an OrionAction
  while (g_current_action != NO_ACTION) {
    g_current_action = process_orion_sm_action(g_current_action);
  }

  // Get the current GPS fix and update the system clock time if needed
  get_gps_fix_and_time();


  // Process serial monitor input
  serial_monitor_interface();

  // Call the scheduler to determine if it is time for any action
  g_current_action = orion_scheduler();

} // end loop ()

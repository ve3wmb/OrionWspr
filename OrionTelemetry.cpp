/*
   OrionTelemetry.cpp - Telemetry Data gathering, formatting and encoding

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
//#include <int.h>
#include "OrionXConfig.h"
#include "OrionBoardConfig.h"
#include "OrionSerialMonitor.h"

#if defined (DS1820_TEMP_SENSOR_PRESENT)
  #include <OneWire.h>
  #include <DallasTemperature.h>
#endif

#if defined (DS1820_TEMP_SENSOR_PRESENT)
OneWire oneWire(ONE_WIRE_BUS);               // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&oneWire);        // Pass our oneWire reference to Dallas Temperature.

// Read temperature in C from Dallas DS1820 temperature sensor
int read_DS1820_temperature() {

  sensors.requestTemperatures();

  // After we have the temperatures, we use the function ByIndex, and in this case only the temperature from the first sensor
  // The temperature is in degrees C, returned as a signed int.
  return ((int)(sensors.getTempCByIndex(0)));
}
#endif // DS1820_TEMP_SENSOR_PRESENT

#if defined (TMP36_TEMP_SENSOR_PRESENT)
int read_TEMP36_temperature() {
  return (double)analogRead(TMP36_PIN) / 1024 * 330 - 50;
}
#endif


int read_voltage_v_x10() {

  int AdcCount, i, voltage_v_x10;
  float Vpower, sum;

if (VCC_SAMPLING_SUPPORTED == true) {

  sum = 0; 

  analogReference(DEFAULT); // ensure that we are using the default 3.3v voltage reference for ADC
  
  // Do a few throw-away reads before the real ones so things can settle down
  for (i = 0; i < 5; i++) {
    AdcCount = analogRead(Vpwerbus); // read Vpwerbus
  }
   
  // Read the voltage 10 times so we can calculate an average
  for (i = 0; i < 10; i++) {

    // Arduino 10bit ADC; 3.3v external AREF each count of 1 = 0.00322265625V
    AdcCount = analogRead(Vpwerbus); // read Vpwerbus
    Vpower = AdcCount * 0.00322 * VpwerDivider;
    sum = sum + Vpower ;
  }

  Vpower = sum / 10.0; // Calculate the average of the 10 voltage samples

  // Shift the voltage one decimal place to the left and convert to an int
  voltage_v_x10 = (int) (Vpower * 10); // i.e. This converts 3.3333 volt reading to 33 representing 3.3 v
}
  
else { // VCC_SAMPLING_SUPPORTED == false

  // Just return the defined OPERATING_VOLTAGE_Vx10 value
  voltage_v_x10 = OPERATING_VOLTAGE_Vx10;
  
}
   


return voltage_v_x10;
 
}

int read_processor_temperature() {

  unsigned int wADC;
  double temp_c;

  // The internal temperature has to be used
  // with the internal reference of 1.1V.
  // Channel 8 can not be selected with
  // the analogRead function yet.

  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  // enable the ADC

  delay(20);            // wait for voltages to become stable.

  ADCSRA |= _BV(ADSC);  // Start the ADC

  // Detect end-of-conversion
  while (bit_is_set(ADCSRA, ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;

  // The offset of 324.31 could be wrong. It is just an indication.
  temp_c = (wADC - 324.31) / 1.22;

  // The returned temperature is in degrees Celsius.
  return (((int)temp_c));
}


uint8_t encode_temperature (int temperature_c) {
  uint8_t ret_value = 0;

  // Encoding of Temperature for PWR/dBm Field

  switch ( temperature_c ){
  
    case 35 ... 100: // aka >= 35 celsius 
      ret_value = 0;
      break;

    case 30 ... 34:
      ret_value = 3;
      break;

    case 25 ... 29:
      ret_value = 7;
      break;

    case 20 ... 24:
      ret_value = 10;
      break;

    case 15 ... 19:
      ret_value = 13;
      break;

    case 10 ... 14:
      ret_value = 17;
      break;

    case 5 ... 9:
      ret_value = 20;
      break;

    case 0 ... 4:
      ret_value = 23;
      break;

    case -5 ... -1:
      ret_value = 27;
      break;

    case -10 ... -6:
      ret_value = 30;
      break;

    case -15 ... -11:
      ret_value = 33;
      break;

    case -20 ... -16:
      ret_value = 37;
      break;

    case -25 ... -21:
      ret_value = 40;
      break;

    case -30 ... -26:
      ret_value = 43;
      break;

    case -35 ... -31:
      ret_value = 47;
      break;

    case -40 ... -36:
      ret_value = 50;
      break;

    case -45 ... -41:
      ret_value = 53;
      break;

    case -50 ... -46:
      ret_value = 57;
      break;

    default: // less than -50 celsius
      ret_value = 60;
      break;

  }
  return ret_value;
}

uint8_t encode_voltage (int voltage_v_x10) {
  uint8_t ret_value = 0;
  
  switch ( voltage_v_x10 ){  // Note: Divide voltage_v_x10 by 10 to get the actual voltage (i.e 36 = 3.6 v)
  
    case 33 : // aka >= 3.3 volts 
      ret_value = 0;
      break;
      
    case 34 :
      ret_value = 3;
      break;
      
    case 35 :
      ret_value = 7;
      break;
      
    case 36 :
      ret_value = 10;
      break;
      
    case 37 :
      ret_value = 13;
      break;
      
    case 38 :
      ret_value = 17;
      break;
      
    case 39 :
      ret_value = 20;
      break;
      
    case 40 :
      ret_value = 23;
      break;
      
    case 41 :
      ret_value = 27;
      break;
      
    case 42 :
      ret_value = 30;
      break;
      
    case 43 :
      ret_value = 33;
      break;
      
    case 44 :
      ret_value = 37;
      break;
      
    case 45 :
      ret_value = 40;
      break;
      
    case 46 :
      ret_value = 43;
      break;
      
    case 47 :
      ret_value = 47;
      break;
      
    case 48 :
      ret_value = 50;
      break;
      
    case 49 :
      ret_value = 53;
      break;
      
    case 50 :
      ret_value = 57;
      break;
      
    default: // greater than 5 volts
      ret_value = 60;
      break;

  } // end switch (voltage)

  return ret_value;

}

uint8_t encode_altitude (int altitude_m) {
  uint8_t ret_value = 0;

  // Encoding of altitude in metres for PWR/dBm Field
  // Note that on both ends of the scale the resolution is 500m whereas mid-scale it
  // changes to 1000m resolution. 
 
  switch (altitude_m) { 
    
    case 0 ... 499 :
      ret_value = 0;
      break;

    case 500 ... 999 :
      ret_value = 3;
      break;

    case 1000 ... 1499 :
      ret_value = 7;
      break;

    case 1500 ... 1999 :
      ret_value = 10;
      break;

    case 2000 ... 2499 :
      ret_value = 13;
      break;

    case 2500 ... 2999 :
      ret_value = 17;
      break;

    case 3000 ... 3999 :
      ret_value = 20;
      break;

    case 4000 ... 4999 :
      ret_value = 23;
      break;

    case 5000 ... 5999 :
      ret_value = 27;
      break;

    case 6000 ... 6999 :
      ret_value = 30;
      break;

    case 7000 ... 7999 :
      ret_value = 33;
      break;

    case 8000 ... 8499 :
      ret_value = 37;
      break;

    case 8500 ... 8999 :
      ret_value = 40;
      break;

    case 9000 ... 9499 :
      ret_value = 43;
      break;

    case 9500 ... 9999 :
      ret_value = 47;
      break;

    case 10000 ... 10499 :
      ret_value = 50;
      break;

    case 10500 ... 10999 :
      ret_value = 53;
      break;

    case 11000 ... 11499 :
      ret_value = 57;
      break;

    default : //  >= 12000 metres
      ret_value = 60;
      break;
  }
  return ret_value;
}

// This function implements the KISS Telemetry scheme proposed by VE3GTC.
// We use the PWR/dBm field in the WSPR Type 1 message to encode the 5th and 6th characters of the
// 6 character Maidenehad Grid Square, following the WSPR encoding rules for this field.
// The 6 Character Grid Square is calculated from the GPS supplied Latitude and Longitude.
// A four character grid square is 1 degree latitude by 2 degrees longitude or approximately 60 nautical miles
// by 120 nautical miles respectively (at the equator). This scheme increases resolution to 1/3 of degree
// (i.e about 20 nautical miles). We encode the 5th and 6th characters of the 6 character Grid Locator into the
// Primary Type 1 message in the PWR (dBm) field,as follows :
//
// Sub-square Latitude (character #6)

//  a b c d e f g                encodes as : 0
//  h i j k l m n o p q          encodes as : 3
//  r s t u v w x                encodes as : 7

// Subsquare Longitude (character #5)

//  a b c             encodes as : 0 (space)
//  d e f g h i       encodes as : 1
//  j k l             encodes as : 2
//  m n o             encodes as : 3
//  p q r s t u       encodes as : 4
//  v w x             encodes as : 5
//
// So FN25di would have FN25 encoded is the GRID field and 'DI' encoded in the PWR/dBm field as 13.
//
uint8_t encode_gridloc_char5_char6(char gridsq_char5, char gridsq_char6) {

  uint8_t latitude = 0;
  uint8_t longitude = 0;

  // Encode the 5th character of the 6 char Grid locator first, this is the longitude portion of the sub-square
  switch (gridsq_char5) {

    case 'A' : case 'B' : case 'C' :
      longitude = 0;
      break;

    case 'D' : case 'E' : case 'F' : case 'G' : case 'H' : case 'I' :
      longitude = 1;
      break;

    case 'J' : case 'K' : case 'L' :
      longitude = 2;
      break;

    case 'M' : case 'N' : case 'O' :
      longitude = 3;
      break;

    case 'P' : case 'Q' : case 'R' : case 'S' : case 'T' : case 'U' :
      longitude = 4;
      break;

    case 'V' : case 'W' : case 'X' :
      longitude = 5;
      break;

    default :
      // We should never get here so Swerr
      swerr(10, gridsq_char5);
      break;
  } // end switch on 5th character of Grid Locator

  longitude = longitude * 10; // This shifts the longitude value one decimal place to the left (i.e a 1 becomes 10).

  // Now we encode the 6th character (array indexing starts at 0) of the 6 character Grid locator, which represents the latitude portion of the sub-square
  switch (gridsq_char6) {

    case 'A' : case 'B' : case 'C' : case 'D' : case 'E' : case 'F' : case 'G' :
      latitude = 0;
      break;

    case 'H' : case 'I' : case 'J' : case 'K' : case 'L' : case 'M' : case 'N' : case 'O' : case 'P' : case 'Q' :
      latitude = 3;
      break;

    case 'R' : case 'S' : case 'T' : case 'U' : case 'V' : case 'W' : case 'X' :
      latitude = 7;
      break;

    default :
      // We should never get here so Swerr
      swerr(11, gridsq_char6);
      break;
  }

  return (longitude + latitude);

} // end encode_gridloc_char5_char6

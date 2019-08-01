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
#include <int.h>
#include "OrionXConfig.h"
#include "OrionBoardConfig.h"

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


int read_voltage_v_x10() {

  int AdcCount, i, voltage_v_x10;
  float Vpower, sum;

  sum = 0;

  // Read the voltage 10 times so we can calculate an average
  for (i = 0; i < 10; i++) {

    // Arduino 10bit ADC; 3.3v external AREF each count of 1 = 0.00322265625V
    AdcCount = analogRead(Vpwerbus); // read Vpwerbus
    Vpower = AdcCount * 0.00322 * 1.33333;
    sum = sum + Vpower ;
  }

  Vpower = sum / 10.0; // Calculate the average of the 10 voltage samples

  // Shift the voltage one decimal place to the left and convert to an int
  voltage_v_x10 = (int) (Vpower * 10); // i.e. This converts 3.3333 volt reading to 33 representing 3.3 v

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

#ifndef ORIONBOARDCONFIG_H
#define ORIONBOARDCONFIG_H

 //  K1FMv1r3BoardConfig.h - Orion Board Configuration for K1FM PicoBv1.3 board
 //  SW serial to GPS, HW serial for debug monitor, Hardware I2C
 //  GPS_POWER_DISABLE and Si5351_POWER_DISABLE supported.

/*
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
#include <Arduino.h>

// THIS FILE CONTAINS THE USER MODIFIABLE #DEFINES TO CONFIGURE A SPECIFIC BOARD TO USE THE ORION WSPR BEACON CODE

#define BOARDNAME " - K1FM v1.3"        // This string is output along with code version using the 'v' command in the monitor

// GPS Communicates with processor via Hardware serial 
//#define GPS_USES_HW_SERIAL              // Comment out if ATMEGA328p communicates with GPS via Software Serial

// Debug serial uses a software serial port
//#define DEBUG_USES_SW_SERIAL            // Comment out if Serial Debug Monitor uses Hardware Serial 

// Processor talks to Si5351a using software I2C
//#define SI5351A_USES_SOFTWARE_I2C       // Comment out if  ATMEGA328p communicates with the Si5351a via Hardware I2C

#define SI5351_SELF_CALIBRATION_SUPPORTED  true // set to false if No Self calibration. It requires an unused Si5351 CLK output fed back to D5 

// Self Calibration uses External Interrupt on PIN D2 or D3 for GPS PPS signal.
// Comment this out if using PinChangeInterrupt on any other PIN 
// This must be defined if the GPS PPS PIN is connected to D2 or D3, otherwise commented out
#define GPS_PPS_ON_D2_OR_D3        //GPS PPS connects to D2 or D3 and thus can use an External Interrupt othwerwise 

// K1FM 1.2 boards and greater support the ability to enable/disable VCC power to GPS  
#define GPS_POWER_DISABLE_SUPPORTED

// K1FM 1.2 boards and greater support the ability to enable/disable VCC power to Si5351a
#define	SI5351_POWER_DISABLE_SUPPORTED

// External Temperature Sensor. If one of the following two are DEFINED this sensor data will be used for temperature telemetry
// otherwise we default to using the internal temperature sensor in the Atmega328p.
/// 
//define DS1820_TEMP_SENSOR_PRESENT  // Dallas Semiconductor DS18020 external One-wire sensor is present
#define TMP36_TEMP_SENSOR_PRESENT   // TMP36 - Placeholder for now, not yet supported. 

#if defined (TMP36_TEMP_SENSOR_PRESENT)
#define TMP36_PIN A1            // TMP36 Temperature sensor. Change to match PIN usage.
#endif

#if defined (DS1820_TEMP_SENSOR_PRESENT)
#define ONE_WIRE_BUS A0          // Dallas Temperature One-Wire Sensor change this to match PIN usage
#endif

#define VCC_SAMPLING_SUPPORTED true  // Board has the capabilty to sample VCC on Vpwerbus using VpwerDivider as a multiplying factor
#define Vpwerbus     A0          	// ADC input for Vpwrbus for measuring battery voltage
#define VpwerDivider 5.7         	// Multiplying factor for ADC voltage divider
	
// Comment these out if not using an LED to indicate TX or GPS Time Synch
//#define TX_LED_PRESENT           
//#define SYNC_LED_PRESENT       

/**********************************************************************************
Arduino Hardware Pin Configurations - change these to match your specific hardware
***********************************************************************************/

// The following two defines select the Arduino pins used for software Serial communications.
// The assumption is that if SW serial is used for communicating with the GPS, that hardware
// serial is used for the debug monitor, or vise versa. One of the two must use hardware serial.
#define SOFT_SERIAL_RX_PIN        4            
#define SOFT_SERIAL_TX_PIN        3            

// PIN definitions for Si5351a software I2C communication. 
// Ignore if using Hardware I2C with Wire Library to communicate with the Si5351a
// These are assuming Hardware Pin assignments compatible with the QRP Labs U3S & U3S-clones
#define SCL_PIN 1  //PB1
#define SCL_PORT PORTB
#define SDA_PIN 2 //PD2
#define SDA_PORT PORTD

// The following defines designate which pins are used for TX and Time Synch if 
// TX_LED_PRESENT and SYNC_LED_PRESENT are defined. They are ignored otherwise.
#define TX_LED_PIN              5             // TX LED on D4.
#define SYNC_LED_PIN            7             // LED on PIN D7 indicates GPS time synchronization. 
// Note the most Arduino Boards have a built-in LED that can be used for either of the above purposes referred to as LED_BUILTIN

#define ANALOG_PIN_FOR_RNG_SEED  A2              // Pin used to generate seed for Random number generator - must be a free analog pin (unused)
 

 
#define CAL_FREQ_IN_PIN 5        // This must be D5 as it is the external clock input for Timer1 when it is used as a counter. 
                                 // Otherwise Calibration is not supported for your board.
                                  
#define GPS_PPS_PIN 2            // This must be either 2 or 3 (i.e. D2 or D3) to use external interrupts, otherwise you must use a PinChangeInterrupt for PPS 
//#define GPS_PPS_PIN A5         // DL6OW STELLA boards and other U3S Clones use A5/ADC5 (physical pin #28) for PPS. This uses PCINT13

// Enable PIN for GPS VCC (LOW = enabled, HIGH = disabled)
#ifdef GPS_POWER_DISABLE_SUPPORTED
	#define GPS_POWER_DISABLE_PIN	7			// Pin D7
#endif

// Enable PIN for Si5351a TX VCC (LOW = enabled, HIGH = disabled)
#ifdef SI5351_POWER_DISABLE_SUPPORTED
	#define TX_POWER_DISABLE_PIN		6			// Pin D6
#endif


/***********************************************************
   Si5351a Configuration Parameters
 ***********************************************************/
#define SI5351A_PARK_CLK_NUM    1              // The Si5351a Clock Number output used to mimic the QRP Labs U3S Park feature. This needs to be an unused clk port.
                                               // I recommend terminating this port to ground via a 47 to 56 ohm resistor.
#define SI5351A_CAL_CLK_NUM     2              // Calibration Clock Number                                
#define SI5351A_WSPRTX_CLK_NUM  0              // The Si5351a Clock Number output used for the WSPR Beacon Transmission



/*********************************************************************************************************************** 
*  You need to calibrate your Si5351a and substitute the your correction value for SI5351A_CLK_FREQ_CORRECTION below.
*  See OrionSi5351_calibration.ino sketch. You may also need to modify SI5351BX_XTALPF
*  in OrionSi5351.h if you need a crystal load capacitance other that 8 pf.
************************************************************************************************************************/
#define SI5351A_CLK_FREQ_CORRECTION  0    //You must modify this with the correction factor for the Si5351a clock

#define SI5351BX_XTALPF   3               // 1:6pf  2:8pf  3:10pf -  assuming 10 pF, otherwise change

// If using 27mhz crystal, set XTAL=27000000, MSA=33.  Then vco=891mhz
#define SI5351BX_XTAL 2500000000ULL      // Crystal freq in centi Hz 
#define SI5351BX_MSA  35                // VCOA is at 25mhz*35 = 875mhz
                                               

/**************************************************************************************
   GPS CONFIGURATION Parameters - IT ASSUMES THE USE OF A HARWARE SERIAL PORT
 **************************************************************************************/
// GPS Serial port Baud rate - For now only a hardware serial connection to the GPS is supported.
#define GPS_SERIAL_BAUD         9600          // Baudrate for the GPS Serial port

#define MONITOR_SERIAL_BAUD     9600          // Baudrate for Orion Serial Monitor      


/***********************************************************
   Parameters dependant on Processor CPU Speed
 ***********************************************************/
// The following value WSPR_CTC is used for Timer1 that generates a 1.46 Hz interrupt.

// THE CURRENT VALUE for WSPR_CTC ASSUMES AN 8 MHZ PROCESSOR CLOCK !

// If you are using most Arduinos (i.e Nano, UNO etc) they use a 16 Mhz clock and the value 10672 must be substituted.
// ie. #define WSPR_CTC                10672       // CTC value for WSPR on Arduino using 16 Mhz clock (i.e. Nano, Uno etc)
// The formula to calculate WSPR_CTC is: 1.4648 = CPU_CLOCK_SPEED_HZ / (PRESCALE_VALUE) x (WSPR_CTC + 1)
#define WSPR_CTC                5336               // CTC value for WSPR on Arduino using an 8 Mhz clock (i.e. Arduino Pro Mini 3.3v 8 Mhz)

#define SI5351_CAL_TARGET_FREQ  320000000ULL; //This is calculated as CPU_CLOCK_SPEED_HZ / 2.5 expressed in hundredths of Hz. Assumes 8 Mhz clk.

#endif

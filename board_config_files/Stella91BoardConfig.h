#ifndef ORIONBOARDCONFIG_H
#define ORIONBOARDCONFIG_H

 //  Stella91BoardConfig.h - Orion Board Configuration for DL6OW STELLA 9.1 - U3S Clone
 //  HW serial to GPS, SW serial for debug monitor, software I2C, supports self-calibration
 //  with PinChangeInterrupts on A5/ADC5.

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

#define BOARDNAME " - STELLA 9.1"        // This string is output along with code version using the 'v' command in the monitor
/*
** Note: When using a previously unprogrammed ATMega chip it is important to "burn bootloader" first so that the fuses are set correctly
** Use Arduino Pro Mini 3.3v / 8 Mhz as Board / Processor Type for 3.3 v operation. 
**
** ATMEL AVR in circuit programming (ISP) header pinout (view from top of header)
** 
**         6 pin                    10 pin 
**         +----+                   +----+    
**    MISO |1  2| VCC          MOSI |1  2| VCC
**     SCK |3  4| MOSI     (unused) |3  4| GND
**   RESET |5  6| GND         RESET |5  6| GND
**         +----+               SCK |7  8| GND
**                             MISO |9 10| GND
**                                  +----+
** 
** 
**  Arduino Function                            ATMega168/328                      Arduino Function
** 	                                              +---v---+
** 	reset		      		(PCINT14/RESET)   PC6 |1    28| PC5 (ADC5/SCL/PCINT13) analog input A5
**  digital pin D0 (RX)	        (PCINT16/RXD) PD0 |2    27| PC4 (ADC4/SDA/PCINT12) analog input A4
**  digital pin D1 (TX)         (PCINT17/TXD) PD1 |3    26| PC3 (ADC3/PCINT11)     analog input A3 
**  digital pin D2             (PCINT18/INT0) PD2 |4    25| PC2 (ADC2/PCINT10)     analog input A2
**  digital pin D3 (PWM)  (PCint19/OC2B/INT1) PD3 |5    24| PC1 (ADC1/PCINT9)      analog input A1 
**  digital pin D4           (PCINT20/XCK/T0) PD4 |6    23| PC0 (ADC0/PCINT8)      analog input A0
**  VCC                                       VCC |7    22| GND                    GND
**  GND                                       GND |8    21| AREF                   analog reference
**  crystal              (PCINT6/XTAL1/TOSC1) PB6 |9    20| AVCC                   analog VCC
**  crystal              (PCINT7/XTAL2/TOSC2) PB7 |10   19| PB5 (SCK/PCINT5)       digital pin D13
**  digital pin D5 (PWM)    (PCINT21/OC0B/T1) PD5 |11   18| PB4 (MISO/PCINT4)      digital pin D12
**  digital pin D6 (PWM)  (PCINT22/OC0A/AIN0) PD6 |12   17| PB3 (MOSI/OC2A/PCINT3) digital pin D11 (PWM)
**  digital pin D7             (PCINT23/AIN1) PD7 |13   16| PB2 (SS/OC1B/PCINT2)   digital pin D10 (PWM)
**  digital pin D8         (PCINTO/CKLO/ICP1) PB0 |14   15| PB1 (OC1A/PCINT1)      digital pin D9  (PWM)
**                                                +-------+
**
**  


DL6OW Stella boards Pin Configuration

atmega328 pin #                                    

      1        PD3 (PCNINT19/OC2B/INT1)
	  2        PD4 (PCINT20/XCK/T0)
	  3        GND
	  4        VCC
	  5        GND
	  6        VCC
	  7        PB6 (PCINT6/XTAL1/TOSC1)
	  8        PB7 (PCINT7/XTAL2/TOSC2)
	  
	  9        PD5 (PCINT21/OC0B/T1)                [U3S - si5351a CLK2 for self-calibration]      Arduino D5
	 10        PD6 (PCINT22/OC0A/AN0)
	 11        PD7 (PCINT23/AN1)
	 12        PB0 (PCINT0/CLK0/ICP1)
	 13        PB1 (PCINT1/OC1A)                    [U3S - si5351a SCL]                            Arduino D9
	 14        PB2 (PCINT2/!SS/OC1B)
	 15        PB3 (PCINT3/OC2A/MOSI)               [software serial TX Data]                      Arduino D11
	 16        PB4 (PCINT4/MISO)                    [software serial RX Data use 10k pullup]       Arduino D12
	 
	 17        PB5 (SCK/PCINT5)
	 18        AVCC
	 19        ADC6
	 20        AREF
	 21        GND
	 22        ADC7
	 23        PCO (ADC0/PCINT8)                    [OneWire Bus for DS1820 temp sensor]          Arduino  A0
	 24        PC1 (ADC1/PCINT9)					[Unused analog port for random number seed]	  Arduino A1 
	 
	 25        PC2 (ADC2/PCINT10)
	 26        PC3 (ADC3/PCINT11)                   [V+ input to ADC for voltage measument]       Arduino A3 
	 27        PC4 (ADC4/SDA/PCINT11)
	 28        PC5 (ADC5/SCL/PCINT13)               [GPS pin  3 - time pulse PPS for calibration] Arduino A5
	 29        PC6 (!RESET/PCINT14)
	 30        PD0 (RXD/PCINT16)                    [GPS pin 20 - TXD]                            Arduino D0
	 31        PD1 (TXD/PCINT17)                    
	 32        PD2 (INT0/PCINT18)                   [U3S - si5351a SDA]                         Arduino D2
	 
	 
	 
Divider uses 1K and 3K ohm
 
4.766667 Vpwrbus = 1.1000000769 V output from divider
 
Set arduino AREF to use internal 1.1V reference
 
Arduino 10bit ADC; each count of 1 = 0.004654V
*/

/*************************************************************************
/* The following #defines select CODE OPTIONALITY via conditional compile*
*************************************************************************/
 
// GPS Communicates with processor via Hardware serial 
#define GPS_USES_HW_SERIAL              // Comment out if ATMEGA328p communicates with GPS via Software Serial

// Debug serial uses a software serial port
#define DEBUG_USES_SW_SERIAL            // Comment out if Serial Debug Monitor uses Hardware Serial 

// Processor talks to Si5351a using software I2C
#define SI5351A_USES_SOFTWARE_I2C       // Comment out if  ATMEGA328p communicates with the Si5351a via Hardware I2C

#define SI5351_SELF_CALIBRATION_SUPPORTED  true // set to false if No Self calibration. It requires an unused Si5351 CLK output fed back to D5 

// Self Calibration uses External Interrup on PIN D2 or D3 for GPS PPS signal.
// Comment this out if using PinChangeInterrupt on any other PIN 
// This must be defined if the GPS PPS PIN is connected to D2 or D3, otherwise commented out
//#define GPS_PPS_ON_D2_OR_D3        //GPS PPS connects to D2 or D3 and thus can use an External Interrupt othwerwise 

// External Temperature Sensor. If one of the following two are DEFINED this sensor data will be used for temperature telemetry
// otherwise we default to using the internal temperature sensor in the Atmega328p.
/// 
//#define DS1820_TEMP_SENSOR_PRESENT  // Dallas Semiconductor DS18020 external One-wire sensor is present
//#define TMP36_TEMP_SENSOR_PRESENT   // TMP36 - Placeholder for now, not yet supported. 

#if defined (DS1820_TEMP_SENSOR_PRESENT)
#define ONE_WIRE_BUS A0          // Dallas Temperature One-Wire Sensor change this to match PIN usage
#endif

#define VCC_SAMPLING_SUPPORTED false  	// Board does not have the capabilty to sample VCC on Vpwerbus using VpwerDivider as a multiplying factor
#define Vpwerbus     A3          		// ADC input for Vpwrbus for measuring battery voltage - unused on Stella9.1
#define VpwerDivider 1.3333      		// Multiplying factor for ADC voltage divider (5.7 for K1FM V1.3) - unused on Stella9.1
  
// Comment these out if not using an LED to indicate WSPR TX or GPS Time Synch
//#define TX_LED_PRESENT           
//#define SYNC_LED_PRESENT 
/*****************************************************************************/

/*****************************************************************************************
* Atmega328p processor Pin Configurations - change these to match your specific hardware *
*****************************************************************************************/

// PIN definitions for Si5351a software I2C communication. 
// Ignore if using Hardware I2C with Wire Library to communicate with the Si5351a
// These are assuming Hardware Pin assignments compatible with the QRP Labs U3S & U3S-clones
#define SCL_PIN 1  //PB1
#define SCL_PORT PORTB
#define SDA_PIN 2 //PD2
#define SDA_PORT PORTD

// The following two defines select the Arduino pins used for software Serial communications.
// The assumption is that if SW serial is used for communicating with the GPS, that hardware
// serial is used for the debug monitor, or vise versa. One of the two must use hardware serial.
// Note that the choice of PINS will impact the setup of the PinChange Interrupts for NeoSWSerial
// See OrionSerialMonitor.cpp
#define SOFT_SERIAL_RX_PIN        12            // MISO of six pin ICSP header on DL6OW boards
#define SOFT_SERIAL_TX_PIN        11            // MOSI of six pin ICSP header on DL6OW board


// The following defines designate which pins are used for TX and Time Synch if 
// TX_LED_PRESENT and SYNC_LED_PRESENT are defined. They are ignored otherwise.
#define TX_LED_PIN              4             // TX LED on D4.
#define SYNC_LED_PIN            7             // LED on PIN D7 indicates GPS time synchronization. 
// Note the most Arduino Boards have a built-in LED that can be used for either of the above purposes referred to as LED_BUILTIN

#define ANALOG_PIN_FOR_RNG_SEED  A1              // Pin used to generate seed for Random number generator - must be a free analog pin (unused) 
#define Vpwerbus     A3          // ADC input for Vpwrbus for measuring battery voltage
#define CAL_FREQ_IN_PIN 5        // This must be D5 as it is the external clock iput for Timer1 when it is used as a counter. 
                                 // Otherwise Calibration is not supported for your board.
                                  
//#define GPS_PPS_PIN 3            // This must be either 2 or 3 (i.e. D2 or D3) to use external interrupts, otherwise you must use a PinChangeInterrupt for PPS 
#define GPS_PPS_PIN A5            // DL6OW STELLA boards and other U3S Clones use A5/ADC5 (physical pin #28) for PPS. This uses PCINT13   


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
*  in OrionSi5351.h is you need a crystal load capacitance other that 8 pf.
************************************************************************************************************************/
#define SI5351A_CLK_FREQ_CORRECTION   -8213  // Correction value for Si5351a on DL6OW Stella 9.1 prototype

#define SI5351BX_XTALPF   3               // 1:6pf  2:8pf  3:10pf -  assuming 10 pF, otherwise change

// If using 27mhz crystal, set XTAL=27000000, MSA=33.  Then vco=891mhz
#define SI5351BX_XTAL 2500000000ULL      // Crystal freq in centi Hz 
#define SI5351BX_MSA  35                // VCOA is at 25mhz*35 = 875mhz
                                               
#define GPS_SERIAL_BAUD         9600          // Baudrate for the GPS Serial port

#define MONITOR_SERIAL_BAUD     9600          // Baudrate for Orion Serial Monitor      


/***************************************************************************
*   Parameters dependant on Processor CPU Speed - Assumption is 8Mhz Clock *
***************************************************************************/
// The following value WSPR_CTC is used for Timer1 that generates a 1.46 Hz interrupt.

// THE CURRENT VALUE for WSPR_CTC ASSUMES AN 8 MHZ PROCESSOR CLOCK !

// If you are using most Arduinos (i.e Nano, UNO etc) they use a 16 Mhz clock and the value 10672 must be substituted.
// ie. #define WSPR_CTC                10672       // CTC value for WSPR on Arduino using 16 Mhz clock (i.e. Nano, Uno etc)
// The formula to calculate WSPR_CTC is: 1.4648 = CPU_CLOCK_SPEED_HZ / (PRESCALE_VALUE) x (WSPR_CTC + 1)
#define WSPR_CTC                5336               // CTC value for WSPR on Arduino using an 8 Mhz clock (i.e. Arduino Pro Mini 3.3v 8 Mhz)

#define SI5351_CAL_TARGET_FREQ  320000000ULL; //This is calculated as CPU_CLOCK_SPEED_HZ / 2.5 expressed in hundredths of Hz. Assumes 8 Mhz clk.

#endif

# OrionWspr


Current version is: v1.0 (released). 
 

Current compile stats are:

Sketch uses 257008 bytes or 81% (previously 86%) of program storage space. Maximum is 30720 bytes. 
Global variables use 1260 bytes or 61% (previously 63%) of dynamic memory, leaving 788 bytes for local variables. 
Maximum is 2048 bytes.

For this sketch to work properly you will need to calibrate the Si5351a Clock to determine the initial CORRECTION value
and hard-code that in the OrionBoardConfig.h file. See https://github.com/ve3wmb/Artemis 

Notes on Software Versioning:

#define ORION_FW_VERSION "v0.08a" // in OrionSerialMonitor.h update with each submission to github

Whole numbers are for released versions. (i.e. 1.0, 2.0 etc.)
Numbers to the right of the decimal are allocated consecutively, one per GITHUB submission.(i.e. 0.01, 0.02 etc)
a=alpha b=beta, r=release


Changelog :

v1.0 - Official Release version. Update to documentation and code versioning to reflect declaration of Release 1. 

v0.30b - Fix for Issue#6 - GPS LOS during Calibration sometimes results in Orion getting stuck in TX.
 Minor change to OrionManual to correct the section on configuring NEOGPS Library. Up-versioned doc to v0.4 draft. 

v0.29b - Miscellaneous changes in preperation for declaring Release 1 load. 

1) Modification to setup() to ensure that the WATCHDOG timer is always disabled on startup (as a precaution).
 
2) OrionXConfig.h - change DEBUG_MODE to DEBUG_LOG_INITIAL add TX_LOG_INITIAL, these are to set the startup values for the booleans
controlling debug and transmit log output. 

3) Reorganize BoardConfig files to match the order of parameters as described in the Orion Documentation.
 
4) Force negative altitude readings to be zero to ensure that this doesn't affect Telemetry encoding. 

5) OrionBoardConfig.h, define new parameters DELAY_STARTUP_ON_OP_VOLTAGE (true/false) and SHUTDOWN_ON_LOW_VOLTAGE (true/false) to control
STARTUP VOLTAGE and SHUDOWN VOLTAGE FEATURES.

6)FREQUENCY DIVERSITY feature (aka QRM Avoidance) is now initially switched off at startup and only enabled once the calibration target frequency is 
reached during CALIBRATION. Is also disabled on Calibration fail. 

7) New documentation folder now includes Orion Documentation (PDF) and Orion State Machine diagram (PDF). Source files for both are in new 
subtending documents_source folder. 

v0.28b - Fix for GITHUB issue number 7 - Orion System Date Anomaly on GPS LOS. 
Re-defines the criteria for a valid GPS time fix from (fix.valid.time == true) to (fix.valid.status == true) && (fix.status > STATUS_TIME_ONLY ).

v0.27b - Up-versioned to reflect change of load status to Beta. 

v0.26a - Minor changes to remove extraneous code and variables, fix and add additional code comments. Introduce an
info() log to replace some instances of swerr() that should still be logged, but are not actually software errors().

v0.25a - Changes to use LowPower.h library to enable processor power management (PowerDown) while in WAIT_OP_VOLTAGE_ST.
Added #define DEBUG_MODE true/false to OrionXConfig to set the variable  g_debug_on_off = DEBUG_MODE to more easily
control the switching of DEBUG off and on at startup (it can still be switched off via the serial monitor.) 

v0.24a - Code added to cover GPS LOS scenaro #4 where calibration_guard_tmr (INITIAL_CALIBRATION_GUARD_TMO_MS) expires 
before g_chrono_GPS_LOS (GPS_LOS_GUARD_TMO_MS). Also added g_chrono_GPS_LOS.stop() to setup() as default for Chrono
is to start the timer in the constructor, which was causing unexpected behaviour. Added new event STARTUP_CALIBRATION_FAIL_EV
to state machine. This triggers QRSS Beaconing on start-up calibration failure. 

v0.23a - Changes to fallback QRSS Beacon. Switched from DFCW10 to FSKCW10 mode. Minor Modes to improve FSKCW signal. Change to transmit log frequency 
to add 2 Hz to reflect TX frequency being between TONE1 and TONE2. 

v0.22a - Utilize self-calibration in the detection and Handling of GPS LOS and AOS. Implementation of QRSS_TX_ST to support QRSS TX as a fallback mode in
the event of a long term (> 30 minute) GPS LOS. Minor change to si5351bx_setfreq() to add a new boolean parameter to control
whether specified clock is enabled or not after the frequency change (needed for QRSS TX). 

v0.21a - Yet another fix for sporadic TELEMETRY_TIME events. This closes the window for multiple generation of these events by introducing a 2 second delay()
between setting the time and generating the TELEMETRY_TIME_EV so there is no way the scheduler can rerun with exactly the same second. 

v0.20a - Implement operating and shutdown voltage handling (adds #defines for OPERATING_VOLTAGE_Vx10, SHUTDOWN_VOLTAGE_Vx10
OPERATING_VOLTAGE_GUARD_TMO_MS to OrionXConfig.h and VCC_SAMPLING_SUPPORTED true/false to OrionBoardConfig.h). 
For boards where VCC_SAMPLING_SUPPORTED is false, attempts to sample Vcc always return the value of OPERATING_VOLTAGE_Vx10.

v0.19a - Fix for sporadic lost time events.

v0.18a - Phase III of Telemetry. Minor Telemetry code cleanup and changes to allow use of TM36 temperature sensor. 

v0.17a - Phase two of Telemetry. This implements the VE3GTC KISS Telemetry scheme, encoding data into the WSPR message Pwr/dBm field.
It includes Orion State Machine and scheduler changes to enable the transmission of a secondary WSPR message carrying telemetry at 
hh:02, hh:12, hh:22, hh:32, hh:42 and hh:52. 

v0.16a - Phase one of Telemetry. This update introduces some Telemetry Data structures and includes the encoding of the 5th and 6th 
Maidenhead Grid Square characters into the PWR/dBm field of the primary WSPR message using a scheme developed by VE3GTC. 

v0.15a - Minor code changes to improve WSPR TX timing. Modified the orion_scheduler() function to generate the PRIMARY_WSPR_TX_TIME_EV
at (Second == 1) rather than (Second = 0) to reduce DT values. Addressed Issue #1 raised against Orion in GITHUB, by
synchronizing the 1.46 second WSPR TX interrupt to the start of the WSPR transmission, by clearing TCNT1 and doing a Prescaler
reset to zero the count. 

v0.14a - Orion Self calibration with Si5351a frequency correction using  a Huff-n-Puff Algorithm. Supports both External Interrupts
and PinChangeInterrupts for GPS PPS signal and requires unused clock output from Si5351a to be wired to ATMEGA328p PIN D5, which
is the external clock input for Timer1. Timer1 is used as a counter to sample a 3.2 Mhz calibration signal from the Si5351a.
The GPS PPS signal is used as a time-base to implement a simple frequency counter. A ten second sample is used to measure the
frequency, accurate to 1/10 Hz and then a correction factor is applied to the Si5351a and the output frequency regenerated. This 
is repeated 24 times, yielding a self-calibration cycle of approximately 4 minutes, prior to each transmission. The initial cycle
after powerup uses a 1 Hz correction factor and subsequent calibrations use 0.1Hz steps to achieve approximately 1 Hz accuracy
for the beacon at 14 Mhz. 

v0.13a - support K1FM v1.2 board and GPS and Si5351 power disable feature.

v0.12a - Introduced board configuration. Individual .h files are stored in board_configuration folder and are coped and renamed  
OrionBoardConfig.h to provide #defines necessary to configure PINs and functionality include HW vs SW Serial (to GPS and debug 
monitor) as well as HW and SW I2C communication with the Si5351a. See OrionBoardConfig.h for more info. 

v0.11a - replaced TinyGps library with NeoGps to resolve compatibility issues with uBlox NEO-M8N due to TinyGpses
lack of support for GNSSes other the GPS. 

v0.10a - added "QRM avoidance" feature which applies a random frequency offset in the range of 0 to 180 Hz to the 
base (bottom + 10 hz) transmit frequency for that transmit cycle. Also added the capability disable this feature via 
the Orion Serial Monitor for testing purposes. Minor changes made to add some preliminary data structures to support
further telemetry data. 

v0.09a - added two additonal states WAIT_TELEMETRY_ST and TELEMETRY_ST and associated events and actions 
(see State Machine Diagram for details). Added support for the calculation of 6 character GRID
Square from GPS Lat/Long and transmission of the first 4 characters of that calculated Grid Square in the Primary 
WSPR message. 

v0.08a - files OrionSerialMonitor.h and OrionSerialMonitor.cpp 
to implement a primative serial shell and implement software error logging and state machine tracing capabilities.
Added software versioning. 
 
v0.07a - Added two additional files OrionStateMachine.cpp and OrionStateMachine.h that 
implement a finate state machine control mechanism for the Orion WSPR Beacon. Added a folder called 
OrionFSMDocs which contains a .graphml source file (yEd Graphical Editor) and a .jpg export which documents the current
implementation of the Orion finite state machine. These are intended to be living documents and will be updated
each time that there are changes made to the state machine. 


# OrionWspr


Consider this to be Alpha Software. Current version is: v0.14a

Current Limitations :
- currently only encodes and sends primary WSPR Message, not secondary message containing Telemetry info.
- beacons on a single band 

Current compile stats are:

Sketch uses 20574 bytes or 66% (previously 59%) of program storage space. Maximum is 30720 bytes. 
Global variables use 994 bytes or 48% (previously 46%) of dynamic memory, leaving 1054 bytes for local variables. 
Maximum is 2048 bytes.

For this sketch to work properly you will need to calibrate the Si5351a Clock to determine the initial CORRECTION value
and hard-code that in the BoardConfig.h file. 

Notes on Software Versioning:

#define ORION_FW_VERSION "v0.08a" // in OrionSerialMonitor.h update with each submission to github

Whole numbers are for released versions. (i.e. 1.0, 2.0 etc.)
Numbers to the right of the decimal are allocated consecutively, one per GITHUB submission.(i.e. 0.01, 0.02 etc)
a=alpha b=beta, r=release


Changelog : 

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


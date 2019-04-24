# OrionWspr


Consider this to be Alpha Software. Current version is: v0.08a

In an effort to make this code compatible with U3S-Clone boards (N2NXZ, DL6OW) it now uses the SoftWire.h
library rather than Wire.h to implement software I2C communication with the Si5351a. 
Current Limitations :
- hardcoded 4 character grid square (will be replaced by Grid Square derived from GPS Coordinates)
- currently only encodes and sends primary WSPR Message, not secondary message containing Telemetry info.
- beacons on a single band/frequency 
 
Changes in this current version were : add two additional files OrionSerialMonitor.h and OrionSerialMonitor.cpp 
to implement a primative serial shell and implement software error logging and state machine tracing capabilities.

Current compile stats are:

Sketch uses 14928 bytes or 45% (previousyl 37%) of program storage space. Maximum is 30720 bytes. 
Global variables use 937 bytes or 45% (previously 39%) of dynamic memory, leaving 1111 bytes for local variables. 
Maximum is 2048 bytes.

For this sketch to work properly you will need to calibrate the Si5351a Clock to determine the CORRECTION value.

Notes on Software Versioning:

#define ORION_FW_VERSION "v0.08a" // in OrionSerialMonitor.h update with each submission to github

Whole numbers are for released versions. (i.e. 1.0, 2.0 etc.)
Numbers to the right of the decimal are allocated consecutively, one per GITHUB submission.(i.e. 0.01, 0.02 etc)
a=alpha b=beta, r=release


Changelog : 

v0.08a - files OrionSerialMonitor.h and OrionSerialMonitor.cpp 
to implement a primative serial shell and implement software error logging and state machine tracing capabilities.
Added software versioning. 
 
v0.07a - Added two additional files OrionStateMachine.cpp and OrionStateMachine.h that 
implement a finate state machine control mechanism for the Orion WSPR Beacon. Added a folder called 
OrionFSMDocs which contains a .graphml source file (yEd Graphical Editor) and a .jpg export which documents the current
implementation of the Orion finite state machine. These are intended to be living documents and will be updated
each time that there are changes made to the state machine. 


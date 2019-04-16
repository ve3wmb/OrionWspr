# OrionWspr


Consider this to be Alpha Software.
 
Changes in this current version were to introduce two additional files OrionStateMachine.cpp and OrionStateMachine.h that 
implement a finate state machine control mechanism for the Orion WSPR Beacon. In addition I have added a folder called 
OrionFSMDocs which contains a .graphml source file (yEd Graphical Editor) and a .jpg export which documents the current
implementation of the Orion finite state machine. These are intended to be living documents and will be updated
each time that there are changes made to the state machine. 

Current compile stats are:

Sketch uses 11604 bytes (37%) of program storage space. Maximum is 30720 bytes. 
Global variables use 810 bytes (39%) of dynamic memory, leaving 1238 bytes for local variables. Maximum is 2048 bytes.


In an effort to make this code compatible with U3S-Clone boards (N2NXZ, DL6OW) it now uses the SoftWire.h
library rather than Wire.h to implement software I2C communication with the Si5351a. 

Current Limitations :
- hardcoded 4 character grid square (will be replaced by Grid Square derived from GPS Coordinates)
- currently only encodes and sends primary WSPR Message, not secondary message containing Telemetry info.
- beacons on a single band/frequency 

For this sketch to work properly you will need to calibrate the Si5351a Clock to determine the CORRECTION value.


# OrionWspr


Consider this to be Alpha Software.
 
Changes in this current version were to replace the Etherkit Si5351 Library with modified code to control the Si5351a that 
was originally written by KE7ER. It was modified by VE3WMB to allow for the sub-Hz precision needed for WSPR and now shows up in 
its own tab called OrionSi5351.cpp. Making this change has also resulted in significant recovery of code space and reduced 
the amount of memory used for globals.

Current stats are:

Sketch uses 11324 bytes (36%) of program storage space. Maximum is 30720 bytes. 
Global variables use 802 bytes (39%) of dynamic memory, leaving 1246 bytes for local variables. Maximum is 2048 bytes.


In an effort to make this code compatible with U3S-Clone boards (N2NXZ, DL6OW) it now uses the SoftWire.h
library rather than Wire.h to implement software I2C communication with the Si5351a. 

Current Limitations :
- hardcoded 4 character grid square (will be replaced by Grid Square derived from GPS Coordinates)
- currently only encodes and sends primary WSPR Message, not secondary message containing Telemetry info.
- beacons on a single band/frequency 

For this sktch to work properly you will need to calibrate the Si5351a Clock to determine the CORRECTION value.
This can be done with si5351_calibration.ino which can be found in the examples directory of the 
Eherkit Si5351 Library (https://github.com/etherkit/Si5351Arduino). If you are using a TCXO in place of a 
crystal for the Si5351a you might be able to get away without the calibration step but it is neccessary with a 
crystal otherwise your signal is likly to fall outside of the 200hz WSPR "Window" and it won't be decoded".

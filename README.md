# OrionWspr


Consider this to be Alpha Software.
 
This version is just a cleaned up version of the proof-of-concept protoype WSPR Beacon developed for
project Aries and as such still needs some work.

Current Limitations :
- hardcoded 4 character grid square (will be replaced by Grid Square derived from GPS Coordinated)
- currently only encodes and sends primary WSPR Message, not secondary message containing Telemetry info.
- beacons on a single band/frequency 

For this sktch to work properly you will need to calibrate the Si5351a Clock to determine the CORRECTION value.
This can be done with si5351_calibration.ino which can be found in the examples directory of the 
Eherkit Si5351 Library (https://github.com/etherkit/Si5351Arduino). If you are using a TCXO in place of a 
crystal for the Si5351a you might be able to get away without the calibration step but it is neccessary with a 
crystal otherwise your signal is likly to fall outside of the 200hz WSPR "Window" and it won't be decoded".

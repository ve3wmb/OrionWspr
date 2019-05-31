README for OrionBoardConfig.h

Use one of the files in this directory to replace OrionBoardConfig.h
in the main code directory (one level up) to configure Orion for a specific board.
Then recompile the code. 

This allows the Orion WSPR beacon to run a number of different boards including U3S clones
as well as K1FM PicoB boards. Be sure to make the necessary changes to #defines to describe
your board configuration (better documentation to come later). 


Note that the value for SI5351A_CLK_FREQ_CORRECTION is derived by doing a manual calibration of the
Si5351a by loading the OrionSi5351aCalibration sketch (https://github.com/ve3wmb/OrionSi5351aCalibration)
if the board uses software I2C and software serial for debug. Use the original Etherkit Si5351Arduino calibration
sketch found in the examples folder for the Etherkit Si5351 Library for boards using hardware I2C and hardware serial.
  (https://github.com/etherkit/Si5351Arduino/tree/master/examples/si5351_calibration).

This also requires that the Etherkit library be installed (https://github.com/etherkit/Si5351Arduino). 

If you fail to do an initial calibration you will likely be far enough off frequency with your
WSPR transmissions that you will fall outside of the 200Hz wide WSPR transmission window and no one will
copy your signal. 

Manual calibration should be a temporary thing until the Orion Self-Calibration feature is completed.

VE3WMB 
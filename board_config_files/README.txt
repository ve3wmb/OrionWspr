README for OrionBoardConfig.h

Use one of the files in this directory to replace OrionBoardConfig.h
in the main code directory (one level up) to configure Orion for a specific board.
Then recompile the code. 

This allows the Orion WSPR beacon to run a number of different boards including U3S clones
as well as K1FM PicoB boards. Be sure to make the necessary changes to #defines to describe
your board configuration.

Note that the value for SI5351A_CLK_FREQ_CORRECTION is derived from self-calibration.
On a new board start with a value of 0. (i.e. #define SI5351A_CLK_FREQ_CORRECTION 0).

Once the beacon has been running for a while you can note when calibration is at or very close to
the 3.2 Mhz test signal value. Note the correction factor (could be negative) and substitute that for 
SI5351A_CLK_FREQ_CORRECTION. Now recompile and reload. This value might change a bit over time 
but it will allow the Si5351a to come up fairly close to the target frequency and self-calibration
won't have to work so hard and long to get it on frequency. 

If you fail to preperly set SI5351A_CLK_FREQ_CORRECTION you will likely be far enough off frequency with your
WSPR transmissions that you will fall outside of the 200Hz wide WSPR transmission window on startup and no one will
copy your signal until a number of self-calibration cycles adjusts the frequency.  


VE3WMB 
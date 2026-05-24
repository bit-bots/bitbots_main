# Status constants
#  GOOD               The heading is reliable and not affected by iron interferences.
#  OK                 The heading is reliable, but affected by slight iron interferences.
#  NOT_GOOD           The heading is not reliable because affected by strong iron interferences.
#  NOT_CALIBRATED     The magnetometer has not been calibrated.
#  MAG_NOT_AVAILABLE  The magnetometer sensor is not available.

uint8 GOOD=0 
uint8 OK = 1
uint8 NOT_GOOD = 2
uint8 NOT_CALIBRATED = 3
uint8 MAG_NOT_AVAILABLE = 4

# Status
uint8 status

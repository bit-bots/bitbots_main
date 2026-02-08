This package provides an hardware interface for the Dynamixel servos in ros control.

For more information have a look in the documentation on doku.bit-bots.de or in the doc folder.

# Important
You will have to use our forked versions of dynamxiel_workbench and DynamixelSDK which provide functions for reading multiple register values in a single syncRead. They can be found here:

https://github.com/bit-bots/DynamixelSDK

https://github.com/bit-bots/dynamixel-workbench


Set your linux usb bus latency timer to 1 or 0 !!!!
see comment here

https://github.com/bit-bots/DynamixelSDK/blob/master/c%2B%2B/src/dynamixel_sdk/port_handler_linux.cpp

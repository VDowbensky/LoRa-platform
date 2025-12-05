================================================================================
                                Sample Usage Instructions
================================================================================
Version History 
Date           Version   Person         IAR     MDK   Description
2019-06-24       0.1      lsq           8.30    5.26  first version
================================================================================
Function Description
================================================================================
Note:
This example mainly demonstrates the function of USB simulated serial 
communication. The serial communication parameters (port, baud rate, data bits, 
parity bits, stop bits) are set by the host computer. After connection, data is 
sent to the MCU through the host computer's serial port debugging assistant. 
When the MCU receives the last data as 0x0d, it will send back a string to the 
host computer.

================================================================================
Test environment
================================================================================
Test board:
---------------------
HC32LF07X-EVB-V11

Auxiliary tools:
---------------------

Auxiliary software:
---------------------

================================================================================
Usage steps
================================================================================
1) Open the project and recompile;
2) Enable the IDE's download and debugging functions;
3) Run;
4) Send data to the MCU via the host computer. The sent data ends with 0x0d. 
When the MCU receives the data, it will send the following string:
    "this is a sample about usb cdc"

================================================================================
Notice

================================================================================
 

================================================================================

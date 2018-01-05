This software lets you use an arduino as an interface between a Linux system and some radio.
In the example code (arduino-kiss.ino) I use the arduino-cc1101-libray to use a CC1101 radio to transmit AX.25 over.
(https://github.com/veonik/arduino-cc1101)

Required steps:
add a line to /etc/ax25/axports:

ax0	CALLSGN	9600	64	1	AX.25 over CC1101

ax0 is the interface name, CALLSGN is an AX.25 call-sign, 9600 is the speed to use when talking to the arduino and 64 is the maximum packet size that the radio can handle.

Then invoke:
kissattach /dev/ttyUSB0 ax0

This will connect /dev/ttyUSB0 (the serial device to which the Arduino is connected) to the network interface called ax0 (see axports).
In Debian the kissattach software can be found in the ax25-tools package.

cudos to:
-- Daniel Berenguer
contact@panstamp.com
-- Folkert van Heusden
folkert@vanheusden.com
-- Tyler Sommer 
contact@tylersommer.pro

port to cc1101 by:
vogt31337
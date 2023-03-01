# State
USB enumeration is not working properly. 
## What works so far:
* Reading/Writing packets from the USB bus
* Allocation of USB endpoints

## What does not work
* Currently the first packet the MCU receives is the USB GET_DESCRIPTOR packet instead of SET_ADDRESS. Still got to figure out how the hell this is happening. Maybe a quirk linux usbcore driver.

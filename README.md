# esphome-abl-emh1
Esphome component for communication with ABL Wallbox eMH1

This is a very basic EspHome component for use with the
ABL eMH1 wallbox.

It reads the current on 3 phases and allows you to set the max current.
There is also a switch to enable/disable the charger.

Some more output (like the serial number) is available in hidden entities.

### My hardware
- ESP-wroom 32 (esp32dev)
- ESP GPIO pin 5 is flow-control 
- SN75176 to convert serial to RS485
- 5V Din-rail power supply
- Esp Ground connected to the GND pin from connector X10 on the ABL eMH1 circuit board. This helps avoiding noise on the RS485.
- RJ45 plug: pin 1 and 2 connected to SN75176, no other pins connected


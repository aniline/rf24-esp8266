Port of https://github.com/maniacbug/RF24 to run on esp8266 module.

## Some gotchas ##

* Only one instance, right now.
* Many functions with different signature have been renamed (like, rf24_write_noack, and bunch of functions with ```_v``` suffix).
* Did not bother about the interference and how the radios (wifi and rf24) would mess each other up.
* I've tried it on ESP-12 type board - It sent stuff, it read stuff and did wifi stuff (not simultanously).
* There is nothing useful being done using wifi, just running in AP mode set to values in user_config.h

## Building ##

The provided files build into an image that can do rudimentary listen and send.
Setup the variables as required by https://github.com/esp8266/source-code-examples . 
This includes stuff like:

    XTENSA_TOOLS_ROOT=/opt/Espressif/crosstool-NG/builds/xtensa-lx106-elf/bin
    SDK_BASE=/opt/Espressif/ESP8266_SDK
    FW_TOOL=${XTENSA_TOOLS_ROOT}/esptool
    ESPTOOL=${XTENSA_TOOLS_ROOT}/esptool.py
    ESPPORT=/dev/ttyUSB0

Build checkdirs because the Makefile symlinks some .c files from SDK in the first step.

	$ make checkdirs
	$ make

Flash using

    $ make flash
    
## Wiring it up ##

     | ESP-12  | RF24 |
     |---------+------|
     | HSPICLK | SCLK |
     | HSPIQ   | MISO |
     | HSPID   | MOSI |
     | GPIO4   | CE   |
     | GPIO5   | CSN  |

The CE, CSN can be swapped and the code changed appropriately.

## Arduino IDE ##

This port is **not** meant for Arduino IDE. You could try out the mildly tweaked version the original maniacbug's version here: https://github.com/aniline/RF24 

## Thanks ##

* https://github.com/maniacbug/RF24
* https://github.com/esp8266/esp8266-wiki/wiki (All contributions to stuff mentioned there)
* https://github.com/vppillai/esp8266-Environment

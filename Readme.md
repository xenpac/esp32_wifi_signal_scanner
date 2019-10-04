# ESP32 Wifi Signal Scanner with LED blinking

## Overview

This application can be used to find Wifi devices in your home.  
It uses the onboard LED of the esp32-cam  board to show signal strength.  
The Led is on IO-Pin 4, change that if your boards LED is different.  
The blink frequency is 5 seconds if no signal detected.
It then increases up to 25Hz, the nearer you get to a wifi device.

As wifi activity is usually quiet with occasional packets being transfered, this detector works best in heavy transfers.  

It works in monitor mode and scans channels 1 to 13.  


## Project:

It is assumed you are familiar with setting up and using the esp32 environment.

You need ESP-IDF  from: https://github.com/espressif/esp-idf
The crosscompiler from: https://dl.espressif.com/dl/xtensa-esp32-elf-linux64-1.22.0-80-g6c4433a-5.2.0.tar.gz

I compiled the project using Ubuntu with crosscompiler and make.

Goto cam_server directory and edit the sdkconfig file to update your wifi AP settings.

Then type "make" to compile the project.

It also possible to use the Arduino IDE.




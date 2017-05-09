# HCS-Bridge ESP32 Indoor Location Beacon Sensor Node

This is the source code for the firmware of a indoor location beacon sensor node to be deployed on ESP32 Devkit-C hardware.

It is based upon the official ESP-IDF framework and xtensa toolchain.

## Setup and Building

In order to configure, build and flash this firmware please make sure that the ESP_IDF path is set as well as the xtensa
toolchain is in your path (as per howto: https://github.com/espressif/esp-idf/blob/master/docs/linux-setup.rst).

With this you should be able to run

    make menuconfig
    
This will allow you to configure the WiFi SSID and Password (as this is currently hardcoded into the firmware).

After the successful configuration you can compile the firmware by running

    make
    
and finally flash it with:

    make flash
    
    
Make sure that you configured the device for the serial-device flashing correctly on the above *menuconfig* step.
Also you can do some adjustments (for example if your mqtt topics are longer than certain lengths) in the component-config
section under the MQTT menu entry.

## Internal flow

* test
* 
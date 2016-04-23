# Weather Station

This is a simple weather station project that measures:

 - Temperature
 - Humidity
 - Relative Pressure
 - UV / Light Level

It's based on the 2x ATmega328 with the Arduino Uno Bootloader. 

This is simply a fun hobby project.

**Transmit.ino**

The transmit unit consists of a custom board with a set of sensors and a Transmitter module. The code in this file simply collects the sensor data into a single string and transmits it.  The code also sleeps the Arduino for 8 seconds between transmits to conserve power. 

 1. [BOSCH BMP180 Digital Pressure Sensor](https://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/Pressure/BMP180.pdf)
 2. [HIH-4030/31 Humidity Sensor](https://www.sparkfun.com/datasheets/Sensors/Weather/SEN-09569-HIH-4030-datasheet.pdf)
 3. A mini photocell
 4. [Wenshing TWS-BS RF Module](https://dlnmh9ip6v2uc.cloudfront.net/datasheets/Wireless/General/TWS-BS-6_315MHz_ASK_RF_Transmitter_Module_Data_Sheet.pdf)
 5. [UV / Light Sensor](https://cdn-shop.adafruit.com/datasheets/Si1145-46-47.pdf)

**Receive.ino**

The Receiver circuit uses areceive module and outputs onto a 16x2 LCD screen using I2c.

This uses the [Wenshing RWS-374 transmitter](https://dlnmh9ip6v2uc.cloudfront.net/datasheets/Wireless/General/RWS-374-3_315MHz_ASK_RF_Receiver_Module_Data_Sheet.pdf)  along with a run of the mill [16x2 LCD](https://proto-pic.co.uk/rgb-backlight-positive-lcd-16x2-extras/)

# Notes

The following libraries are used:
   - [Liquid Crystal](https://github.com/adafruit/LiquidCrystal)
   - [VirtualWire (RF-Links)](https://github.com/sparkfun/RF_Links)
   - [BMP-180](https://github.com/sparkfun/BMP180_Breakout)
   - [Adafruit_SI1145_Library](https://github.com/adafruit/Adafruit_SI1145_Library/)

# License
This code can be used under the terms of the 3-Clause BSD license included in this repository.
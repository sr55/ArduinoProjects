# Weather Station

This is a simple weather station project that measures:

 - Temperature
 - Humidity
 - Relative Pressure
 - Light Level

It's based on the 2x ATmega328 with the Arduino Uno Bootloader. 

It was mainly created for fun and to learn something new.

**Transmit.ino**

The transmit unit consists of a custom board with 3 sensors and a Transmitter module. The code in this file simply collects the sensor data into a single string and transmits it.  The code also sleeps the Arduino for 8 seconds between transmits to conserve power. 

 1. [BOSCH BMP180 Digital Pressure Sensor](https://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/Pressure/BMP180.pdf)
 2. [HIH-4030/31 Humidity Sensor](https://www.sparkfun.com/datasheets/Sensors/Weather/SEN-09569-HIH-4030-datasheet.pdf)
 3. A mini photocell
 4. [Wenshing TWS-BS RF Module](https://dlnmh9ip6v2uc.cloudfront.net/datasheets/Wireless/General/TWS-BS-6_315MHz_ASK_RF_Transmitter_Module_Data_Sheet.pdf)

**Receive.ino**

The Receiver circuit uses areceive module and outputs onto a 16x2 LCD screen using I2c.

This uses the [Wenshing RWS-374 transmitter](https://dlnmh9ip6v2uc.cloudfront.net/datasheets/Wireless/General/RWS-374-3_315MHz_ASK_RF_Receiver_Module_Data_Sheet.pdf)  along with a run of the mill [16x2 LCD](https://proto-pic.co.uk/rgb-backlight-positive-lcd-16x2-extras/)

# Notes

 1. Please be aware that this code is a little "hacked" together and mostly just 
written for some fun.  Please feel free to use, extend, improve and submit pull requests to me. 

 2. The following libraries are used:
	 - [Liquid Crystal](https://github.com/adafruit/LiquidCrystal)
	 - [VirtualWire (RF-Links)](https://github.com/sparkfun/RF_Links)
	 - [BMP-180](https://github.com/sparkfun/BMP180_Breakout)

 3. I don't have experience, or own electronic cad software so am not currently able to provide circuit diagrams.  (Maybe something I can learn in the future.) For the most part, it's simply a case of looking up tutorials on each of the components linked above.

# License
This code can be used under the terms of the 3-Clause BSD license included in this repository.
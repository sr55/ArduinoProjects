// File: Transmit.ino
// Version: 2.0
// Copyright: (C) 2017 Scott (sr55)
// Description: This is a sketch for a basic weather station.
//  It supports
//    - Sensors: Temperature, Pressure, Humidity, UV, IR, Light Levels
//    - LiPo Battery Voltage Monitoring.
//    - Storage to SD card
//    - Bluetooth interface
//        - Fetch current sensor data
//        - Download the file from SD card.
//        - Delete the file on the SD card.
//        - Get the current sensor details.

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include <math.h>
#include "Adafruit_SI1145.h"
#include <Wire.h>
#include <LiFuelGauge.h>
#include <SPI.h>
#include <SD.h>
#include "RTClib.h"
#include <SoftwareSerial.h>  

#define aref_voltage 5.00 
#define LED_PIN (13)
#define ALTITUDE 142.0 // Altitude of Sensor

// Pins
int transmit_pin = 8;
int HIH4030_Pin = A3; //analog pin 0
int led_blink = 0; // Slow down the blinking.
const uint8_t chipSelect = 8;
const uint8_t cardDetect = 9;

// Globals
volatile int f_wdt=1;
SFE_BMP180 pressure;
Adafruit_SI1145 uv = Adafruit_SI1145();
RTC_DS1307 rtc;
bool alreadyBegan = false;  // SD.begin() misbehaves if not first call
 
// Power Management
volatile boolean alert = false; // A flag to indicate a generated alert interrupt
void lowPower() { alert = true; } 
LiFuelGauge gauge(MAX17043, 0, lowPower);

// Communications
SoftwareSerial bluetooth(2, 3);// TX-O pin of bluetooth mate, Arduino D2  | RX-I pin of bluetooth mate, Arduino D3

void setup()
{
    // LED Indication
    digitalWrite(7, HIGH);             // Turn on the status LED

    // Setup Serial
    Serial.begin(9600);                  // Debugging only
    Serial.println(F("Initialising..."));
    pinMode(13, OUTPUT);

    // Setup Lipo Montioring (MAX17043)
    gauge.reset();  
    delay(200);  // Waits for the initial measurements to be made  
    gauge.setAlertThreshold(10); // Sets the Alert Threshold to 10% of full capacity
    
    // Setup RTC
    if (! rtc.begin()) {
      Serial.println(F("Couldn't find RTC"));
      while (1);
    }
    if (! rtc.isrunning()) {
      Serial.println(F("RTC is NOT running!"));
    }
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

    // Setup Pressure Sensor (BMP180)
    if (pressure.begin())
      Serial.println(F("BMP180 init success"));
    else
    {
      Serial.println(F("BMP180 init fail\n\n"));
      while(1); // Pause forever.
    }

    // Setup Light Sensor (Si1145)
    if (! uv.begin()) {
      Serial.println(F("Didn't find Si1145"));
      while (1);
    }

    // SD Card
    // Note: To satisfy the AVR SPI gods the SD library takes care of setting
    // SS_PIN as an output. We don't need to.
    pinMode(cardDetect, INPUT);
    initializeCard();

    // Comms
    bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
    bluetooth.print("$");  // Print three times individually
    bluetooth.print("$");
    bluetooth.print("$");  // Enter command mode
    delay(100);  // Short delay, wait for the Mate to send back CMD
    bluetooth.println("U,115K,N");  // Temporarily Change the baudrate to 9600, no parity

    // We are done.
    Serial.println(F("Setup complete."));
    digitalWrite(7, LOW);              // Turn on the status LED
    delay(100);                        // Allow for serial print to complete.
}

void loop()
{ 
  digitalWrite(7, HIGH);
  ProcessBluetoothCommand();

  Serial.println(F("Writing ... "));
  delay(50);
  char fileName[] = "weather.txt"; // SD library only supports up to 8.3 names
  File fd = SD.open(fileName, FILE_WRITE);
  delay(50);
  writeToSD(fd, "{ weather: {");
  writeToSD(fd, "date: \"" + getTime() + "\"," );
  writeToSD(fd, "pressure: \"" + getPressureData() + "\"," );
  writeToSD(fd, "humidity: " + floatToString(getHumidity()) + "," );
  writeToSD(fd, "ir: " + getIR() + "," );
  writeToSD(fd, "visible: " + getVisible() + "," );
  writeToSD(fd, "uv: " + getUvIndex() + "," );
  writeToSD(fd, "power: \"" + getPower() + "\"," );
  writeToSD(fd, "}}, \n");
  fd.close();  
  Serial.println(F("Done ... "));
  digitalWrite(7, LOW);
  
  enterSleep(); //  Re-enter sleep mode. 
}


/*************************************************** 
 *  Bluetooth Support. Send and Receive Commands
 ***************************************************/

void ProcessBluetoothCommand(){

  if (bluetooth.available()){  
    String command = bluetooth.readString();
    Serial.println(command);
    bluetooth.println("Got command: " + command);
    command.trim(); // Remove Any newline chars.
    if (command.equals("D")){
       sendFile();
       SD.remove("Weather.txt"); // Clean the file up. We want to start fresh after download.
    }

    if (command.equals("C")){
       bluetooth.println("date: \"" + getTime() + "\"," );
       bluetooth.println("pressure: \"" + getPressureData() + "\"," );
       bluetooth.println("humidity: " + floatToString(getHumidity()) + "," );
       bluetooth.println("ir: " + getIR() + "," );
       bluetooth.println("visible: " + getVisible() + "," );
       bluetooth.println("uv: " + getUvIndex() + "," );
       bluetooth.println("power: \"" + getPower() + "\"," );
    }
    
    bluetooth.println("-end-");
  }
}

void sendFile(){
  char fileName[] = "weather.txt"; // SD library only supports up to 8.3 names
  File fd = SD.open(fileName, FILE_WRITE);
  delay(50);
  if (fd) {
     while (fd.available()) {
      String line = fd.readStringUntil('\n');
      bluetooth.println(line);
     }
  }
}

/*************************************************** 
 *  Sensor Data
 ***************************************************/
float getHumidity(){
  float degreesCelsius;
  double T;

  // Fetch a temperature reading
  char status = pressure.startTemperature();
  if (status != 0)
  {
    delay(status); // Wait for the measurement to complete:
    status = pressure.getTemperature(T);
    if (status != 0)
    {
      degreesCelsius = T;
    } 
  }  
  
  //caculate relative humidity
  float supplyVolt = 5.0;

  // read the value from the sensor:
  int HIH4030_Value = analogRead(HIH4030_Pin);
  float voltage = HIH4030_Value/1023. * supplyVolt; // convert to voltage value

  // convert the voltage to a relative humidity
  // - the equation is derived from the HIH-4030/31 datasheet
  // - it is not calibrated to your individual sensor
  //  Table 2 of the sheet shows the may deviate from this line
  float sensorRH = 161.0 * voltage / supplyVolt - 25.8;
  float trueRH = sensorRH / (1.0546 - 0.0026 * degreesCelsius); //temperature adjustment 

  return trueRH;
}

String getIR(){
  return  floatToString(uv.readIR());
}

String getVisible(){
  return  floatToString(uv.readVisible());
}

String getUvIndex() {
  float UVindex = uv.readUV();
  // the index is multiplied by 100 so to get the
  // integer index, divide by 100!
  UVindex /= 100.0;  

  return floatToString(UVindex);
}

String getPressureData() {
  char status;
  double T,P,p0,a;
  String result;

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.
    status = pressure.getTemperature(T);
    if (status != 0)
    {     
       result += "t:" + floatToString(T);
      
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.
      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.
        status = pressure.getPressure(P,T);
        if (status != 0)
        {         
          result += " a: " + floatToString(P);
          
          // The pressure sensor returns abolute pressure, which varies with altitude.
          // To remove the effects of altitude, use the sealevel function and your current altitude.
          // This number is commonly used in weather reports.
          // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
          // Result: p0 = sea-level compensated pressure in mb
          p0 = pressure.sealevel(P,ALTITUDE); // we're at 1655 meters (Boulder, CO)

          result += " r: " + floatToString(p0);
        }
        else Serial.println(F("error retrieving pressure measurement\n"));
      }
      else Serial.println(F("error starting pressure measurement\n"));
    }
    else Serial.println(F("error retrieving temperature measurement\n"));
  }
  else Serial.println(F("error starting temperature measurement\n"));
  
  return result;
}

String getPower() {

    double voltage = gauge.getVoltage();
    double percentage = gauge.getSOC();
    String power = "Power: " + doubleToString(percentage) + " %, V: " + doubleToString(voltage) + "V";
   
    if ( alert )
    {
        power = power + " (Low Batt)";
        gauge.clearAlertInterrupt();  // Resets the ALRT pin
        alert = false;
        gauge.sleep();  // Forces the MAX17043 into sleep mode
    }
    
    return power;
}

String getTime () {
    DateTime now = rtc.now();
    uint16_t currentYear = now.year();
    char currentDate[30];
    sprintf( currentDate, "%02hhu-%02hhu-%02hhu", now.year(), now.month(), now.day() );
    char currentTime[30];
    sprintf( currentTime, "%02hhu:%02hhu:%02hhu", now.hour(), now.minute(), now.second() );
    String currDate(currentDate);
    String currTime(currentTime);
    String currentDateTime = currDate + ", " + currTime;

    return currentDateTime;
}

/*************************************************** 
 *  328 Power Management
 ***************************************************/

ISR (WDT_vect) 
{
   wdt_disable();  // disable watchdog
} 

void enterSleep(void)
{
  Serial.println(F("Sleeping"));
  delay(100);
  // disable ADC
  ADCSRA = 0;  

  // clear various "reset" flags
  MCUSR = 0;     
  // allow changes, disable reset
  WDTCSR = bit (WDCE) | bit (WDE);
  // set interrupt mode and an interval 
  WDTCSR = bit (WDIE) | bit (WDP3) | bit (WDP0);  // set WDIE, and 8 seconds delay
  wdt_reset();  // pat the dog
  
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
  noInterrupts ();           // timed sequence follows
  sleep_enable();
 
  // turn off brown-out enable in software
  MCUCR = bit (BODS) | bit (BODSE);
  MCUCR = bit (BODS); 
  interrupts ();             // guarantees next instruction executed
  sleep_cpu ();  
  
  // cancel sleep as a precaution
  sleep_disable();
  Serial.println(F("Waking up..."));
  delay(100);

}

/*************************************************** 
 *  SD Card Management
 ***************************************************/
void initializeCard(void)
{
  Serial.print(F("Initializing SD card..."));

  // Is there even a card?
  if (!digitalRead(cardDetect))
  {
    Serial.println(F("Waiting for card..."));
    while (!digitalRead(cardDetect));
    delay(250); // 'Debounce insertion'
  }

  // Card seems to exist.  begin() returns failure
  // even if it worked if it's not the first call.
  if (!SD.begin(chipSelect) && !alreadyBegan)  // begin uses half-speed...
  {
    Serial.println(F("Initialization failed!"));
    initializeCard(); // Possible infinite retry loop is as valid as anything
  }
  else
  {
    alreadyBegan = true;
  }

  if (SD.exists("Weather.txt"))
      SD.remove("Weather.txt");
      
  Serial.println(F("Initialization done."));
  delay(50);
}

void writeToSD(File fd, String contents)
{
  delay(50);
  if (fd) {
    fd.println(contents);
    Serial.println(contents);
    delay(100);  
  }
}

/*************************************************** 
 *  Helper Methods 
 ***************************************************/

String floatToString(float value) {
    char  outstr[15];
    dtostrf(value, 4, 2, outstr);    
    return outstr;
}
String doubleToString(double value) {
    char  outstr[15];
    dtostrf(value, 4, 2, outstr);    
    return outstr;
}


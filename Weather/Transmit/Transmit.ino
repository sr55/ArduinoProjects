// Receove.ino
// Author: Scott (sr55)
// Copyright (C) 2016 Scott (sr55)

#include <VirtualWire.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include <math.h>
#include "Adafruit_SI1145.h"


#define aref_voltage 5.00 
#define LED_PIN (13)
#define ALTITUDE 142.0 // Altitude of Sensor

// Pins
int transmit_pin = 8;
int HIH4030_Pin = A3; //analog pin 0
int led_blink = 0; // Slow down the blinking.

// Globals
volatile int f_wdt=1;
SFE_BMP180 pressure;
Adafruit_SI1145 uv = Adafruit_SI1145();

void setup()
{
    digitalWrite(13, HIGH);             // Turn on the status LED
  
    Serial.begin(9600);                  // Debugging only
    Serial.println("Initialising...");
    pinMode(13, OUTPUT);

    // Initialise the IO and ISR
    vw_set_ptt_inverted(true);          // Required for DR3100
    vw_setup(2000);                 // Bits per sec
    vw_set_tx_pin(transmit_pin);        // Pin D3
    
    // Sensor Setup
    if (pressure.begin())
      Serial.println("BMP180 init success");
    else
    {
      Serial.println("BMP180 init fail\n\n");
      while(1); // Pause forever.
    }


    if (! uv.begin()) {
      Serial.println("Didn't find Si1145");
      while (1);
    }
       
    power_spi_disable();                // Turn off stuff we don't need
    power_twi_disable(); 
     
    MCUSR &= ~(1<<WDRF);                // Clear the reset flag.
    WDTCSR |= (1<<WDCE) | (1<<WDE);     // In order to change WDE or the prescaler, we need to set WDCE (This will allow updates for 4 clock cycles).
    WDTCSR = 1<<WDP0 | 1<<WDP3;         // Set new watchdog timeout prescaler value - 8.0 seconds
    WDTCSR |= _BV(WDIE);                // Enable the WD interrupt (note no reset).
   
    Serial.println("Setup complete.");
    digitalWrite(13, LOW);              // Turn on the status LED
    delay(100);                         // Allow for serial print to complete.
}

void loop()
{
  if(f_wdt == 1)
  {   
    f_wdt = 0;   // Don't forget to clear the flag.
    enterSleep(); //  Re-enter sleep mode. 
  } 
  // Read Data from sensors
  String bmp180   = getPressureData(); // tmp, abs and rel pressure
  String humidity = "H:" + floatToString(getHumidity());
  
  String ir = getIR(); // Adafruit_SI1145
  String visible = getVisible();
  String uvi = getUvIndex();
 
  String light = "L:" + visible + ", U:" + uvi + ", IR: " + ir ;

  // Splitting the transmission payload. The message is too long for the library and smaller messages should be more reliable. 
  SendMessage(bmp180);
  delay(50);
  SendMessage(humidity);
  delay(50);
  SendMessage(light);

  Serial.println("Data Sent.");
  Serial.println("");
  delay(50);     
}

void SendMessage(String data){

  led_blink = led_blink + 1;
  if (led_blink == 4){
     digitalWrite(13, true); // Flash a light to show transmitting
     led_blink = 0;
  }

  // Convert the Data
  char tmpArr[data.length()+1];
  data.toCharArray(tmpArr, data.length()+1);
  char *msg = tmpArr;

  Serial.print("Sending: ");
  Serial.print( msg);
  Serial.println();

 
  vw_send((uint8_t *)msg, strlen(msg));
  vw_wait_tx(); // Wait until the whole message is gone
  digitalWrite(13, false);
  delay(10);
}

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
       result += "t:" + floatToString(T) + "";
      
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
          result += ", a: " + floatToString(P);
          
          // The pressure sensor returns abolute pressure, which varies with altitude.
          // To remove the effects of altitude, use the sealevel function and your current altitude.
          // This number is commonly used in weather reports.
          // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
          // Result: p0 = sea-level compensated pressure in mb
          p0 = pressure.sealevel(P,ALTITUDE); // we're at 1655 meters (Boulder, CO)

          result += ", r: " + floatToString(p0);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
  
  return result;
}

ISR(WDT_vect)
{
  // Watchdog Interrupt Service. This is executed when watchdog timed out.
  if(f_wdt == 0)
  {
    f_wdt=1;
  }
  else
  {
    f_wdt=1;
    Serial.println("WDT Overrun!!!");
  }
}

void enterSleep(void)
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   /* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */
  sleep_enable();
  
  /* Now enter sleep mode. */
  sleep_mode();
  
  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */
  
  /* Re-enable the peripherals. */
  power_all_enable();
}

String floatToString(float value) {
    char  outstr[15];
    dtostrf(value, 4, 2, outstr);    
    return outstr;
}

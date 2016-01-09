// Receove.ino
// Author: Scott (sr55)
// Copyright (C) 2016 Scott (sr55)

#include <VirtualWire.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#define aref_voltage 5.00 
#define LED_PIN (13)
#define ALTITUDE 142.0 // Altitude of Sensor

// Pins
int transmit_pin = 8;
int HIH4030_Pin = A3; //analog pin 0
int light_pin = A2;

// Globals
volatile int f_wdt=1;
SFE_BMP180 pressure;

void setup()
{
    digitalWrite(13, HIGH);             // Turn on the status LED
  
    Serial.begin(9600);	                // Debugging only
    Serial.println("Initialising...");
    pinMode(13, OUTPUT);

    // Initialise the IO and ISR
    vw_set_ptt_inverted(true);          // Required for DR3100
    vw_setup(2000);	                // Bits per sec
    vw_set_tx_pin(transmit_pin);        // Pin D3
    
    // Sensor Setup
    if (pressure.begin())
      Serial.println("BMP180 init success");
    else
    {
      Serial.println("BMP180 init fail\n\n");
      while(1); // Pause forever.
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
  String bmp180   = getPressureData();
  String humidity = floatToString(getHumidity());
  String lightLevel = String(getLightLevel());
  String data = bmp180 + ", Hum: " + humidity + "%, Light: " + lightLevel;
  Serial.println(data);
    
  // Transmit the data
  char tmpArr[data.length()+1];
  data.toCharArray(tmpArr, data.length()+1);
  char *msg = tmpArr;
  SendMessage(msg);
}

void SendMessage(char* msg){
  digitalWrite(13, true); // Flash a light to show transmitting
  delay(1000);
  vw_send((uint8_t *)msg, strlen(msg));
  vw_wait_tx(); // Wait until the whole message is gone
  digitalWrite(13, false);
}

int getLightLevel(){
   int reading  = analogRead(light_pin);
   return reading;
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
       result += "tmp:" + floatToString(T) + "C";
      
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
          result += ", abs: " + floatToString(P) + "mb";
          
          // The pressure sensor returns abolute pressure, which varies with altitude.
          // To remove the effects of altitude, use the sealevel function and your current altitude.
          // This number is commonly used in weather reports.
          // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
          // Result: p0 = sea-level compensated pressure in mb
          p0 = pressure.sealevel(P,ALTITUDE); // we're at 1655 meters (Boulder, CO)

          result += ", rel: " + floatToString(p0) + "mb";
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
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);   /* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */
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
    dtostrf(value, 7, 3, outstr);    
    return outstr;
}

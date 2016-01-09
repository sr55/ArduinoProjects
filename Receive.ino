// Receove.ino
// Author: Scott (sr55)
// Copyright (C) 2016 Scott (sr55)


#include <VirtualWire.h>
#include "Wire.h"
#include "LiquidCrystal.h"

int counter;
int retryCount;
int flipflop;
int RF315R_Pin = 9; 
bool continueProcessing;
LiquidCrystal lcd(0);

void setup()
{
    // Start the init process for the lcd and rx module.
    lcd.begin(16, 2); 
    lcd.setBacklight(HIGH);
    lcd.print("Initialising ...");
    
    Serial.begin(9600);	// Debugging only
    Serial.println("");

    // Initialise the IO and ISR
    vw_set_ptt_inverted(true); // Required for DR3100
    vw_setup(2000);	 // Bits per sec
    vw_set_rx_pin(RF315R_Pin);    // Configure the pin D2 to read the data   
    vw_rx_start();       // Start the receiver PLL running
}

void loop()
{
    delay(5000); // Slow down the looping.

    // Initialise vars
    uint8_t buf[VW_MAX_MESSAGE_LEN];
    uint8_t buflen = VW_MAX_MESSAGE_LEN;
    counter = counter + 1;

    // Check if we've got any data.   
    if (vw_get_message(buf, &buflen)) // Non-blocking
    {
        retryCount = 0; // We got data, reset the re-try counter
        counter = 0;
        
	      int i;
        digitalWrite(13, true); // Flash a light to show received good message
	      // Message with a good checksum received, dump it.

        char contents[73] = "Waiting ...";

      	for (i = 0; i < buflen; i++)
      	{
          contents[i] = (char)buf[i];
      	}
        String weather = contents;
        Serial.println(contents);

        // Makes it easier to see screen updates when weather data doesn't change.
        String flipflopchar = "/";
        if (flipflop == 1)
        {
           flipflopchar = "-";
           flipflop = 0;
        } else {
           flipflopchar = "/";
           flipflop = 1;
        }
        
        // Update the LCD
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("T:" + weather.substring(5,9) + "C H:" + weather.substring(52,56) + "% ");
        lcd.setCursor(0, 1);
        lcd.print("P:" + weather.substring(35,38) + "mb L:" + weather.substring(68,72) + " " + flipflopchar);
        
        digitalWrite(13, false);
    }

    // If we don't get data after 2 tries, 10 seconds, Display a no-signal warning.
    if (counter > 2) {
        retryCount = retryCount +1;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("No Signal");
        lcd.setCursor(0, 1);
        lcd.print("Retry Count: " + String(retryCount));
       
        counter = 0; // reset  
    }
}

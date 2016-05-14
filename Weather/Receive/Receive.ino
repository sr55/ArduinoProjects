// Receove.ino
// Author: Scott (sr55)
// Copyright (C) 2016 Scott (sr55)


#include <VirtualWire.h>
#include "Wire.h"
#include "LiquidCrystal.h"

int RF315R_Pin = 9; 
bool continueProcessing;
LiquidCrystal lcd(0);

// Current Data Stored outside the program loop.
String tmp;
String relative;
String absolute;
String light;
String uvi;
String irl;
String humidity;

// Display Control.
int displayScreenCounter = 0;


void setup()
{
    Serial.begin(9600);  // Debugging only
    Serial.println("Initialising ...");
    // Start the init process for the lcd and rx module.
    lcd.begin(16, 2); 
    lcd.setBacklight(HIGH);
    lcd.print("Initialising ...");
    
    // Initialise the IO and ISR
    vw_set_ptt_inverted(true); // Required for DR3100
    vw_setup(2000);	 // Bits per sec
    vw_set_rx_pin(RF315R_Pin);    // Configure the pin D2 to read the data   
    vw_rx_start();       // Start the receiver PLL running

    delay(100);  
}

void loop()
{
    // Initialise vars
    uint8_t buf[VW_MAX_MESSAGE_LEN];
    uint8_t buflen = VW_MAX_MESSAGE_LEN;
   
    // Check if we've got any data.   
    if (vw_get_message(buf, &buflen)) // Non-blocking
    {      
	      int i;
        digitalWrite(13, true); // Flash a light to show received good message

        // Read the contents of the buffer.
        char contents[73] = "";
      	for (i = 0; i < buflen; i++)
      	{
          contents[i] = (char)buf[i];
      	}
       
        String weather = contents;
        Serial.println(contents);

        // Handle the different message types
        String firstChar = weather.substring(0,1);
        if (firstChar == "t"){
          // Example: t:23.09, a: 1003.61, r: 1020.68
          Serial.println("Received BMP180 Data");
          
          int t_index = weather.indexOf("t:") +2;
          int a_index = weather.indexOf("a:") +2;
          int r_index = weather.indexOf("r:") +2;
          
          tmp = weather.substring(t_index, t_index + 4);
          relative = weather.substring(r_index, r_index + 6);
          absolute = weather.substring(a_index, a_index + 7);

          tmp.replace(',', ' ');
          tmp.trim();
          relative.replace(',', ' ');
          relative.trim();
          absolute.replace(',', ' ');
          absolute.trim();

          Serial.println("Parsed: T: " + tmp + ", R: " + relative + ", A: " + absolute);
          Serial.println("");
        } 

        // We will get 3 Transmissions.  Light, Temp/Baro and Humidity
        if (firstChar == "L"){
          // Example: L:269.00, U:0.06, IR: 370.00
          Serial.println("Received Light Data");

          int l_index = weather.indexOf("L:") +2;
          int u_index = weather.indexOf("U:") +2;
          int ir_index = weather.indexOf("IR:") +3;

          light = weather.substring(l_index, l_index + 7);
          uvi = weather.substring(u_index, u_index + 5);
          irl = weather.substring(ir_index, ir_index + 8);

          light.replace(',', ' ');
          light.trim();
          uvi.replace(',', ' ');
          uvi.trim();
          irl.replace(',', ' ');
          irl.trim();

          Serial.println("Parsed: L: " + light + ", UV: " + uvi + ", IR: " + irl);   
          Serial.println("");  
        }
        
        if (firstChar == "H"){
          Serial.println("Received Humidity Data");
          int h_index = weather.indexOf("H:") +2;
          humidity = weather.substring(h_index, h_index + 4);
          
          humidity.replace(',', ' ');
          humidity.trim();
                    
          Serial.println("Parsed:" + humidity);     
          Serial.println("");
        }
        
        // Update the LCD               
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("T:" + tmp + "C H:" + humidity + "%");

        // Note, this is going to change the second line of the display based on iterations of 3.
        // Since we get 3 transmits, we need to use 3.
        if (displayScreenCounter >= 0 && displayScreenCounter < 4){
          lcd.setCursor(0, 1);
          lcd.print("A:" + absolute + " R:" + relative);  
          displayScreenCounter = displayScreenCounter + 1; 
        } else if(displayScreenCounter >=4 && displayScreenCounter < 6) {
          lcd.setCursor(0, 1);
          lcd.print("L:" + light + " UV:" + uvi);  
          displayScreenCounter = displayScreenCounter + 1; 
        } else {
          displayScreenCounter = 0; 
        }      
        
        digitalWrite(13, false);
    }
}

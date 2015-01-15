// receiver.pde
//
// Simple example of how to use VirtualWire to receive messages
// Implements a simplex (one-way) receiver with an Rx-B1 module
//
// See VirtualWire.h for detailed API docs
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2008 Mike McCauley
// $Id: receiver.pde,v 1.3 2009/03/30 00:07:24 mikem Exp $

#include <VirtualWire.h>
#include <Time.h>

float RH, Pa, LL, Tm, RF, WS, WD;

void setup()
{
    Serial.begin(9600);	// Debugging only
    Serial.println("setup");

    // Initialise the IO and ISR
    vw_set_ptt_inverted(true); // Required for DR3100
    vw_setup(2000);	 // Bits per sec
    vw_set_rx_pin(8);
    vw_rx_start();       // Start the receiver PLL running
}

void loop()
{
    uint8_t buf[VW_MAX_MESSAGE_LEN];
    uint8_t buflen = VW_MAX_MESSAGE_LEN;
    
    //convert buffer of chars to a string
    if (vw_get_message(buf, &buflen)) // Non-blocking
    {
        time_t time = now(); 
        //recieve data and translate back to useable data
        String rawData;
	int i;
        
	for (i = 0; i < buflen; i++)
	{
            rawData = rawData + (char)buf[i];
	}
        String data = rawData.substring(2);
        int type = rawData.charAt(0) - 48;
        logData(data, type, time);  
        
    }
}

void logData(String data, int type, time_t time)
{
  // asign data to var acording to type
        switch (type)
        {
          case 0:
            //humidity
            RH = strToFloat(data);
            Serial.println(RH);
            Serial.println(" RH");
            
            break;
          case 1:
            //air presure
            Pa = strToFloat(data);
            Serial.println(Pa);
            Serial.println(" Pa");
            
            break;
          case 2:
            //light
            LL = strToFloat(data);
            Serial.println(LL);
            Serial.println(" Light");
            
            break;
          case 3:
            //temp
            Tm = strToFloat(data);
            Serial.println(Tm);
            Serial.println(" Temp");
            
            break;
        }
}

float strToFloat(String str)
{
  String integer = getValue(str, '.', 0);
  String decimal = getValue(str, '.', 1);
  float out = integer.toInt() + (decimal.toInt()/(decimal.length()*10.00));
  return out;
  
}

 String getValue(String data, const char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}


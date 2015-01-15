/**
 * Cosm Arduino sensor client example.
 *
 * This sketch demonstrates connecting an Arduino to Cosm (https://cosm.com),
 * using the new Arduino library to send and receive data.
 *
 * Requirements
 *   * Arduino with Ethernet shield or Arduino Ethernet (board must use the
 *     Wiznet Ethernet chipset)
 *   * Arduino software with version >= 1.0
 *   * An account at Cosm (https://cosm.com)
 *
 * Optional
 *   * An analog sensor connected to pin 2 (note we can still read a value from
 *     the pin without this)
 *
 * Created 8th January, 2013 using code written by Adrian McEwen with
 * modifications by Sam Mulube
 *
 * Full tutorial available here: https://cosm.com/docs/quickstart/arduino.html
 *
 * This code is in the public domain.
 */

#include <SPI.h>
#include <Ethernet.h>
#include <HttpClient.h>
#include <EthernetUdp.h>
#include <Time.h>
#include <Cosm.h>
#include <avr/pgmspace.h>
// Only used for dtostrf
#include <stdio.h>

#define API_KEY "mZ3NPPNX1Nn9sLnoQ4Hz-mwt9MSSAKxHYnozMy80dENRQT0g" // your Cosm API key
#define FEED_ID 131247 // your Cosm feed ID

// A UDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

// MAC address for your Ethernet shield
byte mac[] = {0x90, 0xA2, 0xDA, 0x0D, 0xD7, 0xC3 };

unsigned int localPort = 8888;      // local port to listen for UDP packets
IPAddress timeServer(132, 163, 4, 101); // time-a.timefreq.bldrdoc.gov NTP server

unsigned long lastConnectionTime = 0;                // last time we connected to Cosm
const unsigned long connectionInterval = 10000;      // delay between connecting to Cosm in milliseconds

// Initialize the Cosm library
char sensor0Id[] = "Pascals";
char sensor1Id[] = "Inches_Mercuy";
char sensor2Id[] = "Degrees_Farengeight";
char sensor3Id[] = "Degrees_Celsius";
char sensor4Id[] = "Dew_Point";
char sensor5Id[] = "Relative_Humidity";
char sensor6Id[] = "Light_Level";
CosmDatastream datastreams[] = {
  CosmDatastream(sensor0Id, strlen(sensor0Id), DATASTREAM_FLOAT),
  CosmDatastream(sensor1Id, strlen(sensor1Id), DATASTREAM_FLOAT),
  CosmDatastream(sensor2Id, strlen(sensor2Id), DATASTREAM_FLOAT),
  CosmDatastream(sensor3Id, strlen(sensor3Id), DATASTREAM_FLOAT),
  CosmDatastream(sensor4Id, strlen(sensor4Id), DATASTREAM_FLOAT),
  CosmDatastream(sensor5Id, strlen(sensor5Id), DATASTREAM_FLOAT),
  CosmDatastream(sensor6Id, strlen(sensor6Id), DATASTREAM_FLOAT)
  };

  // Wrap the datastream into a feed
  CosmFeed feed(FEED_ID, datastreams, 7 /* number of datastreams */);

EthernetClient client;
CosmClient cosmclient(client);

const int timeZone = 0;

// receiver.pde
// Simple example of how to use VirtualWire to receive messages
// Implements a simplex (one-way) receiver with an Rx-B1 module
//
// See VirtualWire.h for detailed API docs
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2008 Mike McCauley
// $Id: receiver.pde,v 1.3 2009/03/30 00:07:24 mikem Exp $

#include <VirtualWire.h>
#include <Time.h>

float RH, Pa, IM, LL, CC, TC, TF, RF, WS, WD, DP;
String relH, pasc, inM, lightL, cloudC, tempC, tempF, rainF, windS, windD, dewP;
const int dataNum = 11;//number of vars to store data
boolean dataSet[dataNum+1]; //index 0 is for over all wireless data recieve
boolean wuCon;
int i;;


void setup()
{
  Serial.begin(9600);	// Debugging only
  Serial.println("start up");

  // Initialise the IO and ISR
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(2000);	 // Bits per sec
  vw_set_rx_pin(8);
  vw_rx_start();       // Start the receiver PLL running
  
  while (Ethernet.begin(mac) != 1) {
    delay(5000);
    Serial.println("Re-Atempting Network Connection");
  }
  Serial.println("Ethernet Started");
  Udp.begin(localPort);
  setSyncProvider(getNtpTime);
  setSyncInterval(3600);    // Set the number of seconds between re-sync
  
  //data set flags = false
  setDF(false);
  
  //connect to weather underground servers
  wuCon = connectWund();
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
    int type = rawData.charAt(0) - 48; //decode data type ie. temperature, light
    logData(data, type, time);  
    dataSet[0] = true;
    
  }
  
  //upload data to COSM and Weather Underground
  if (millis() - lastConnectionTime > connectionInterval) {
    
    // send to Cosm
    setData(Pa, 0);
    setData(IM, 1);
    setData(TF, 2);
    setData(TC, 3);
    setData(DP, 4);
    setData(RH, 5);
    setData(LL, 6);
    sendData();

    //send to Wunderground
    putWU();

    // update connection time so we wait before connecting again
    lastConnectionTime = millis();

  }
}

void logData(String data, int type, time_t time)
{
  // asign data to var acording to type
  switch (type)
  {
  case 0:
    //humidity
    dataSet[1] = true;
    relH = data;
    RH = strToFloat(data);
    break;
  case 1:
    //air presure
    dataSet[2] = true;
    dataSet[3] = true;
    pasc = data;
    Pa = strToFloat(data);
    IM = Pa/3386.389; // most accurate at 60 degrees celsius
    break;
  case 2:
    //light
    dataSet[4] = true;
    lightL = data;
    LL = strToFloat(data);
    break;
  case 3:
    //temp
    dataSet[6] = true;
    dataSet[7] = true;
    tempC = data;
    TC = strToFloat(data);
    TF = TC * 1.8 + 32;
    break;
  }
  if (dataSet[1] & dataSet[6])
  {
    DP = pow(RH/100.0, 1.0/8.0)*(112+0.9*TC)+0.1*TC-112;//dewpoint calculation
    dataSet[11] = true;
  }
}

float strToFloat(String str)
{
  String integer = getValue(str, '.', 0);
  String decimal = getValue(str, '.', 1);
  float out = integer.toInt() + (decimal.toInt()/(decimal.length()*10.00));
  return out;

}

void setDF(boolean state)
{for (i=0; i<dataNum; i++) {dataSet[i] = state;}}

void setDF(int, boolean state) {dataSet[i] = state;}

String getValue(String data, const char separator, int index)
{
  int found = 0;
  int strIndex[] = {
    0, -1  };
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

void setData(float sensorValue, int sensorNum) {
  datastreams[sensorNum].setFloat(sensorValue);
}

// send the supplied value to Cosm, printing some debug information as we go
void sendData() {
  int ret = cosmclient.put(feed, API_KEY);
  if (ret == 200) {}
  else {}
}

//connect to the weather underground server
boolean connectWund()
{
  if (client.connect("weatherstation.wunderground.com", 80)) {
    return true;
  } else {return false;}

}

// put data to weather underground
void putWU()
{
  if (!client.connected()) {wuCon = connectWund();}
  if (wuCon) {
    String urlBase = "PUT /weatherstation/updateweatherstation.php?ID=KCONEDER19&PASSWORD=Kalidascope7&action=updateraw";
    String urlDate = urlBase + "&dateutc=" + year() + "-";
    if (month()<10) {urlDate = urlDate + "0" + month() + "-";}
    else {urlDate = urlDate + month() + "-";}
    if (day()<10) {urlDate = urlDate + "0" + day() + " ";}
    else {urlDate = urlDate + day() + " ";}
    if (hour()<10) {urlDate = urlDate + "0" + hour() + ":";}
    else {urlDate = urlDate + hour() + ":";}
    if (minute()<10) {urlDate = urlDate + "0" + minute() + ":";}
    else {urlDate = urlDate + minute() + ":";}
    if (second()<10) {urlDate = urlDate + "0" + second();}
    else {urlDate = urlDate + second();}
    
    String urlData = urlDate + "&tempf=";
    urlData = urlData + tempF + "&humidity=" + relH;// + "&dewptf=" + dewP + "&baromin=" + inM;
    client.println(urlData);//sends url with data to the server
  }
}

String wuRcvResponse()
{
  // if there are incoming bytes available 
  // from the server, read them and print them:
  if (client.available()) {
    char c = client.read();
  }

  // if the server's disconnected, resconnect
  if (!client.connected()) {while (!connectWund());}
}

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:                 
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}


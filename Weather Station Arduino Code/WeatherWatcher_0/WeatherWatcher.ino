#define LLPIN 0

#include <DHT22.h>
// Only used for dtostrf
#include <stdio.h>

// include virtual wire for sending out data via txmiter
#include <VirtualWire.h>

// Humidity/Temp sensor data wire is plugged into port 7 on the Arduino
// Connect a 4.7K resistor between VCC and the data pin (strong pullup)
#define DHT22_PIN 7

// Setup a DHT22 instance
DHT22 myDHT22(DHT22_PIN);

#include <Wire.h> // for IIC communication
const int SENSORADDRESS = 0x60; // address specific to the MPL3115A1

long int lastTX = 0;
const int txDelay = 500;

float LL, RH, T, PA;
int lastRead = 0;
boolean barConnected = false, dhtUpdate = false;
DHT22_ERROR_t errorCode;
const int ledPin = 9;

void setup()
{ 
  Wire.begin();        // join i2c bus
  Serial.begin(9600);  // just for debuging

  // This is a basic II2 communication check. If the sensor doesn't
  // return decicmal number 196 (see 0x0C register in datasheet), 
  // "IIC bad" is printed, otherwise nothing happens. 
  if(IIC_Read(0x0C) == 196) {
    barConnected = true;
  }  //checks who_am_i bit for basic I2C handshake test

  pinMode(ledPin, OUTPUT);
  // Enable and configure the sensor. 
  bar_sensor_config();

  // Initialise the wireless out and ISR
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(2000);	 // Bits per sec
  vw_set_tx_pin(8);

  delay((2000-millis()));
}

void loop()
{
  if (millis()-lastTX > txDelay)
  {
    getData();
    sendData();
    lastTX = millis();
  }
}

void getData()
{
  // The sensor humidity sensor can only be read from every 1-2s, and requires a minimum
  // 2s warm-up after power-on.

  LL = analogRead(LLPIN) / 1024.0 * 100.0;
  if (barConnected) {
    read_barometer();
  }
  errorCode = myDHT22.readData();
  if (errorCode == DHT_ERROR_NONE) 
  {
    RH = myDHT22.getHumidity();
    dhtUpdate = true;
  }
}

void sendData()
{
  //format all data to char array
  static char humidity[6];
  static char presure[9];
  static char light[6];
  static char temp[6];
  
  if (barConnected)
  {
    dtostrf(PA,8,2,presure);
    Serial.println(presure); 
    dtostrf(T,5,2,temp);
    Serial.println(T);
  }
  if (dhtUpdate)
  {
    dtostrf(RH,5,2,humidity);
    Serial.println(humidity);
  }
  dtostrf(LL,5,2,light);
  Serial.println(light);
  
  char* data[] = {humidity, presure, light, temp};
  int i;
  
  for (i=0;i<4;i++)
  {
    digitalWrite(ledPin, HIGH);
    String msg;
    msg = msg + String(i);
    msg = msg + "$" + data[i];
    char out[msg.length()+2];
    msg.toCharArray(out, String(data[i]).length()+3);
    Serial.println(out);
    rfOut(out, 2);
    digitalWrite(ledPin, LOW);
  }

  dhtUpdate = false;
}

void rfOut(const char *msg, int txRepeat)
{
  int i;
  for (i=0;i<txRepeat;i++)
  {
    vw_send((uint8_t *)msg, strlen(msg));
    vw_wait_tx(); // Wait until the whole message is gone
  }
}

boolean bar_check_new()
{
  // This function check to see if there is new data. 
  // You can call this function and it will return TRUE if there is
  // new data and FALSE if there is no new data.

  // If INT_SOURCE (0x12) register's DRDY flag is enabled, return
  if(IIC_Read(0x12) == 0x80) // check INT_SOURCE register on
    // new data ready (SRC_DRDY)
  {
    return true;
  }
  else return false;
}

void bar_sensor_config()
{
  // To configure the sensor, find the register hex value and 
  // enter it into the first field of IIC_Write (see below). Then
  // fingure out the hex value of the data you want to send.
  // 
  // For example:
  // For CTRL_REG1, the address is 0x26 and the data is 0xB9
  // 0xB9 corresponds to binary 1011 1001. Each place holder 
  // represents a data field in CTRL_REG1. 

  // CTRL_REG1 (0x26): enable sensor, oversampling, barometer mode 
  IIC_Write(0x26, 0x39);

  // CTRL_REG4 (0x29): Data ready interrupt enbabled
  IIC_Write(0x29, 0x80); 

  // PT_DATA_CFG (0x13): enable both pressure and temp event flags 
  IIC_Write(0x13, 0x07);
}

void read_barometer()
{
  // This function reads the presure and temperature registers, then 
  // concatenates the data together, and prints in values of 
  // pascals for presure and degrees C for temperature. 

  // variables for the calculations
  int m_presure, m_temp, c_presure, l_presure, w_presure; 
  // these must be floats since there is a fractional calculation
  float f_presure, l_temp;

  // read registers 0x01 through 0x05
  m_presure = (IIC_Read(0x01) << 10);
  c_presure = (IIC_Read(0x02) << 2);
  // the least significant bytes l_temp is 4-bit,
  // fractional values, so you must cast the calulation in (float),
  // shift the value over 4 spots to the right and divide by 16 (since 
  // there are 16 values in 4-bits). 
  // the least significant bytes l_presure is 2-bit,
  // fractional values, so you must cast the calulation in (float),
  // shift the value over 4 spots to the right and divide by 4 (since 
  // there are 4 values in 2-bits). 
  
  //Bit masks to separate LSB whole bits
  byte pWMask = -64;
  //Bit masks to separate LSB fractional bits
  byte pFMask = 48;
  l_presure = IIC_Read(0x03);
  w_presure = ((l_presure & pWMask)>>6);
  f_presure = (float)((l_presure & pFMask)>>4)/4.0;
  m_temp = IIC_Read(0x04); //temp, degrees
  l_temp = (float)(IIC_Read(0x05)>>4)/16.0; //temp, fraction of a degree

  // here is where we calculate the presure and temperature and store them in global variables fo later
  PA = (float)((m_presure|c_presure)|w_presure) + l_presure;
  T = (float)(m_temp + l_temp);

  // wait here for new data
  while(bar_check_new() == false);

}

// These are the two I2C functions in this sketch.
byte IIC_Read(byte regAddr)
{
  // This function reads one byte over IIC
  Wire.beginTransmission(SENSORADDRESS);
  Wire.write(regAddr);  // Address of CTRL_REG1
  Wire.endTransmission(false); // Send data to I2C dev with option
  //  for a repeated start. THIS IS
  //  NECESSARY and not supported before
  //  Arduino V1.0.1!!!!!!!!!
  Wire.requestFrom(SENSORADDRESS, 1); // Request the data...
  return Wire.read();
}

void IIC_Write(byte regAddr, byte value)
{
  // This function writes one byto over IIC
  Wire.beginTransmission(SENSORADDRESS);
  Wire.write(regAddr);
  Wire.write(value);
  Wire.endTransmission(true);
}


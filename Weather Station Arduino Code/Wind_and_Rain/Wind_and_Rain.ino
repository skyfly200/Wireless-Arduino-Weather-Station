#define uint  unsigned int
#define ulong unsigned long

#define PIN_ANEMOMETER  2     // Digital 2
#define PIN_RAIN        3     // Digital 3
#define PIN_VANE        1     // Analog 5

// How often we want to calculate wind speed or direction
#define MSECS_CALC_WIND_SPEED 5000
#define MSECS_CALC_RAIN_FALL 5000
#define MSECS_CALC_WIND_DIR   5000

volatile int numRevsAnemometer = 0; // Incremented in the interrupt
volatile int numRainFills = 0;      // Incremented in the interrupt
ulong nextCalcSpeed;                // When we next calc the wind speed
ulong nextCalcDir;                  // When we next calc the direction
ulong nextCalcRain;                 // When we next calc the rain fall
ulong time;                         // Millis() at each start of loop().

// ADC readings:
#define NUMDIRS 8
ulong   adc[NUMDIRS] = {26, 45, 77, 118, 161, 196, 220, 256};

// These directions match 1-for-1 with the values in adc, but
// will have to be adjusted as noted above. Modify 'dirOffset'
// to which direction is 'away' (it's West here).
char *strVals[NUMDIRS] = {"W","NW","N","SW","NE","S","SE","E"};
byte dirOffset=0;

//=======================================================
// Initialize
//=======================================================
void setup() {
   Serial.begin(9600);
   pinMode(PIN_ANEMOMETER, INPUT);
   digitalWrite(PIN_ANEMOMETER, HIGH);
   pinMode(PIN_RAIN, INPUT);
   digitalWrite(PIN_RAIN, HIGH);
   attachInterrupt(0, countAnemometer, FALLING);
   attachInterrupt(1, countRainFall, FALLING);
   nextCalcSpeed = millis() + MSECS_CALC_WIND_SPEED;
   nextCalcRain = millis() + MSECS_CALC_RAIN_FALL;
   nextCalcDir   = millis() + MSECS_CALC_WIND_DIR;
}

//=======================================================
// Main loop.
//=======================================================
void loop() {
   time = millis();

   if (time >= nextCalcSpeed) {
      calcWindSpeed();
      nextCalcSpeed = time + MSECS_CALC_WIND_SPEED;
   }
   if (time >= nextCalcDir) {
      calcWindDir();
      nextCalcDir = time + MSECS_CALC_WIND_DIR;
   }
   if (time >= nextCalcRain) {
      calcRainFall();
      nextCalcRain = time + MSECS_CALC_RAIN_FALL;
   }
}

//=======================================================
// Interrupt handler for anemometer. Called each time the reed
// switch triggers (one revolution).
//=======================================================
void countAnemometer() {
   numRevsAnemometer++;
}

//=======================================================
// Interrupt handler for Rain Fall Meter. Called each time the reed
// switch triggers (one latch opening).
//=======================================================
void countRainFall() {
   numRainFills++;
}

//=======================================================
// Find vane direction.
//=======================================================
void calcWindDir() {
   int val;
   byte x, reading;

   val = analogRead(PIN_VANE);
   val >>=2;                        // Shift to 255 range
   reading = val;

   // Look the reading up in directions table. Find the first value
   // that's >= to what we got.
   for (x=0; x<NUMDIRS; x++) {
      if (adc[x] >= reading)
         break;
   }
   //Serial.println(reading, DEC);
   x = (x + dirOffset) % 8;   // Adjust for orientation
   Serial.print("  Dir: ");
   Serial.print(strVals[x]);
}


//=======================================================
// Calculate the wind speed, and display it (or log it, whatever).
// 1 rev/sec = 1.492 mph
//=======================================================
void calcWindSpeed() {
   int x, iSpeed;
   // This will produce mph * 10
   // (didn't calc right when done as one statement)
   long speed = 14920;
   speed *= numRevsAnemometer;
   speed /= MSECS_CALC_WIND_SPEED;
   iSpeed = speed;         // Need this for formatting below

   Serial.print("Wind speed: ");
   x = iSpeed / 10;
   Serial.print(x);
   Serial.print('.');
   x = iSpeed % 10;
   Serial.print(x);

   numRevsAnemometer = 0;        // Reset counter
}

void calcRainFall()
{
   int x, iRain;
   // This will produce inches * 1000
   // (didn't calc right when done as one statement)
   long rain = 11;
   rain *= numRevsAnemometer;
   iRain = rain;         // Need this for formatting below

   Serial.print("  Rain Fall: ");
   x = iRain / 10000;
   Serial.print(x);
   Serial.print('.');
   x = iRain % 10000;
   Serial.println(x);

   numRevsAnemometer = 0;        // Reset counter
}

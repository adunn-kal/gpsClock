#include <Arduino.h>
// #include "Adafruit_GPS.h"
#include "gpsClock.h"

#define myRX GPIO_NUM_16
#define myTX GPIO_NUM_17
#define myEN GPIO_NUM_27


GpsClock myGPS(&Serial2, myRX, myTX, myEN);
uint32_t timer = 0;
uint32_t timer2 = 0;

void setup()
{
  Serial.begin(115200);
  while (!Serial) {}
}

void loop()
{
  Adafruit_GPS myClock = myGPS.begin();
  while (true)
  {
    // myClock.read();
    if ((millis() - timer) > 100)
    {
      timer = millis();
      myGPS.update(myClock);
      // Serial.println(myGPS.getTime(myClock));
      
      if (myGPS.newData)
      {
        myGPS.read(myClock);
        // Serial.printf("%f, %f, %f, %d, %d\n", myGPS.latitude, myGPS.longitude, myGPS.altitude, myGPS.fixType, myGPS.millisOffset);
        // Serial.println(myClock.milliseconds);
      }
    }

    if ((millis() - timer2) > 200)
    {
      timer2 = millis();

      Serial.println();
      Serial.println(myGPS.getDisplayTime(myClock));
      Serial.println(myGPS.getUnixTime(myClock));
    }
  }
}
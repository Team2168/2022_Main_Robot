#include <i2c_t3.h>
#include <math.h>
#include "TCS34725/TCS34725.h"
#include <fstream>
#define HWSERIAL Serial1
TCS34725 tcs;

uint8_t buff[4]; // {r, g, b, checksum}

void setup(void)
{
    Serial.begin(9600);
    HWSERIAL.begin(9600);
    Wire.begin();//comm with sensor. teensy master
    
    if (!tcs.attach(Wire))
        Serial.println("ERROR: TCS34725 NOT FOUND !!!");
   // tcs.integrationTime(33); // ms
    tcs.gain(TCS34725::Gain::X01);
    // set LEDs...
}
void loop(void)
{
    if (tcs.available()) // if current measurement has done
    {
          TCS34725::Color color = tcs.color();
          Serial.print("R: "); Serial.print(color.r); Serial.print("     ");
          Serial.print("G: "); Serial.print(color.g); Serial.print("     ");
          Serial.print("B: "); Serial.print(color.b); Serial.print("     ");
          Serial.println();
  
          buff[0] = round(color.r);  // convert float to int
          buff[1] = round(color.g);
          buff[2] = round(color.b);
          buff[3] = buff[0] ^ buff[1] ^ buff[2];
      
          if(HWSERIAL.available())
          {
                Serial.println("writing to serial!"); 
                HWSERIAL.write(buff,4);
                delay(20); // Write to bus every 20ms
          }
    }
}

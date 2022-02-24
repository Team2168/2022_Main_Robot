#include <i2c_t3.h>
#include <math.h>
#include "TCS34725/TCS34725.h"
#include <fstream>
#define HWSERIAL Serial1
TCS34725 tcs;

uint8_t buff[4]; // {r, g, b, checksum}

const int ledPin = LED_BUILTIN;
long interval_s = 1000;
int ledState = LOW;
unsigned long previousMillis = 0;


void setup(void)
{
  pinMode(ledPin, OUTPUT);
   
  Serial.begin(9600);
  HWSERIAL.begin(9600);
  Wire.begin();//comm with sensor. teensy master
    
  if (!tcs.attach(Wire)) {
    Serial.println("ERROR: TCS34725 NOT FOUND !!!");
    interval_s = 100; //Blink fast - the sensor's not plugged in?
  }
  //tcs.integrationTime(33); // ms
  tcs.gain(TCS34725::Gain::X01);

  buff[0] = 0;
  buff[1] = 0;
  buff[2] = 0;
  buff[3] = 0;
}

//IS THIS THING ON?
void toggleLED()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval_s) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  }
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
  }
  Serial.println("writing to serial !"); 
  HWSERIAL.write(buff,4);

  toggleLED();
  
  delay(50); // Write to bus every 50ms
}

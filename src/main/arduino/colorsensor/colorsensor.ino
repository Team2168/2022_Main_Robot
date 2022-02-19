#include <i2c_t3.h>
#include "TCS34725/TCS34725.h"
#include <fstream>
#define HWSERIAL Serial1
TCS34725 tcs;
uint8_t buff[3];
//uint8_t r_norm,g_norm,b_norm;
uint8_t r=0;
uint8_t g=0;
uint8_t b=0;

void setup(void)
{
    Serial.begin(9600);
    HWSERIAL.begin(9600);
    Wire.begin();//comm with sensor. teensy master
   // Wire1.begin(2);//comm with rio. teensy slave
    
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
        //Serial.print("Color Temp : "); Serial.println(tcs.colorTemperature());
        //Serial.print("Lux        : "); Serial.println(tcs.lux());
        Serial.print("R: "); Serial.print(color.r); Serial.print("     ");
        Serial.print("G: "); Serial.print(color.g); Serial.print("     ");
        Serial.print("B: "); Serial.print(color.b); Serial.print("     ");
        Serial.println();
        //r=color.r;
        //g=color.g;
        //b=color.b;
        buff[0]=color.r;
        buff[1]=color.g;
        buff[2]=color.b;
    }
    if(HWSERIAL.available()){
     byte value=HWSERIAL.read();
     
     if(value==0x12){
     HWSERIAL.write(buff,3);
     }
    }
    //requestEvent();
    //else Serial.println("ERROR: TCS34725 NOT FOUND !!!");
   // Wire1.onRequest(requestEvent);
       
}

// this function is registered as an event, see setup()
void requestEvent(){
  uint8_t buff[3];
  buff[0]= r;
  buff[1]= g;
  buff[2]= b;
  HWSERIAL.write(buff,3);
          Serial.println("sending");

}

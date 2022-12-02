#include "MPU.h"
TorpedoMPU tmpu(0,1,2);

void setup()
{   
    Serial.begin(115200);
    tmpu.start();
    tmpu.check();
}

void loop()
{
   tmpu.calculate();
   Serial.print("Roll: ");
   Serial.print(tmpu.Return_roll());
   Serial.print(":");
   Serial.print("Pitch: ");
   Serial.print(tmpu.Return_pitch());
   Serial.print(":");
   Serial.print("Yaw: ");
   Serial.print(tmpu.Return_yaw());
   Serial.println(".");
}

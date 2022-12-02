#ifndef MPU_H_
#define MPU_H_

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
class TorpedoMPU:public MPU6050{
private:
bool dmpReady = false;  
uint8_t mpuIntStatus;  
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;    
uint8_t fifoBuffer[64];
Quaternion q;           
VectorFloat gravity;    
float ypr[3];           
volatile bool mpuInterrupt = false; 
float roll=0;
float pitch=0;
float yaw=0;  
public:
TorpedoMPU(float roll,float pitch,float yaw){
    this->roll=roll;
    this->pitch=pitch;
    this->yaw=yaw;
};
public:
void TorpedoMPU::start(){
    mpuInterrupt = true;
    Wire.begin();
    Wire.setClock(400000);
    while (!Serial); 
    Serial.println(F("Initializing I2C devices..."));
    initialize();
    Serial.println(F("Testing device connections..."));
    Serial.println(testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(F("Initializing DMP..."));
    devStatus = dmpInitialize();
    
    setXGyroOffset(220);
    setYGyroOffset(76);
    setZGyroOffset(-85);
    setZAccelOffset(1788);
}
void TorpedoMPU::check(){
     if (devStatus == 0)
    {
           CalibrateAccel(6);
           CalibrateGyro(6);
           PrintActiveOffsets();

        Serial.println(F("Enabling DMP..."));
        setDMPEnabled(true);
        
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        mpuIntStatus = getIntStatus();
        
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        packetSize = dmpGetFIFOPacketSize();  
    }
    else
    {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}
void TorpedoMPU::calculate(){
    if (!dmpReady) return;
    mpuInterrupt = false;
    mpuIntStatus = getIntStatus();
    fifoCount = getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        resetFIFO();
        Serial.println(F("FIFO overflow!"));
    }
    else if (mpuIntStatus & 0x02)
    {
        while (fifoCount < packetSize) fifoCount = getFIFOCount();
        getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        dmpGetQuaternion(&q, fifoBuffer);
        dmpGetGravity(&gravity, &q);
        dmpGetYawPitchRoll(ypr, &q, &gravity);       
   }
    if(abs(yaw * 180/M_PI - ypr[0] * 180/M_PI) > 0){
          yaw = ypr[0];
  }
   roll = (ypr[2] * 180/M_PI);
   pitch = (ypr[1] * 180/M_PI);
   yaw = (yaw * 180/M_PI);
} 
float TorpedoMPU::Return_roll(){
    calculate();
    return roll;
}
float TorpedoMPU::Return_pitch(){
    calculate();
    return pitch;
}
float TorpedoMPU::Return_yaw(){
    calculate();
    return yaw;
}
};
#endif

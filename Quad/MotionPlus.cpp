#include <Arduino.h>
#include <Wire.h>
#include "MotionPlus.h"

const int kBeginAddress = 0x53;
const int kDataAddress = 0x52;
const int kSlowScale = 20;
const int kFastScale = 5;

void MotionPlus::init(int calibrationCount)
{      
    Wire.begin();
    MotionPlus::_sendByte(kBeginAddress, 0x04, 0xfe);
    MotionPlus::_calibrate(calibrationCount);
}
    
void MotionPlus::update()
{ 
    int yawScale, pitchScale, rollScale;
    
    MotionPlus::_sendZero(kDataAddress);
    Wire.requestFrom(kDataAddress, 6);

    int data[6];
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
    data[3] = Wire.read();
    data[4] = Wire.read();
    data[5] = Wire.read();

    int slowRoll  = data[3] & 1;
    int slowYaw   = data[3] & 2;
    int slowPitch = data[4] & 2;

    yawScale   = (slowYaw)   ? kSlowScale : kFastScale;
    rollScale  = (slowRoll)  ? kSlowScale : kFastScale;
    pitchScale = (slowPitch) ? kSlowScale : kFastScale;

    yaw_r   =      (((data[3] >> 2) << 8) + data[0] - yaw0)   / yawScale;
    roll_r  = -1 * (((data[4] >> 2) << 8) + data[1] - pitch0) / rollScale;
    pitch_r =      (((data[5] >> 2) << 8) + data[2] - roll0)  / pitchScale;
}
  
void MotionPlus::_sendByte(byte address, byte data, byte location)
{   
    Wire.beginTransmission(address);
    Wire.write(location);
    Wire.write(data);  
    Wire.endTransmission();
}

void MotionPlus::_sendZero(byte address)
{
    Wire.beginTransmission(address);
    Wire.write(0x00);
    Wire.endTransmission();
}

void MotionPlus::_calibrate(int calibrationCount)
{
    yaw0 = 0;
    pitch0 = 0;
    roll0 = 0;
    
    for (int i=0; i<calibrationCount; i++) {
        MotionPlus::_sendZero(kDataAddress);
        Wire.requestFrom(kDataAddress, 6);
        
        int data[6];
        data[0] = Wire.read();
        data[1] = Wire.read();
        data[2] = Wire.read();
        data[3] = Wire.read();
        data[4] = Wire.read();
        data[5] = Wire.read();
        yaw0   += (((data[3] >> 2) << 8) + data[0]);
        roll0  += (((data[4] >> 2) << 8) + data[1]);
        pitch0 += (((data[5] >> 2) << 8) + data[2]);
        delay(2);
    }


    yaw0   /= calibrationCount;
    roll0  /= calibrationCount;
    pitch0 /= calibrationCount;
}

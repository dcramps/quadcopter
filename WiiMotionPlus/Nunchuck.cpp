#include <Arduino.h>
#include <Wire.h>
#include "Nunchuck.h"

#define ADDRESS 0x52


void Nunchuck::init()
{
    Nunchuck::init(10);
}

void Nunchuck::init(int calibrationCount)
{            
    Wire.begin();
    
    Nunchuck::_sendByte(0x55, 0xF0);
    Nunchuck::_sendByte(0x00, 0xFB);
    Nunchuck::update();
}
        
void Nunchuck::update()
{ 
    Wire.requestFrom(ADDRESS, 6); 
    
    int data[6];
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
    data[3] = Wire.read();
    data[4] = Wire.read();
    data[5] = Wire.read();
    
    Nunchuck::accelX = data[2] * 2 * 2 + ((data[5] >> 2) & 1) * 2 + ((data[5] >> 3) & 1) - accelX0;
    Nunchuck::accelY = data[3] * 2 * 2 + ((data[5] >> 4) & 1) * 2 + ((data[5] >> 5) & 1) - accelY0;
    Nunchuck::accelZ = data[4] * 2 * 2 + ((data[5] >> 6) & 1) * 2 + ((data[5] >> 7) & 1) - accelZ0;
    
    Nunchuck::_sendZero(ADDRESS);
}
    
void Nunchuck::_sendByte(byte data, byte location)
{    
    Wire.beginTransmission(ADDRESS);
    Wire.write(location);
    Wire.write(data);    
    Wire.endTransmission();
}

void Nunchuck::_sendZero(byte address)
{
    Wire.beginTransmission(address);
    Wire.write(0x00);
    Wire.endTransmission();
}


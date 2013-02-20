#ifndef Nunchuck_H
#define Nunchuck_H

#include <Arduino.h>

class Nunchuck
{  
    public:
        int16_t accelX;
        int16_t accelY;
        int16_t accelZ;
    
	void init();
        void init(int calibrationCount);    
        void update();
        
    private:  
        int32_t accelX0;
        int32_t accelY0;
        int32_t accelZ0;
        
        void _sendByte(byte data, byte location);
	void _calibrate(int calibrationCount);
        void _sendZero(byte address);
};

#endif

#ifndef Nunchuck_H
#define Nunchuck_H

#include <Arduino.h>

class Nunchuck
{  
    public:
        int accelX;
        int accelY;
        int accelZ;
    
	void init();
        void init(int calibrationCount);    
        void update();
        
    private:  
        int accelX0;
        int accelY0;
        int accelZ0;
        
        void _sendByte(byte data, byte location);
	void _calibrate(int calibrationCount);
        void _sendZero(byte address);
};

#endif

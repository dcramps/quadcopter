#ifndef MotionPlus_H
#define MotionPlus_H

#include <Arduino.h>
class MotionPlus
{  
    public:  
        int yaw;
        int pitch;
        int roll;
	
        void init();
        void init(int calibrationCount);    
        void update();

    private:  
    	int yaw0;
        int pitch0;
        int roll0;
    
        void _sendByte(byte address, byte data, byte location);
        void _sendZero(byte address);
	void _calibrate(int calibrationCount);	
};

#endif

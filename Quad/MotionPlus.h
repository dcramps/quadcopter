#ifndef MotionPlus_H
#define MotionPlus_H

#include <Arduino.h>
class MotionPlus
{  
    public:
        int16_t yaw_r;
        int16_t pitch_r;
        int16_t roll_r;
	
        void init();
        void init(int calibrationCount);    
        void update();

    private:  
    	int32_t yaw0;
        int32_t pitch0;
        int32_t roll0;
    
        void _sendByte(byte address, byte data, byte location);
        void _sendZero(byte address);
	void _calibrate(int calibrationCount);	
};

#endif

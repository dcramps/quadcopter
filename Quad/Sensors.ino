
//WMP and Nunchuck communicate on the same I²C address
//so use two NPN transistors to switch the data line
//between them as needed.
//Setting a line to HIGH enables it, LOW disables it.

inline void switchWmp() 
{
    digitalWrite(NUNCHUCK,   LOW);
    digitalWrite(MOTIONPLUS, LOW);
    digitalWrite(MOTIONPLUS, HIGH);    
}

inline void switchNunchuck()
{
    digitalWrite(MOTIONPLUS, LOW);
    digitalWrite(NUNCHUCK,   LOW);
    digitalWrite(NUNCHUCK,   HIGH);
}

inline void getAccel()
{
    switchNunchuck();
    nunchuck.update();

    accelX = -1 * (((nunchuck.accelX * VddStep) - Voff) * SoInv - 0.05);
    accelY =       ((nunchuck.accelY * VddStep) - Voff) * SoInv - 0.03;
    accelZ = -1 * (((nunchuck.accelZ * VddStep) - Voff) * SoInv);
}

inline void getGyro()
{
    switchWmp();
    wmp.update();    

    gyroX = -1 *  RAD(wmp.roll_r);
    gyroY = -1 *  RAD(wmp.pitch_r);
    gyroZ =       RAD(wmp.yaw_r);
}

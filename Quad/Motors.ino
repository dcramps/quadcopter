void initMotors()
{
    //Pitch: M1,M4
    motor1.attach(kMotorPin1);
    motor4.attach(kMotorPin4);    

    //Roll: M2,M3
    motor2.attach(kMotorPin2);
    motor3.attach(kMotorPin3);

    delay(100);

    motor1value = kThrottleMin;
    motor2value = kThrottleMin;
    motor3value = kThrottleMin;
    motor4value = kThrottleMin;

    motor1.writeMicroseconds(kThrottleMin);
    motor2.writeMicroseconds(kThrottleMin);
    motor3.writeMicroseconds(kThrottleMin);
    motor4.writeMicroseconds(kThrottleMin);

    delay(1000);
}

inline void updateMotors()
{
    if (motorsEnabled) {
        ledOn();
        
        //pitch motors
        motor1value = constrain(throttle-out_p, kThrottleMin, kThrottleMax);
        motor1.writeMicroseconds(motor1value);

        motor4value = constrain(throttle+out_p, kThrottleMin, kThrottleMax);
        motor4.writeMicroseconds(motor4value);

        //roll motors
        //    motor2.writeMicroseconds(constrain(throttleMin+out_r,1000,2000));
        //    motor3.writeMicroseconds(constrain(throttleMin-out_r,1000,2000));
    } else {
        ledOff();
        
        motor1value = kThrottleMin;
        motor2value = kThrottleMin;
        motor3value = kThrottleMin;
        motor4value = kThrottleMin;

        motor1.writeMicroseconds(kThrottleMin);
        motor2.writeMicroseconds(kThrottleMin);
        motor3.writeMicroseconds(kThrottleMin);
        motor4.writeMicroseconds(kThrottleMin);
    }
}




void initMotors()
{
    motor1.attach(kMotorPin1);
    motor2.attach(kMotorPin2);
    motor3.attach(kMotorPin3);
    motor4.attach(kMotorPin4);

    delay(100);

    motor1value = kThrottleMin;
    motor2value = kThrottleMin;
    motor3value = kThrottleMin;
    motor4value = kThrottleMin;

    motor1.writeMicroseconds(kThrottleMin);
    motor2.writeMicroseconds(kThrottleMin);
    motor3.writeMicroseconds(kThrottleMin);
    motor4.writeMicroseconds(kThrottleMin);

    motor1.writeMicroseconds(1250);
    motor2.writeMicroseconds(1250);
    delay(250);
    motor1.writeMicroseconds(kThrottleMin);
    motor2.writeMicroseconds(kThrottleMin);
    delay(250);
    motor3.writeMicroseconds(1250);
    motor4.writeMicroseconds(1250);
    delay(250);
    motor3.writeMicroseconds(kThrottleMin);
    motor4.writeMicroseconds(kThrottleMin);

    delay(1000);

    motor1.writeMicroseconds(1250);
    delay(20);
    motor1.writeMicroseconds(kThrottleMin);
    delay(250);
    motor2.writeMicroseconds(1250);
    delay(20);
    motor2.writeMicroseconds(kThrottleMin);
    delay(250);
    motor3.writeMicroseconds(1250);
    delay(20);
    motor3.writeMicroseconds(kThrottleMin);
    delay(250);
    motor4.writeMicroseconds(1250);
    delay(20);
    motor4.writeMicroseconds(kThrottleMin);
    delay(2500);
}

inline void armMotors()
{
    motorsEnabled = true;
}

inline void disarmMotors()
{
    motorsEnabled = false;
    throttle = 0;
    yaw = 128;
    pitch = 128;
    roll = 128;
}

inline void updateMotors()
{
    if (motorsEnabled) {
        ledOn();
        int throttleValue = map(throttle, 0, 255, kThrottleMin, kThrottleMax);

        //Pitch forward = 1/2 less than 3/4
        motor1value = constrain(throttleValue+outPitch, kThrottleMin, kThrottleMax);
        motor2value = constrain(throttleValue+outPitch, kThrottleMin, kThrottleMax);

        motor3value = constrain(throttleValue-outPitch, kThrottleMin, kThrottleMax);
        motor4value = constrain(throttleValue-outPitch, kThrottleMin, kThrottleMax);

        motor1.writeMicroseconds(motor1value);
        motor2.writeMicroseconds(motor2value);
        motor3.writeMicroseconds(motor3value);
        motor4.writeMicroseconds(motor4value);

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




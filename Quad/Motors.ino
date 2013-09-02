void initMotors()
{
    //Pitch: M1,M4
    motor1.attach(MOTOR_1_PIN);
    motor4.attach(MOTOR_4_PIN);    

    //Roll: M2,M3
    motor2.attach(MOTOR_2_PIN);
    motor3.attach(MOTOR_3_PIN);

    delay(100);

    motor1value = THROTTLE_ZERO;
    motor2value = THROTTLE_ZERO;
    motor3value = THROTTLE_ZERO;
    motor4value = THROTTLE_ZERO;

    motor1.writeMicroseconds(THROTTLE_ZERO);
    motor2.writeMicroseconds(THROTTLE_ZERO);
    motor3.writeMicroseconds(THROTTLE_ZERO);
    motor4.writeMicroseconds(THROTTLE_ZERO);

    delay(1000);
}

inline void updateMotors()
{
    if (motorsEnabled) {
        ledOn();
        
        //pitch motors
        motor1value = constrain(THROTTLE_SET-out_p,THROTTLE_ZERO,THROTTLE_MAX);
        motor1.writeMicroseconds(motor1value);

        motor4value = constrain(THROTTLE_SET+out_p,THROTTLE_ZERO,THROTTLE_MAX);
        motor4.writeMicroseconds(motor4value);

        //roll motors
        //    motor2.writeMicroseconds(constrain(throttleMin+out_r,1000,2000));
        //    motor3.writeMicroseconds(constrain(throttleMin-out_r,1000,2000));
    } else {
        ledOff();
        
        motor1value = THROTTLE_ZERO;
        motor2value = THROTTLE_ZERO;
        motor3value = THROTTLE_ZERO;
        motor4value = THROTTLE_ZERO;

        motor1.writeMicroseconds(THROTTLE_ZERO);
        motor2.writeMicroseconds(THROTTLE_ZERO);
        motor3.writeMicroseconds(THROTTLE_ZERO);
        motor4.writeMicroseconds(THROTTLE_ZERO);
    }
}




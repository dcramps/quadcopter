void initMotors()
{
    //Pitch: M1,M4
    motor1.attach(MOTOR_1_PIN);
    motor4.attach(MOTOR_4_PIN);    
    
    //Roll: M2,M3
    motor2.attach(MOTOR_2_PIN);
    motor3.attach(MOTOR_3_PIN);
    
    delay(100);
    
    motor1.writeMicroseconds(1000);
    motor2.writeMicroseconds(1000);
    motor3.writeMicroseconds(1000);
    motor4.writeMicroseconds(1000);
    
    delay(1000);
}

inline void updateMotors()
{
    //pitch motors
    motor1.writeMicroseconds(constrain(throttleMin-out_p,1000,2000));
    motor4.writeMicroseconds(constrain(throttleMin+out_p,1000,2000));

    //roll motors
//    motor2.writeMicroseconds(constrain(throttleMin+out_r,1000,2000));
//    motor3.writeMicroseconds(constrain(throttleMin-out_r,1000,2000));
}


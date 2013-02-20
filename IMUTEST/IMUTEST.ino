#include <openIMU.h>
float accelX, accelY, accelZ, gyroX, gyroY, gyroZ, dt;
long timer;

openIMU imu(&gyroX, &gyroY, &gyroZ, &accelX, &accelY, &accelZ, &dt);

void setup()
{
    Serial.begin(115200);
    accelX = 0.01;
    accelY = 0.0;
    accelZ = 1.0;
    gyroX = 0.035;
    gyroY = 0.00;
    gyroZ = 0.00;
    
    timer = millis();
}

void loop()
{
    dt = millis()-timer * 0.001;
    timer=millis();
    if (millis()-timer <= 3) {
        imu.IMUupdate();
        imu.GetEuler();
        Serial.print(imu.pitch);
        Serial.print(",");
        Serial.println(imu.roll);
    }
}


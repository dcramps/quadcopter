#include <PID_v1.h>
#include <openIMU.h>
#include "MotionPlus.h"
#include "Nunchuck.h"
#include <SPI.h>
// #include <WiFly.h>
#include "SpiUart.h"
#include <Wire.h>
#include <Servo.h>

#define RAD(x) ((x) * 0.0174532925)
template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; } 

//Pin constants
const int kMotionPlusPin = 2;
const int kNunchukPin = 4;
const int kMotorPin1 = 9;
const int kMotorPin2 = 5;
const int kMotorPin3 = 3;
const int kMotorPin4 = 6;

//IMU values
float accelX = 0;
float accelY = 0;
float accelZ = 0;
float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;

//IMU Constants 
//Values derived from the datasheet
//http://html.alldatasheet.com/html-pdf/171987/STMICROELECTRONICS/LIS3L02AL/9754/5/LIS3L02AL.html

const float Vdd = 3.3;
const float Voff = Vdd/2.0f;// + (Vdd/2.0f)*(0.0015); //Vdd/2 +/-6%
const float So = Vdd/5.0f; //Vdd/5 +/-10%
const float SoInv = 1/So;
const float radToDeg = 180/M_PI;
const float VddStep = Vdd/1023;

//timing
long  timer = 0; //millis at start of calculation
int   diff  = 0; //difference between now and last cycle
float dt    = 0; //delta in seconds
int   calc  = 0; //number of calculations since last data transmit

char ssid[] = "drone";

//Gyro
MotionPlus wmp = MotionPlus();

//Accel
Nunchuck nunchuck = Nunchuck();

//IMU
openIMU imu(&gyroX, &gyroY, &gyroZ, &accelX, &accelY, &accelZ, &dt);

//SPI for WiFly communication
SpiUartDevice SpiSerial;

//PID
double setPitch;
double outPitch;

double setRoll;
double outRoll;

PID pitchPID((double*)&imu.pitch, &outPitch, &setPitch, 2.15, 0.03, 0.56, DIRECT);
PID  rollPID((double*)&imu.roll,  &outRoll, &setRoll, 0.0, 0.0, 0.0, DIRECT);

//Motors
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

int motor1value = 1000;
int motor2value = 1000;
int motor3value = 1000;
int motor4value = 1000;

float yawZero = 0.0;
float pitchZero = 0.0;
float rollZero = 0.0;

boolean motorsEnabled = false;

void ledOn() 
{
    digitalWrite(13,HIGH);
}

void ledOff()
{
    digitalWrite(13,LOW);
}

void setup()
{
    Serial.begin(115200);
    Serial << "Connecting to SPI";
    SpiSerial.begin();
    Serial << "...done\n";

    pinMode(13,OUTPUT);
    ledOn();
    initMotors();
    TWBR = ((F_CPU / 4000000) - 16) / 2;

    pinMode(kMotionPlusPin, OUTPUT);
    pinMode(kNunchukPin, OUTPUT);

    digitalWrite(kNunchukPin, HIGH);
    digitalWrite(kMotionPlusPin, HIGH);

    Serial << "Starting gyroscope";
    switchWmp();
    wmp.init(500);
    Serial << "...done\n";

    Serial << "Starting accelerometer";
    switchNunchuck();
    nunchuck.init(10);
    Serial << "...done\n";

    timer = millis();

    pitchPID.SetMode(AUTOMATIC);
    pitchPID.SetOutputLimits(-200,200);
    pitchPID.SetSampleTime(3);

    rollPID.SetMode(AUTOMATIC);
    rollPID.SetOutputLimits(-1000,1000);
    rollPID.SetSampleTime(3);

    delay(2000);
    ledOff();

    Serial << "Flushing SPI";
    SpiSerial.flush();
    Serial << "...done\n";
}

uint8_t sendCounter = 0;
uint8_t control     = 0;
uint8_t throttle    = 0;
uint8_t yaw         = 128;
uint8_t pitch       = 128;
uint8_t roll        = 128;

const unsigned int kThrottleMin = 1000;
const unsigned int kThrottleMax = 2000;

void loop()
{
    pitchPID.Compute();

    diff = millis()-timer;
    dt = diff*0.001; //delta in seconds

    if (diff >= 3) {
        timer = millis();
        imuUpdate();
        updateMotors();
        sendCounter++;
    }
    
    receiveData();

    if (sendCounter == 15) {
        sendCounter = 0;
        sendData();
    }
}

inline void imuUpdate()
{
    //get sensor data
    getAccel();
    getGyro();

    //run IMU calc
    imu.IMUupdate();
    imu.GetEuler();
}


int byteIndex = 0;
int8_t data[5];
void receiveData()
{
    while (SpiSerial.available()) {
        byte thisByte = SpiSerial.read();
        data[byteIndex++] = thisByte;

        if (byteIndex == 5) {
            byteIndex = 0;

            throttle = data[0];
            yaw = data[1];
            pitch = data[2];
            roll = data[3];
            control = data[4];
            setPitch = (double)map(pitch, 0, 255, -45, 45);

            if (control == '/') {
                Serial << "Disarming\n";
                disarmMotors();
            } else if (control == '^') {
                Serial << "Arming\n";
                armMotors();
            }
        }
    }
}

char imuRollBuffer[8];
char imuPitchBuffer[8];
char imuYawBuffer[8];
char throttleBuffer[8];
char yawBuffer[8];
char pitchBuffer[8];
char rollBuffer[8];
void sendData()
{
    dtostrf(imu.roll, 4, 1, imuRollBuffer);
    dtostrf(imu.pitch, 4, 1, imuPitchBuffer);
    dtostrf(imu.yaw, 4, 1, imuYawBuffer);
    sprintf(throttleBuffer, "/st%d", throttle);
    sprintf(yawBuffer, "/sy%d", yaw);
    sprintf(pitchBuffer, "/sp%d", pitch);
    sprintf(rollBuffer, "/sr%d", roll);

    SpiSerial.write("/ir"); SpiSerial.write(imuRollBuffer);
    SpiSerial.write("/ip"); SpiSerial.write(imuPitchBuffer);
    SpiSerial.write("/iy"); SpiSerial.write(imuYawBuffer);
    SpiSerial.write(throttleBuffer);
    SpiSerial.write(yawBuffer);
    SpiSerial.write(pitchBuffer);
    SpiSerial.write(rollBuffer);
}

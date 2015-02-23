#include <PID_v1.h>
#include <openIMU.h>
#include "MotionPlus.h"
#include "Nunchuck.h"
#include <SPI.h>
#include <WiFly.h>
#include <Wire.h>
#include <Servo.h>

#define RAD(x) ((x) * 0.0174532925)

//Pin constants
const int kMotionPlusPin = 2;
const int kNunchukPin = 4;
const int kMotorPin1 = 9;
const int kMotorPin2 = 6;
const int kMotorPin3 = 3;
const int kMotorPin4 = 5;

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

WiFlyServer server(80);
char ssid[] = "DRONE";
unsigned int bytesAvailable = 0;

//Gyro
MotionPlus wmp = MotionPlus();

//Accel
Nunchuck nunchuck = Nunchuck();

//IMU
openIMU imu(&gyroX, &gyroY, &gyroZ, &accelX, &accelY, &accelZ, &dt);

//PID
double set_p;
double out_p;

double set_r;
double out_r;

PID pitchPID((double*)&imu.pitch, &out_p, &set_p, 2.15, 0.03, 0.56, DIRECT);
PID  rollPID((double*)&imu.roll,  &out_r, &set_r, 0.0, 0.0, 0.0, DIRECT);

//Motors
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

int motor1value = 1000;
int motor2value = 1000;
int motor3value = 1000;
int motor4value = 1000;

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
    SpiSerial.begin();
    
    Serial.println("Starting WiFi");
    WiFly.begin(true);
    server.begin();
    SpiSerial.begin();
    
    if (!WiFly.createAdHocNetwork(ssid)) {
        while (1) {} //bad things have happened.
    }
    
    server.begin();
   
    Serial.print("WiFi started");
    
    pinMode(13,OUTPUT);
    ledOn();
    initMotors();
    TWBR = ((F_CPU / 4000000) - 16) / 2;

    pinMode(kMotionPlusPin, OUTPUT);
    pinMode(kNunchukPin, OUTPUT);

    digitalWrite(kNunchukPin, HIGH);
    digitalWrite(kMotionPlusPin, HIGH);

    switchWmp();
    wmp.init(500);

    switchNunchuck();
    nunchuck.init(10);

    timer = millis();

    pitchPID.SetMode(AUTOMATIC);
    pitchPID.SetOutputLimits(-200,200);
    pitchPID.SetSampleTime(3);

    rollPID.SetMode(AUTOMATIC);
    rollPID.SetOutputLimits(-1000,1000);
    rollPID.SetSampleTime(3);

    delay(2000);
    ledOff();
}

unsigned int warmup      = 1500;
unsigned int sendCounter = 0;
unsigned int throttle    = 1000;

const unsigned int kThrottleMin = 1000;
const unsigned int kThrottleMax = 1800;

void loop()
{    
    receiveData();
    pitchPID.Compute();
    diff = millis()-timer;
    dt = diff*0.001; //delta in seconds

    if (diff >= 3) {
        timer = millis();
    
        //get sensor data
        getAccel();
        getGyro();
    
        //run IMU calc
        imu.IMUupdate();
        imu.GetEuler();
        
        
        updateMotors();
    }
}

void receiveData()
{
    while (SpiSerial.available() > 0) {
        char data = SpiSerial.read();
        Serial.print(data);
        SpiSerial.write(data);
    }
}


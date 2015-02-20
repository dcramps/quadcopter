#include <PID_v1.h>
#include <openIMU.h>
#include "MotionPlus.h"
#include "Nunchuck.h"
#include <SPI.h>
//#include <WiFly.h>
#include <Wire.h>
#include <Servo.h>

#define WIFLY 0 //0 = off, 1 = client, 2 = server
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

////WiFly stuff
//#if WIFLY == 1
////Web Client - Send gyro data to Processing server
//char passphrase[] = "6476291353";
//char ssid[] = "robot";
//byte server[] = { 192, 168, 1, 6 };
//WiFlyClient client(server, 8000);
//#elsif WIFLY == 2
////start a server on port 80
//WiFlyServer server(80);
//char ssid[] = "DRONE";
//unsigned int bytesAvailable = 0;
//#endif

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
    pinMode(13,OUTPUT);
    ledOn();
    initMotors();
    Serial.begin(115200);
    TWBR = ((F_CPU / 4000000) - 16) / 2;

    pinMode(kMotionPlusPin, OUTPUT);
    pinMode(kNunchukPin, OUTPUT);

    digitalWrite(kNunchukPin, HIGH);
    digitalWrite(kMotionPlusPin, HIGH);

    switchWmp();
    wmp.init(500);

    switchNunchuck();
    nunchuck.init(10);

    //#if WIFLY == 1
    //    // Client
    //    WiFly.begin();
    //
    //    if (!WiFly.join(ssid, passphrase)) {
    //        while (1) {
    //        }
    //    }
    //#elsif WIFLY == 2
    //    WiFly.begin(true);
    //    server.begin();
    //
    //    if (!WiFly.createAdHocNetwork(ssid)) {
    //        while (1) {
    //        } //bad things have happened.
    //    }
    //
    //    server.begin();
    //#endif

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
    if (warmup==0) {
        pitchPID.SetMode(AUTOMATIC);
    } else {
        pitchPID.SetMode(MANUAL);
    }
    
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

       if (warmup==0) {
            //run PID calc
            //rollPID.Compute();
            updateMotors();

            if (sendCounter>100) {
                //processing
                SerialSend();
                SerialReceive();
                sendCounter = 0;
            }
            sendCounter++;
       } else {
            warmup--;
        }
    }
}

/********************************************
 * Serial Communication functions / helpers
 ********************************************/


union {
    byte asBytes[24];
    float asFloat[6];
}
data;



//Byte listing
//0     - kill (if 1, shut down)
//1-4   - setPoint X axis
//5-8   - setPoint Y axis
//9-12  - P value
//13-16 - I value
//17-20 - D value
//21-24 - Throttle
void SerialReceive()
{

    // read the bytes sent from Processing
    int index=0;
    byte motors = -1;
    while(Serial.available()&&index<=24)
    {
        if(index==0) {
            motors = Serial.read();
        } else {
            data.asBytes[index-1] = Serial.read();
        }
        index++;
    }

    // if the information we got was in the correct format, read it into the system
    if(index==25 && (motors==0 || motors==1))
    {
        double p, i, d;
        set_p    = double(data.asFloat[0]);
        set_r    = double(data.asFloat[1]);
        p        = double(data.asFloat[2]);
        i        = double(data.asFloat[3]);
        d        = double(data.asFloat[4]);
        throttle = double(data.asFloat[5]);
        
        pitchPID.SetTunings(p, i, d);
        
        if (motors==1) {
            ledOn();
            motorsEnabled = true;
        } else {
            ledOff();
            motorsEnabled = false;
        }
    }
    Serial.flush(); //clear any random data from the serial buffer
}

//Data sent:
//set pitch
//set roll
//IMU pitch
//IMU roll
//P value
//I value
//D value
//Throttle
//Motor 1
//Motor 2
//Motor 3
//Motor 4
void SerialSend()
{
    Serial.print(set_p);   
    Serial.print(" ");
    Serial.print(set_r);
    Serial.print(" ");
    Serial.print((double)imu.pitch);
    Serial.print(" ");
    Serial.print((double)imu.roll);
    Serial.print(" ");
    Serial.print(pitchPID.GetKp());   
    Serial.print(" ");
    Serial.print(pitchPID.GetKi());   
    Serial.print(" ");
    Serial.print(pitchPID.GetKd());
    Serial.print(" ");
    Serial.print(throttle);   
    Serial.print(" ");
    Serial.print(motor1value);
    Serial.print(" ");
    Serial.print(motor2value);
    Serial.print(" ");
    Serial.print(motor3value);
    Serial.print(" ");
    Serial.println(motor4value);
}






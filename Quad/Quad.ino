#include <openIMU.h>
#include <PID_AutoTune_v0.h>
#include "MotionPlus.h"
#include "Nunchuck.h"
#include <SPI.h>
#include <WiFly.h>
#include <Wire.h>
#include <math.h>
#include <Servo.h>
#include <PID_v1.h>


#define WIFLY 0 //1 = client, 2 = server

#define RAD(x) ((x) * 0.0174532925)

float accelX = 0;
float accelY = 0;
float accelZ = 0;
float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;

/* Motors */
//#define MOTOR_1_PIN 3
//#define MOTOR_2_PIN 9
//#define MOTOR_3_PIN 5
//#define MOTOR_4_PIN 6
//Servo m1;
//Servo m2;
//Servo m3;
//Servo m4;

/* Constants 
 * Values derived from the datasheet
 * http://html.alldatasheet.com/html-pdf/171987/STMICROELECTRONICS/LIS3L02AL/9754/5/LIS3L02AL.html
 */
const float Vdd = 3.3;
const float Voff = Vdd/2.0f + (Vdd/2.0f)*(0.0015); //Vdd/2 +/-6%
const float So = Vdd/5.0f; //Vdd/5 +/-10%

/*
 * Pre-calculate some more stuff. Division is expensive, so
 * I don't want to waste cycles on it needlessly for constant
 * values.
 */
const float SoInv = 1/So;
const float radToDeg = 180/M_PI;
const float VddStep = Vdd/1023;

/* loop stuff */
long timer;       //millis at start of calculation
int diff;        //difference between now and last cycle
float dt;         //delta in seconds
int calc = 0;     //number of calculations since last data transmit

/* Wii Motion + */
#define MOTIONPLUS 2
MotionPlus wmp = MotionPlus();

/* Nunchuck */
#define NUNCHUCK 4
Nunchuck nunchuck = Nunchuck();

/* Output data */
//double pitch_angle = 0.0;
//double roll_angle = 0.0;
//double yaw_angle = 0.0;

#if WIFLY == 1
    /* Web Client - Send gyro data to Processing server */
    char passphrase[] = "6476291353";
    char ssid[] = "robot";
    byte server[] = { 192, 168, 1, 6 };
    WiFlyClient client(server, 8000);
#elsif WIFLY == 2
    /* start a server on port 80 */
    WiFlyServer server(80);
    char ssid[] = "DRONE";
    unsigned int bytesAvailable = 0;
#endif

/*PID things*/
//const int maxThrottle = 1600; //for now never changing because no input
//const int minThrottle = 1200; //motors turn on around here?
//double setPoint = 0; //for now never changing because no input

//double consKp = 4.5;
//double consKi = 0.2;
//double consKd = 0.15;

//double roll_output;

//int throttle1, throttle2, throttle3, throttle4;

//PID rollPID(&roll_angle, &roll_output, &setPoint, consKp, consKi, consKd, DIRECT);
//PID pitchPID;
//PID yawPID;

/* OpenIMU */
openIMU imu(&gyroX,&gyroY,&gyroZ,&accelX,&accelY,&accelZ,&dt);

/*
 * WMP and Nunchuck communicate on the same I²C address
 * so use two NPN transistors to switch the data line
 * between them as needed.
 *
 * Setting a line to HIGH enables it, LOW disables it.
 */
void switchWmp() 
{
    digitalWrite(NUNCHUCK,   LOW);
    digitalWrite(MOTIONPLUS, LOW);
    digitalWrite(MOTIONPLUS, HIGH);    
}

void switchNunchuck()
{
    digitalWrite(MOTIONPLUS, LOW);
    digitalWrite(NUNCHUCK,   LOW);
    digitalWrite(NUNCHUCK,   HIGH);
}

void getAccel()
{
    switchNunchuck();
    nunchuck.update();
    
    accelX =       ((nunchuck.accelX * VddStep) - Voff) * SoInv - 0.05;
    accelY = -1 * (((nunchuck.accelY * VddStep) - Voff) * SoInv - 0.03);
    accelZ = -1 * (((nunchuck.accelZ * VddStep) - Voff) * SoInv);
}

void getGyro()
{
    switchWmp();
    wmp.update();    
    
    gyroX =       RAD(wmp.roll_r);
    gyroY = -1 *  RAD(wmp.pitch_r);
    gyroZ =       RAD(wmp.yaw_r);
}

void setup()
{
    Serial.begin(115200);
    TWBR = ((F_CPU / 4000000) - 16) / 2;
    //Serial.println("SETUP\tStarting...");

    /*Setup Pins*/
    pinMode(MOTIONPLUS, OUTPUT);
    pinMode(NUNCHUCK, OUTPUT);
//    m1.attach(MOTOR_1_PIN);
//    m2.attach(MOTOR_2_PIN);
//    m3.attach(MOTOR_3_PIN);
//    m4.attach(MOTOR_4_PIN);

    /*Setup PIDs*/
//    rollPID.SetMode(AUTOMATIC);
//    rollPID.SetSampleTime(10);
//    rollPID.SetOutputLimits(-200, 200);

    /*Necessary?*/
    digitalWrite(NUNCHUCK, HIGH);
    digitalWrite(MOTIONPLUS, HIGH);

    /*Init MotionPlus*/
    switchWmp();
    wmp.init();
    Serial.println("Gyro enabled");

    /*Init Nunchuck*/
    switchNunchuck();
    nunchuck.init();
    Serial.println("Accel enabled");

    #if WIFLY == 1
        // Client
        Serial.println("SETUP\tStarting WiFly");
        WiFly.begin();
          
        if (!WiFly.join(ssid, passphrase)) {
            //Serial.println("CLIENT\tAssociation failed.");
            while (1) { }
        }
          
        Serial.println("CLIENT\tConnecting.");
        if (client.connect()) {
            Serial.println("CLIENT\tConnected.");
        } else {
            Serial.println("CLIENT\tConnection failed.");
        }
    #elsif WIFLY == 2
        WiFly.begin(true);
        server.begin();
        
        if (!WiFly.createAdHocNetwork(ssid)) {
            while (1) { 
            } //bad things have happened.
        }
    
        Serial.print("Network is up at ");
        Serial.println(WiFly.ip());
    
        Serial.println("Starting the server");
        server.begin();
    #endif

    timer = millis();
    Serial.println("Starting loop");
}

void loop()
{
    diff = millis()-timer;
    dt = diff*0.001; //delta in seconds

    if (diff >= 3) {
        calc++;
        timer = millis();
        getAccel();
        getGyro();
        imu.IMUupdate();
    }
    
    if (calc==2) {
        calc=0;
        imu.GetEuler();
        Serial.print(accelX,3);
        comma();
        Serial.print(accelY,3);
        comma();
        Serial.print(accelZ,3);
        comma();
        Serial.print(gyroX,3);
        comma();
        Serial.print(gyroY,3);
        comma();
        Serial.print(imu.roll,3);
        comma();
        Serial.println(imu.pitch,3);
    }

//    //    if (bytesAvailable %12 == 0 && bytesAvailable > 0) {
//    //        Serial.println(bytesAvailable);
//    //        bytesAvailable=0;
//    //    }
//    //
//    //    if (bytesAvailable > 0) {
//    //        Serial.println(bytesAvailable);
//    //    }
//    //    delay(1);
//
//
//    if (calc==10) {
//        if (client.connected()) {
//            //yaw
//            client.write((byte)((int16_t)yaw_angle >> 8));
//            client.write((byte)((int16_t)yaw_angle));
//
//            //pitch
//            client.write((byte)((int16_t)pitch_angle >> 8));
//            client.write((byte)((int16_t)pitch_angle));
//
//            //roll
//            client.write((byte)((int16_t)roll_angle >> 8));
//            client.write((byte)((int16_t)roll_angle));     
//        }
//        calc=0;
//    }


}

void tab()
{
    Serial.print("\t");
}

void comma()
{
    Serial.print(",");
}



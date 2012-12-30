#include "Credentials.h"
#include "MotionPlus.h"
#include "Nunchuck.h"
#include <SPI.h>
#include <WiFly.h>
#include <Wire.h>
#include <math.h>
#include <Servo.h>


/* Motors */
#define MOTOR_1_PIN 3
#define MOTOR_2_PIN 9
#define MOTOR_3_PIN 5
#define MOTOR_4_PIN 6
Servo m1;
Servo m2;
Servo m3;
Servo m4;

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
float Rx, Ry, Rz; //unconverted acc values
float ax, ay;     //accelerometer values in degrees
long timer;       //millis at start of calculation
long diff;        //difference between now and last cycle
float dt;         //delta in seconds
int calc = 0;     //number of calculations since last data transmit

/* Wii Motion + */
#define MOTIONPLUS 2
MotionPlus wmp = MotionPlus();

/* Nunchuck */
#define NUNCHUCK 4
Nunchuck nunchuck = Nunchuck();

/* Output data */
float pitch_angle = 0.0;
float roll_angle = 0.0;
float yaw_angle = 0.0;

/* Web Client - Send gyro data to Processing server */
byte server[] = { 192, 168, 1, 3 };
WiFlyClient client(server, 8000);

/*
 * WMP and Nunchuck communicate on the same I²C address
 * so use two NPN transistors to switch the data line
 * between them as needed.
 *
 * Setting a line to HIGH enables it, LOW disables it.
 */
void switchWmp() 
{
    digitalWrite(NUNCHUCK, LOW);
    digitalWrite(MOTIONPLUS, LOW);
    digitalWrite(MOTIONPLUS, HIGH);    
}

void switchNunchuck()
{
    digitalWrite(MOTIONPLUS, LOW);
    digitalWrite(NUNCHUCK, LOW);
    digitalWrite(NUNCHUCK, HIGH);
}

void setup()
{
    Serial.begin(115200);
    //Serial.println("SETUP\tStarting...");

    /*Setup Pins*/
    pinMode(MOTIONPLUS, OUTPUT);
    pinMode(NUNCHUCK, OUTPUT);
    m1.attach(MOTOR_1_PIN);
    m2.attach(MOTOR_2_PIN);
    m3.attach(MOTOR_3_PIN);
    m4.attach(MOTOR_4_PIN);

    /*Necessary?*/
    digitalWrite(NUNCHUCK, HIGH);
    digitalWrite(MOTIONPLUS, HIGH);
     
    /*Init MotionPlus*/
    switchWmp();
    wmp.init();
    //Serial.println("Gyro enabled");
    
    /*Init Nunchuck*/
    switchNunchuck();
    nunchuck.init();
    //Serial.println("Accel enabled");
          
    /* Arm ESCs */
    m1.writeMicroseconds(1000);
    m2.writeMicroseconds(1000);
    m3.writeMicroseconds(1000);
    m4.writeMicroseconds(1000);
    delay(5000);
    
    /* Client */
    //Serial.println("SETUP\tStarting WiFly");
    WiFly.begin();
    
    if (!WiFly.join(ssid, passphrase)) {
        //Serial.println("CLIENT\tAssociation failed.");
        while (1) { }
    }

    //WiFly.configure(WIFLY_BAUD, 38400);    
    //Serial.println("CLIENT\tConnecting.");
    if (client.connect()) {
        //Serial.println("CLIENT\tConnected.");
    } else {
        //Serial.println("CLIENT\tConnection failed.");
    }
    
    timer = millis();
}
int motor=1000;

void loop()
{
    diff = millis()-timer;
    dt = diff*0.001; //delta in seconds
    
    if (diff > 9) {
        calc++;
        timer = millis();
        
        
        /*Gyro*/
        switchWmp();
        wmp.update();
        
        /*Accel*/
        switchNunchuck();
        nunchuck.update();
       
        Rx = ((nunchuck.accelX * VddStep) - Voff) * SoInv - 0.05;
        Ry = ((nunchuck.accelY * VddStep) - Voff) * SoInv - 0.03;
        Rz = ((nunchuck.accelZ * VddStep) - Voff) * SoInv;
        
        ax = atan(Rx / sqrt(pow(Ry,2) + pow(Rz,2))) * radToDeg;
        ay = atan(Ry / sqrt(pow(Rx,2) + pow(Rz,2))) * radToDeg;
                
        pitch_angle = 0.02 * ay + 0.98 * (pitch_angle + wmp.pitch * dt);
        roll_angle  = 0.02 * ax + 0.98 * (roll_angle  + wmp.roll  * dt); //negated due to IMU orientation
        yaw_angle   = yaw_angle + wmp.yaw * dt;

//        float vals[] = {ay, ax, wmp.pitch, wmp.roll, pitch_angle, roll_angle};
//        printVals(vals,sizeof(vals)/sizeof(float),2);
    }
    
    if (calc==10) {
        if (client.connected()) {
            //yaw
            client.write((byte)((int16_t)yaw_angle >> 8));
            client.write((byte)((int16_t)yaw_angle));
        
            //pitch
            client.write((byte)((int16_t)pitch_angle >> 8));
            client.write((byte)((int16_t)pitch_angle));
        
            //roll
            client.write((byte)((int16_t)roll_angle >> 8));
            client.write((byte)(roll_angle));     
        }
        calc=0;
        motor++;
        Serial.println(motor);
        m1.writeMicroseconds(motor);
        m2.writeMicroseconds(motor);
        m3.writeMicroseconds(motor);
        m4.writeMicroseconds(motor);
    }
}
    
//void printVals(float vals[], int idx, int tabidx)
//{   
//    for (int x = 0; x < idx; x++) {
//        if(x%tabidx==0 && x>0) {
//            Serial.print("|"); tab();
//        }
//        Serial.print(vals[x]); tab();
//    }
//    Serial.println();
//}
//
//void tab()
//{
//    Serial.print("\t");
//}

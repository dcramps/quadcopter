#include "Credentials.h"
#include "MotionPlus.h"
#include "Nunchuck.h"
#include <SPI.h>
#include <WiFly.h>
#include <Wire.h>
#include <math.h>

/* Constants */
const float Vdd = 3.3;
const float Voff = Vdd/2.0f + (Vdd/2.0f)*(0.03); //Vdd/2 +/-6%
const float So = Vdd/5.0f; //Vdd/5 +/-10%
const float SoInv = 1/So;
const float radToDeg = 180/M_PI;
const float VddStep = Vdd/1023;

/* loop stuff */
const int r2d = 57.2957795; //radians to de
long timer;                         //millis at start of calculation
long diff;                          //difference between now and last cycle
float dt;                           //delta in seconds
int calc = 0;                       //number of calculations since last data transmit

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
    Serial.println("SETUP\tStarting...");

    /*Setup Pins*/
    pinMode(MOTIONPLUS, OUTPUT);
    pinMode(NUNCHUCK, OUTPUT);

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
          
    /* Client */
    Serial.println("SETUP\tStarting WiFly");
    WiFly.begin();
    
    if (!WiFly.join(ssid, passphrase)) {
        Serial.println("CLIENT\tAssociation failed.");
        while (1) { }
    }

    //WiFly.configure(WIFLY_BAUD, 38400);    
    Serial.println("CLIENT\tConnecting.");
    if (client.connect()) {
        Serial.println("CLIENT\tConnected.");
    } else {
        Serial.println("CLIENT\tConnection failed.");
    }
    
    timer = millis();
}

float Rx, Ry, Rz, ax, ay, az, R;
void loop()
{
    diff = millis()-timer;
    dt = diff*0.001; //delta in seconds
    
    if (diff > 9) {
//        Serial.println(diff);
        calc++;
        timer = millis();
        
        
        /*Gyro*/
        switchWmp();
        wmp.update();
        
        /*Accel*/
        switchNunchuck();
        nunchuck.update();
       
        Rx = ((nunchuck.accelX * VddStep) - Voff) * SoInv;
        Ry = ((nunchuck.accelY * VddStep) - Voff) * SoInv;
        Rz = ((nunchuck.accelZ * VddStep) - Voff) * SoInv;
        
        float ax = -1 * atan(Rx / sqrt(pow(Ry,2) + pow(Rz,2))) * radToDeg;
        float ay = -1 * atan(Ry / sqrt(pow(Rx,2) + pow(Rz,2))) * radToDeg;
        
                            
        /* Using negatives for Gyro since it's installed opposite as it should*/
        pitch_angle = 0.92 * (pitch_angle - wmp.pitch * dt) + 0.08 * ax;
        roll_angle  = 0.92 * (roll_angle  - wmp.roll  * dt) + 0.08 * ay;
        yaw_angle   = yaw_angle + wmp.yaw * dt;
        
        Serial.print((int)yaw_angle); tab();
        Serial.print((int)pitch_angle); tab();
        Serial.print((int)roll_angle); tab();
        Serial.println(Rz);
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
    }
}

void tab()
{
    Serial.print("\t");
}

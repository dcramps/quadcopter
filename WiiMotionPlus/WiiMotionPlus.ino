#include "Credentials.h"
#include "MotionPlus.h"
#include "Nunchuck.h"
#include <SPI.h>
#include <WiFly.h>
#include <Wire.h>
#include <math.h>

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

#define ZEROX 525
#define ZEROY 516
#define ZEROZ 306


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

    delay(1000);
    timer = millis();
}

float pitch_angle = 0.0;
float roll_angle = 0.0;
float yaw_angle = 0.0;

void loop()
{
    diff = millis()-timer;
    dt = diff*0.001; //delta in seconds
    
    if (diff > 9) {
        //Serial.println(diff);
        calc++;
        timer = millis();
        
        
        /*Gyro*/
        switchWmp();
        wmp.update();
        
        /*Accel*/
        switchNunchuck();
        nunchuck.update();
        float rx = nunchuck.accelX - ZEROX;
        float ry = nunchuck.accelY - ZEROY;
        float rz = nunchuck.accelZ - ZEROZ;
        
        float ax = rx*0.001619;
        float ay = ry*0.001615;
        float az = rz*0.001599;
        
        float x_angle = (atan(ax/sqrt(pow(ay,2)+pow(az,2)))*r2d)*2;
        float y_angle = (atan(ay/sqrt(pow(ax,2)+pow(az,2)))*r2d)*2;
        float z_angle = atan(sqrt(pow(ay,2)+pow(ax,2))/az)*r2d;
            
        /* Do math! Using negatives for Gyro since it's installed opposite as it should*/
        pitch_angle = 0.95 * (pitch_angle - wmp.pitch * dt) + 0.05 * x_angle;
        roll_angle  = 0.95 * (roll_angle  - wmp.roll  * dt) + 0.05 * y_angle;
        yaw_angle = yaw_angle + wmp.yaw * dt; //this is so shitty.
        //yaw_angle = 0; //so ignore for now
        
        Serial.print(yaw_angle); printTab();
        Serial.print(pitch_angle); printTab();
        Serial.println(roll_angle);
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

void printTab()
{
    Serial.print("\t");
}

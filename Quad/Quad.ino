#include <PID_v1.h>
#include <openIMU.h>
#include "MotionPlus.h"
#include "Nunchuck.h"
#include <SPI.h>
#include <WiFly.h>
#include <Wire.h>
#include <Servo.h>

#define WIFLY 0 //0 = off, 1 = client, 2 = server

#define RAD(x) ((x) * 0.0174532925)

float accelX = 0;
float accelY = 0;
float accelZ = 0;
float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;

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
long  timer = 0; //millis at start of calculation
int   diff  = 0; //difference between now and last cycle
float dt    = 0; //delta in seconds
int   calc  = 0; //number of calculations since last data transmit

/* Wii Motion + */
#define MOTIONPLUS 2
MotionPlus wmp = MotionPlus();

/* Nunchuck */
#define NUNCHUCK 4
Nunchuck nunchuck = Nunchuck();

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

    accelX = -1 * (((nunchuck.accelX * VddStep) - Voff) * SoInv - 0.05);
    accelY =       ((nunchuck.accelY * VddStep) - Voff) * SoInv - 0.03;
    accelZ = -1 * (((nunchuck.accelZ * VddStep) - Voff) * SoInv);
}

void getGyro()
{
    switchWmp();
    wmp.update();    

    gyroX = -1 *  RAD(wmp.roll_r);
    gyroY = -1 *  RAD(wmp.pitch_r);
    gyroZ =       RAD(wmp.yaw_r);
}

//PID
double set; //where we want to be
double out; //output from PID (motor adjustment value)

double p = 0.04;
double i = 0.01;
double d = 0.00;

PID pitchPID((double*)&imu.pitch, &out, &set, 0.04,0.01,0.00, DIRECT);

//MOTORS
#define MOTOR_1_PIN 9
#define MOTOR_2_PIN 6
#define MOTOR_3_PIN 5
#define MOTOR_4_PIN 3

#define MOTOR_1 1
#define MOTOR_2 2
#define MOTOR_3 3
#define MOTOR_4 4

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

void setup()
{
    Serial.begin(115200);
    delay(500);
    TWBR = ((F_CPU / 4000000) - 16) / 2;
//    Serial.println("SETUP\tStarting...");

    /*Setup Pins*/
    pinMode(MOTIONPLUS, OUTPUT);
    pinMode(NUNCHUCK, OUTPUT);

    /*Necessary?*/
    digitalWrite(NUNCHUCK, HIGH);
    digitalWrite(MOTIONPLUS, HIGH);

    /*Init MotionPlus*/
    switchWmp();
    wmp.init();
//    Serial.println("Gyro enabled");

    /*Init Nunchuck*/
    switchNunchuck();
    nunchuck.init();
//    Serial.println("Accel enabled");

#if WIFLY == 1
    // Client
    Serial.println("SETUP\tStarting WiFly");
    WiFly.begin();

    if (!WiFly.join(ssid, passphrase)) {
        //Serial.println("CLIENT\tAssociation failed.");
        while (1) { 
        }
    }

    Serial.println("CLIENT\tConnecting.");
    if (client.connect()) {
        Serial.println("CLIENT\tConnected.");
    } 
    else {
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
    pitchPID.SetMode(AUTOMATIC);
    pitchPID.SetOutputLimits(-1000,1000);
    pitchPID.SetSampleTime(3);
//    Serial.println("Starting loop");

      motor1.attach(MOTOR_1_PIN);
      motor2.attach(MOTOR_2_PIN);
      motor3.attach(MOTOR_3_PIN);
      motor4.attach(MOTOR_4_PIN);
      
      motor1.writeMicroseconds(1000);
      motor2.writeMicroseconds(1000);
      motor3.writeMicroseconds(1000);
      motor4.writeMicroseconds(1000);
}

const int throttleMin = 1300;

void loop()
{
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
        
        //run PID calc
//        in = imu.pitch;
        pitchPID.Compute();
        SerialSend();
        SerialReceive();
        
        motor1.writeMicroseconds(constrain(throttleMin+out,1000,2000));
        motor3.writeMicroseconds(constrain(throttleMin-out,1000,2000));        
    }
}

void tab()
{
    Serial.print("\t");
}

void comma()
{
    Serial.print(",");
}

void endl()
{
    Serial.println();
}

/********************************************
 * Serial Communication functions / helpers
 ********************************************/


union {                // This Data structure lets
  byte asBytes[24];    // us take the byte array
  float asFloat[6];    // sent from processing and
}                      // easily convert it to a
foo;                   // float array



// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats

//  the bytes coming from the arduino follow the following
//  format:
//  0: 0=Manual, 1=Auto, else = ? error ?
//  1: 0=Direct, 1=Reverse, else = ? error ?
//  2-5: float setpoint
//  6-9: float input
//  10-13: float output  
//  14-17: float P_Param
//  18-21: float I_Param
//  22-245: float D_Param
void SerialReceive()
{

  // read the bytes sent from Processing
  int index=0;
  byte Auto_Man = -1;
  byte Direct_Reverse = -1;
  while(Serial.available()&&index<26)
  {
    if(index==0) Auto_Man = Serial.read();
    else if(index==1) Direct_Reverse = Serial.read();
    else foo.asBytes[index-2] = Serial.read();
    index++;
  } 
  
  // if the information we got was in the correct format, 
  // read it into the system
  if(index==26  && (Auto_Man==0 || Auto_Man==1)&& (Direct_Reverse==0 || Direct_Reverse==1))
  {
    set=double(foo.asFloat[0]);
    //Input=double(foo.asFloat[1]);       // * the user has the ability to send the 
                                          //   value of "Input"  in most cases (as 
                                          //   in this one) this is not needed.
    if(Auto_Man==0)                       // * only change the output if we are in 
    {                                     //   manual mode.  otherwise we'll get an
      out=double(foo.asFloat[2]);      //   output blip, then the controller will 
    }                                     //   overwrite.
    
    double p, i, d;                       // * read in and set the controller tunings
    p = double(foo.asFloat[3]);           //
    i = double(foo.asFloat[4]);           //
    d = double(foo.asFloat[5]);           //
    pitchPID.SetTunings(p, i, d);            //
    
    if(Auto_Man==0) pitchPID.SetMode(MANUAL);// * set the controller mode
    else pitchPID.SetMode(AUTOMATIC);             //
    
    if(Direct_Reverse==0) pitchPID.SetControllerDirection(DIRECT);// * set the controller Direction
    else pitchPID.SetControllerDirection(REVERSE);          //
  }
  Serial.flush();                         // * clear any random data from the serial buffer
}

// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?
void SerialSend()
{
  Serial.print("PID ");
  Serial.print(set);   
  Serial.print(" ");
  Serial.print((double)imu.pitch);   
  Serial.print(" ");
  Serial.print(out);   
  Serial.print(" ");
  Serial.print(pitchPID.GetKp());   
  Serial.print(" ");
  Serial.print(pitchPID.GetKi());   
  Serial.print(" ");
  Serial.print(pitchPID.GetKd());   
  Serial.print(" ");
  if(pitchPID.GetMode()==AUTOMATIC) Serial.print("Automatic");
  else Serial.print("Manual");  
  Serial.print(" ");
  if(pitchPID.GetDirection()==DIRECT) Serial.println("Direct");
  else Serial.println("Reverse");
}



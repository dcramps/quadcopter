#include "WiFly.h"
#include <Servo.h>

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

Server server(80);

void setup() 
{  
  motor1.attach(MOTOR_1_PIN);
  motor2.attach(MOTOR_2_PIN);
  motor3.attach(MOTOR_3_PIN);
  motor4.attach(MOTOR_4_PIN);
  
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
  
  WiFly.begin(); //I edited the library code

// I don't even know what the fuck this does or how it got here
  pinMode(2,OUTPUT);
  server.begin();
}

int freeRam()
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

void loop() 
{
  Client client = server.available();
  if (client) {
    String clientMsg ="";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        clientMsg+=c;
        if (c == ';') {
          setMotorSpeed(clientMsg);
          clientMsg="";
        }
      }
    }
    // give the Client time to receive the data
    delay(1);
  }
}

/*
 * clientMsg is in form 1_xxx and this is horribly inefficient
 * but that's okay because it's just for testing :]
 */
void setMotorSpeed(String clientMsg)
{
  int motorNum, speedValue;
  
  char buf[clientMsg.length()];
  clientMsg.toCharArray(buf,clientMsg.length());

  char motorBuf[1];
  motorNum = atoi(buf);

  char speedBuf[3];
  speedBuf[0] = buf[2];
  speedBuf[1] = buf[3];
  speedBuf[2] = buf[4];
  speedBuf[3] = buf[5];
  speedValue = atoi(speedBuf);

  switch (motorNum) {
    case MOTOR_1:
      motor1.writeMicroseconds(speedValue);
      break;
    case MOTOR_2:
      motor2.writeMicroseconds(speedValue);
      break;
    case MOTOR_3:
      motor3.writeMicroseconds(speedValue);
      break;
    case MOTOR_4:
      motor4.writeMicroseconds(speedValue);
      break;
    default:
      break;
  }
}

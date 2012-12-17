#include "Credentials.h"
#include <SPI.h>
#include <WiFly.h>
#include <Wire.h>

/* Wii Motion + */
byte data[6];                        //data bytes from device
int yaw, pitch, roll;                //each axis
int yaw0, pitch0, roll0;             //calibration zeroes
bool slowYaw, slowPitch, slowRoll;   //slow and fast mode support
int yawScale, pitchScale, rollScale; //scale factor to use based on slow/fast mode
const int slowScale = 20;            //slow scale factor
const int fastScale = 5;             //fast scale factor

/* Web Client - Send gyro data to Processing server */
byte server[] = { 192, 168, 1, 3 };
WiFlyClient client(server, 8000);

void wmpOn(){
  Wire.beginTransmission(0x53);    //WM+ starts out deactivated at address 0x53
  Wire.write(0xfe);                //send 0x04 to address 0xFE to activate WM+
  Wire.write(0x04);
  Wire.endTransmission();          //WM+ jumps to address 0x52 and is now active
}

void wmpSendZero(){
  Wire.beginTransmission(0x52);    //now at address 0x52
  Wire.write(0x00);                //send zero to signal we want info
  Wire.endTransmission();
}

void calibrateZeroes(){
  for (int i=0;i<10;i++){
    wmpSendZero();
    Wire.requestFrom(0x52,6);
    data[0]=Wire.read();
    data[1]=Wire.read();
    data[2]=Wire.read();
    data[3]=Wire.read();
    data[4]=Wire.read();
    data[5]=Wire.read();
  
    //average 10 readings for each zero
    yaw0+=(((data[3]>>2)<<8)+data[0])/10;
    pitch0+=(((data[4]>>2)<<8)+data[1])/10;
    roll0+=(((data[5]>>2)<<8)+data[2])/10;
  }
//  Serial.print("Yaw0:");
//  Serial.print(yaw0);
//  Serial.print("  Pitch0:");
//  Serial.print(pitch0);
//  Serial.print("  Roll0:");
//  Serial.println(roll0);
}

void receiveData(){
  wmpSendZero();                   //send zero before each request (same as nunchuck)
  Wire.requestFrom(0x52,6);        //request the six bytes from the WM+
  data[0]=Wire.read();
  data[1]=Wire.read();
  data[2]=Wire.read();
  data[3]=Wire.read();
  data[4]=Wire.read();
  data[5]=Wire.read();
  
  
  //check if in slow or fast mode
  slowPitch = data[3] & 1;
  slowYaw = data[3] & 2;
  slowRoll = data[4] & 2;
  
  if (slowYaw) {
    yawScale = slowScale; 
  } else {
    yawScale = fastScale;
  }  
  
  if (slowPitch) {
    pitchScale = slowScale; 
  } else {
    pitchScale = fastScale;
  } 
  
  if (slowRoll) {
    rollScale = slowScale; 
  } else {
    rollScale = fastScale;
  } 

  //values as degrees/sec
  yaw=(((data[3]>>2)<<8)+data[0]-yaw0)/yawScale;
  pitch=(((data[4]>>2)<<8)+data[1]-pitch0)/pitchScale;
  roll=(((data[5]>>2)<<8)+data[2]-roll0)/rollScale;   
  
}


long timer;                       //millis at start of calculation
long diff;                        //difference between now and last cycle
float dt;                         //delta in seconds
float y_angle, p_angle, r_angle;  //gyro data as angles (degrees)
int calc = 0;

void setup(){
  Serial.begin(115200);

  /* Gyro */
  Wire.begin();
  wmpOn();
  calibrateZeroes();
  timer = millis();  
  
  /* Client */
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
}

void loop(){
  diff = millis()-timer;
  dt = diff*0.001; //delta in seconds
  if (diff > 10) {
    calc++;
    timer = millis();
    receiveData();
    y_angle = y_angle + (float)yaw * dt;
    p_angle = p_angle + (float)pitch * dt;
    r_angle = r_angle + (float)roll * dt;

//    Serial.print(y_angle);
//    Serial.print("\t");
//    Serial.print(p_angle);
//    Serial.print("\t");
//    Serial.print(r_angle);
//    Serial.println();
  }
  
  if (calc==25) {
    if (client.connected()) {
      //yaw
      client.write((byte)((int16_t)y_angle >> 8));
      client.write((byte)((int16_t)y_angle));
    
      //pitch
      client.write((byte)((int16_t)p_angle >> 8));
      client.write((byte)((int16_t)p_angle));
    
      //roll
      client.write((byte)((int16_t)r_angle >> 8));
      client.write((byte)(r_angle));   
    }
    calc=0;
  }
}

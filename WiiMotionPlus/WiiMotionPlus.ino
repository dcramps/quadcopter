#include <Wire.h>

const int slowScale = 20;
const int fastScale = 5;

byte data[6];          //six data bytes
int yaw, pitch, roll;  //three axes
int yaw0, pitch0, roll0;  //calibration zeroes
bool slowYaw, slowPitch, slowRoll;
int yawScale, pitchScale, rollScale;

void wmpOn(){
  Wire.beginTransmission(0x53);    //WM+ starts out deactivated at address 0x53
  Wire.write(0xfe);                 //send 0x04 to address 0xFE to activate WM+
  Wire.write(0x04);
  Wire.endTransmission();          //WM+ jumps to address 0x52 and is now active
}

void wmpSendZero(){
  Wire.beginTransmission(0x52);    //now at address 0x52
  Wire.write(0x00);                 //send zero to signal we want info
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
  
    yaw0+=(((data[3]>>2)<<8)+data[0])/10;        //average 10 readings for each zero
    pitch0+=(((data[4]>>2)<<8)+data[1])/10;
    roll0+=(((data[5]>>2)<<8)+data[2])/10;
  }
  Serial.print("Yaw0:");
  Serial.print(yaw0);
  Serial.print("  Pitch0:");
  Serial.print(pitch0);
  Serial.print("  Roll0:");
  Serial.println(roll0);
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


long timer;
long diff;
int readings;
float dt; 
float y_angle, p_angle, r_angle;

void setup(){
  Serial.begin(115200);
  Wire.begin();
  wmpOn();
  calibrateZeroes();
  timer = millis();  
  delay(5000);
}

void loop(){
  diff = millis()-timer;
  dt = diff*0.001; //delta in seconds
  if (diff > 10) {
//    readings++;
    timer = millis();
    receiveData();
    y_angle = y_angle + (float)yaw * dt;
    p_angle = p_angle + (float)pitch * dt;
    r_angle = r_angle + (float)roll * dt;
    Serial.print(y_angle);
    Serial.print("\t");
    Serial.print(p_angle);
    Serial.print("\t");
    Serial.print(r_angle);
    Serial.println();
  }
//  if(readings>=1000) {
//   while(1) { } 
//  }
}

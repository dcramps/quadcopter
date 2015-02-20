// Need G4P library
import g4p_controls.*;
import processing.serial.*;
import java.nio.ByteBuffer;

Serial serialPort;
boolean shouldSend;
boolean kill = true;


public void setup(){
  size(640, 580, JAVA2D);
  createGUI();
  customGUI();
  shouldSend = false;
  kill = false;
  
  println(Serial.list());
  serialPort = new Serial(this, Serial.list()[1], 115200);
  serialPort.bufferUntil(13);
}

public void draw(){
  background(230);
  send();
}

// Use this method to add additional statements
// to customise the GUI controls
public void customGUI(){

}

public void send() {
  if (!shouldSend) {
    return;
  } 
  
  Byte motorsOff = (byte)0;
  if (kill) {
    println("Killing motors");
    motorsOff = (byte)0;
    kill = !kill;
  } else {
    println("Not killing motors");
    motorsOff = (byte)1;
  }
  
  shouldSend = false;
  
  //Do sendy stuff here:
  float [] toSend = new float[6];
  
  toSend[0] = setSlider.getValueXF();
  toSend[1] = setSlider.getValueYF();
  toSend[2] = pKnob.getValueF();
  toSend[3] = iKnob.getValueF();
  toSend[4] = dKnob.getValueF();
  toSend[5] = (throttle.getValueF() * 10) + 1000.0f;

  serialPort.write(motorsOff);
  serialPort.write(floatArrayToByteArray(toSend));
}

//Data received from Arduino:
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
void serialEvent(Serial serialPort)
{
  String read = serialPort.readStringUntil(13);

  String[] s = split(read, " ");
  
  if (s.length == 12) {
    println("SetX: " + float(s[0]));
//    setSlider.setValueX(float(s[0]));
    
    println("SetY: " + float(s[1]));
//    setSlider.setValueY(float(s[1]));
    
    println("OutX: " + float(s[2]));
    outputSlider.setValueX(float(s[2]));
    
    println("OutY: " + float(s[3]));
    outputSlider.setValueY(float(s[3]));
    
    println("P: " + float(s[4]));
//    pKnob.setValue(float(s[4]));
//    
    println("I: " + float(s[5]));
//    iKnob.setValue(float(s[5]));
//    
    println("D: " + float(s[6]));
//    dKnob.setValue(float(s[6]));
//    
    println("Throttle: " + float(s[7]));
//    throttle.setValue((float(s[7]) - 1000) / 10);
    //float(s[8])
    //float(s[9])
    //float(s[10])
    //float(s[11])
  }
}

byte[] floatArrayToByteArray(float[] input) {
  int len        = 4*input.length;
  int index      = 0;
  byte[] b       = new byte[4];
  byte[] out     = new byte[len];
  ByteBuffer buf = ByteBuffer.wrap(b);
  
  for(int i=0;i<input.length;i++) {
    buf.position(0);
    buf.putFloat(input[i]);
    for(int j=0;j<4;j++) {
      out[j+i*4]=b[3-j];
    }
  }
  return out;
}

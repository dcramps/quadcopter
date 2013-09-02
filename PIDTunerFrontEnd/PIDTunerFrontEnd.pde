/********************************************************
 * Arduino PID Tuning Front-End,  Version 0.3
 * by Brett Beauregard
 * License: Creative-Commons Attribution Share-Alike
 * April 2011
 *
 * This application is designed to interface with an
 * arduino running the PID Library.  From this Control
 * Panel you can observe & adjust PID performance in 
 * real time
 *
 * The ControlP5 library is required to run this sketch.
 * files and install instructions can be found at
 * http://www.sojamo.de/libraries/controlP5/
 *
 * MODIFIED BY DC TO NOT SUCK SO MUCH
 * 
 ********************************************************/

import java.nio.ByteBuffer;
import processing.serial.*;
import controlP5.*;

/***********************************************
 * User spcification section
 **********************************************/
int windowWidth = 900;     // set the size of the 
int windowHeight = 600;    // form

float InScaleMin = -180;   // set the Y-Axis Min
float InScaleMax = 180;    // and Max for both
float OutScaleMin = -200; // the top and 
float OutScaleMax = 200;  // bottom trends


int windowSpan = 30000;   // number of mS into the past you want to display
int refreshRate = 100;    // how often you want the graph to be reDrawn;

//float displayFactor = 1;    //display Time as Milliseconds
float displayFactor = 1000; //display Time as Seconds
//float displayFactor = 60000;  //display Time as Minutes

String outputFileName = "";   // if you'd like to output data to a file, specify the path here

/***********************************************
 * end user spec
 **********************************************/

int nextRefresh;
int arrayLength = windowSpan / refreshRate+1;
int[] InputData = new int[arrayLength];     //we might not need them this big, but
int[] SetpointData = new int[arrayLength];  // this is worst case
int[] OutputData = new int[arrayLength];


float inputTop     = 25;
float inputHeight  = (windowHeight-70)*2/3;
float outputTop    = inputHeight+50;
float outputHeight = (windowHeight-70)*1/3;

float ioLeft     = 150;
float ioWidth    = windowWidth-ioLeft-50;
float ioRight    = ioLeft+ioWidth;
float pointWidth = (ioWidth)/float(arrayLength-1);

int vertCount    = 10;

int nPoints      = 0;

float Input;
float Setpoint;
float Output;

float lastP;
float lastI;
float lastD;

boolean madeContact   = false;
boolean justSent      = true;
boolean motorsEnabled = false;

Serial serialPort;

ControlP5 controlP5;

controlP5.Textlabel InLabel;
controlP5.Textlabel OutLabel;
controlP5.Textlabel SPLabel;
controlP5.Textlabel PLabel;
controlP5.Textlabel ILabel;
controlP5.Textlabel DLabel;
controlP5.Textlabel SPTitle;
controlP5.Textlabel InTitle;
controlP5.Textlabel OutTitle;

controlP5.Textfield PField;
controlP5.Textfield IField;
controlP5.Textfield DField;

controlP5.Textlabel m1Label;
controlP5.Textlabel m2Label;
controlP5.Textlabel m3Label;
controlP5.Textlabel m4Label;

PrintWriter output;
PFont AxisFont;
PFont TitleFont; 

void setup()
{
  frameRate(30);
  size(windowWidth, windowHeight);

  println(Serial.list());
  serialPort = new Serial(this, Serial.list()[1], 115200);
  serialPort.bufferUntil(13);

  controlP5 = new ControlP5(this);
  SPTitle   = controlP5.addTextlabel("1", "Setpoint", 10, 13);
  InTitle   = controlP5.addTextlabel("2", "Input",    10, 63);
  OutTitle  = controlP5.addTextlabel("3", "Output",   10, 113);
  PField    = controlP5.addTextfield("Kp (Proportional)", 10, 160, 60, 20);
  IField    = controlP5.addTextfield("Ki (Integral)",     10, 210, 60, 20);
  DField    = controlP5.addTextfield("Kd (Derivative)",   10, 260, 60, 20);
  
  SPLabel   = controlP5.addTextlabel("SP",  "Waiting...", 100, 13);
  InLabel   = controlP5.addTextlabel("In",  "Waiting...", 100, 63);
  OutLabel  = controlP5.addTextlabel("Out", "Waiting...", 100, 113);
  PLabel    = controlP5.addTextlabel("P",   "Waiting...", 100, 163);
  ILabel    = controlP5.addTextlabel("I",   "Waiting...", 100, 213);
  DLabel    = controlP5.addTextlabel("D",   "Waiting...", 100, 263);
  
  m1Label   = controlP5.addTextlabel("m1",  "Waiting...", 60,  500);
  m2Label   = controlP5.addTextlabel("m2",  "Waiting...", 35,  525); 
  m3Label   = controlP5.addTextlabel("m3",  "Waiting...", 85,  525);
  m4Label   = controlP5.addTextlabel("m4",  "Waiting...", 60,  550);

  controlP5.addButton("Send",0.0,10,310,120,20);
  controlP5.addButton("ON",0.0,10,340,120,20);
  controlP5.addButton("OFF",0.0,10,370,120,120);
  AxisFont = loadFont("axis.vlw");
  TitleFont = loadFont("Titles.vlw");
 
  nextRefresh=millis();
  if (outputFileName!="") output = createWriter(outputFileName);
}

void draw()
{
  background(200);
  drawGraph();
  drawButtonArea();
}

void drawGraph()
{
  //draw Base, gridlines
  stroke(0);
  fill(230);
  rect(ioLeft, inputTop,ioWidth-1 , inputHeight);
  rect(ioLeft, outputTop, ioWidth-1, outputHeight);
  stroke(210);

  //Section Titles
  textFont(TitleFont);
  fill(255);
  text("PID Input / Setpoint",(int)ioLeft+10,(int)inputTop-5);
  text("PID Output",(int)ioLeft+10,(int)outputTop-5);


  //GridLines and Titles
  textFont(AxisFont);
  
  //horizontal grid lines
  int interval = (int)inputHeight/12;
  for(int i=0;i<13;i++)
  {
    if(i>0&&i<12) line(ioLeft+1,inputTop+i*interval,ioRight-2,inputTop+i*interval);
    text(str((InScaleMax-InScaleMin)/12*(float)(12-i)+InScaleMin),ioRight+5,inputTop+i*interval+4);
  }
  
  interval = (int)outputHeight/6;
  for(int i=0;i<7;i++)
  {
    if(i>0&&i<6) line(ioLeft+1,outputTop+i*interval,ioRight-2,outputTop+i*interval);
    float rounded = Float.parseFloat(String.format("%.2f",(OutScaleMax-OutScaleMin)/6*(float)(6-i)+OutScaleMin));
    text(str(rounded),ioRight+5,outputTop+i*interval+4);
  }


  //vertical grid lines and TimeStamps
  int elapsedTime = millis();
  interval = (int)ioWidth/vertCount;
  int shift = elapsedTime*(int)ioWidth / windowSpan;
  shift %=interval;

  int iTimeInterval = windowSpan/vertCount;
  float firstDisplay = (float)(iTimeInterval*(elapsedTime/iTimeInterval))/displayFactor;
  float timeInterval = (float)(iTimeInterval)/displayFactor;
  for(int i=0;i<vertCount;i++)
  {
    int x = (int)ioRight-shift-2-i*interval;

    line(x,inputTop+1,x,inputTop+inputHeight-1);
    line(x,outputTop+1,x,outputTop+outputHeight-1);    

    float t = firstDisplay-(float)i*timeInterval;
    t = Float.parseFloat(String.format("%.2f", t));
    if(t>=0)  text(str(t),x,outputTop+outputHeight+10);
  }


  // add the latest data to the data Arrays.  the values need
  // to be massaged to get them to graph correctly.  they 
  // need to be scaled to fit where they're going, and 
  // because 0, 0 is the top left, we need to flip the values.
  // this is easier than having the user stand on their head
  // to read the graph.
  if(millis() > nextRefresh && madeContact)
  {
    nextRefresh += refreshRate;

    for(int i=nPoints-1;i>0;i--)
    {
      InputData[i]=InputData[i-1];
      SetpointData[i]=SetpointData[i-1];
      OutputData[i]=OutputData[i-1];
    }
    if (nPoints < arrayLength) nPoints++;

    InputData[0] = int(inputHeight)-int(inputHeight*(Input-InScaleMin)/(InScaleMax-InScaleMin));
    SetpointData[0] =int( inputHeight)-int(inputHeight*(Setpoint-InScaleMin)/(InScaleMax-InScaleMin));
    OutputData[0] = int(outputHeight)-int(outputHeight*(Output-OutScaleMin)/(OutScaleMax-OutScaleMin));
  }
  
  //draw lines for the input, setpoint, and output
  for(int i=0; i<nPoints-2; i++)
  {
    int X1 = int(ioRight-2-float(i)*pointWidth);
    int X2 = int(ioRight-2-float(i+1)*pointWidth);
    boolean y1Above, y1Below, y2Above, y2Below;


    //DRAW THE INPUT
    boolean drawLine=true;
    stroke(255,0,0);
    int Y1 = InputData[i];
    int Y2 = InputData[i+1];

    y1Above = (Y1>inputHeight);                     // if both points are outside 
    y1Below = (Y1<0);                               // the min or max, don't draw the 
    y2Above = (Y2>inputHeight);                     // line.  if only one point is 
    y2Below = (Y2<0);                               // outside constrain it to the limit, 
    if(y1Above)                                     // and leave the other one untouched.
    {                                               //
      if(y2Above) drawLine=false;                   //
      else if(y2Below) {                            //
        Y1 = (int)inputHeight;                      //
        Y2 = 0;                                     //
      }                                             //
      else Y1 = (int)inputHeight;                   //
    }                                               //
    else if(y1Below)                                //
    {                                               //
      if(y2Below) drawLine=false;                   //
      else if(y2Above) {                            //
        Y1 = 0;                                     //
        Y2 = (int)inputHeight;                      //
      }                                             //
      else Y1 = 0;                                  //
    }                                               //
    else                                            //
    {                                               //
      if(y2Below) Y2 = 0;                           //
      else if(y2Above) Y2 = (int)inputHeight;       //
    }                                               //

    if(drawLine)
    {
      strokeWeight(1);
      line(X1,Y1+inputTop, X2, Y2+inputTop);
    }

    //DRAW THE SETPOINT
    drawLine=true;
    stroke(0,255,0);
    Y1 = SetpointData[i];
    Y2 = SetpointData[i+1];

    y1Above = (Y1>(int)inputHeight);                // if both points are outside 
    y1Below = (Y1<0);                               // the min or max, don't draw the 
    y2Above = (Y2>(int)inputHeight);                // line.  if only one point is 
    y2Below = (Y2<0);                               // outside constrain it to the limit, 
    if(y1Above)                                     // and leave the other one untouched.
    {                                               //
      if(y2Above) drawLine=false;                   //
      else if(y2Below) {                            //
        Y1 = (int)(inputHeight);                    //
        Y2 = 0;                                     //
      }                                             //
      else Y1 = (int)(inputHeight);                 //
    }                                               //
    else if(y1Below)                                //
    {                                               //
      if(y2Below) drawLine=false;                   //
      else if(y2Above) {                            //
        Y1 = 0;                                     //
        Y2 = (int)(inputHeight);                    //
      }                                             //
      else Y1 = 0;                                  //
    }                                               //
    else                                            //
    {                                               //
      if(y2Below) Y2 = 0;                           //
      else if(y2Above) Y2 = (int)(inputHeight);     //
    }                                               //

    if(drawLine)
    {
      strokeWeight(0.5);
      line(X1, Y1+inputTop, X2, Y2+inputTop);
    }

    //DRAW THE OUTPUT
    drawLine=true;
    stroke(0,0,255);
    Y1 = OutputData[i];
    Y2 = OutputData[i+1];

    y1Above = (Y1>outputHeight);                   // if both points are outside 
    y1Below = (Y1<0);                              // the min or max, don't draw the 
    y2Above = (Y2>outputHeight);                   // line.  if only one point is 
    y2Below = (Y2<0);                              // outside constrain it to the limit, 
    if(y1Above)                                    // and leave the other one untouched.
    {                                              //
      if(y2Above) drawLine=false;                  //
      else if(y2Below) {                           //
        Y1 = (int)outputHeight;                    //
        Y2 = 0;                                    //
      }                                            //
      else Y1 = (int)outputHeight;                 //
    }                                              //
    else if(y1Below)                               //
    {                                              //
      if(y2Below) drawLine=false;                  //
      else if(y2Above) {                           //
        Y1 = 0;                                    //
        Y2 = (int)outputHeight;                    //
      }                                            //  
      else Y1 = 0;                                 //
    }                                              //
    else                                           //
    {                                              //
      if(y2Below) Y2 = 0;                          //
      else if(y2Above) Y2 = (int)outputHeight;     //
    }                                              //

    if(drawLine)
    {
      strokeWeight(1);
      line(X1, outputTop + Y1, X2, outputTop + Y2);
    }
  }
  strokeWeight(1);
}

void drawButtonArea()
{
  stroke(0);
  fill(100);
  rect(0, 0, ioLeft, windowHeight);
}

// Sending Floating point values to the arduino
// is a huge pain.  if anyone knows an easier
// way please let know.  the way I'm doing it:
// - Take the 6 floats we need to send and
//   put them in a 6 member float array.
// - using the java ByteBuffer class, convert
//   that array to a 24 member byte array
// - send those bytes to the arduino
void Send()
{
  float[] toSend = new float[6];

  toSend[0] = float("0.0");
  toSend[1] = float("0.0");
  toSend[2] = float("0.0");
  toSend[3] = float(PField.getText());
  toSend[4] = float(IField.getText());
  toSend[5] = float(DField.getText());
  Byte motors = (byte)0; //make sure it's false until we set it
  if (motorsEnabled == true) {
    motors = (byte)1;
  } else {
    motors = (byte)0;
  }
  Byte d = (byte)0; //NOT USED
  serialPort.write(motors);
  serialPort.write(d);
  serialPort.write(floatArrayToByteArray(toSend));
} 

void ON()
{
  motorsEnabled = true;
  Send();
}

void OFF()
{
  motorsEnabled = false;
  Send();
}

byte[] floatArrayToByteArray(float[] input)
{
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


//take the string the arduino sends us and parse it
void serialEvent(Serial serialPort)
{
  String read = serialPort.readStringUntil(13);
  
  if(outputFileName!="") {
    output.print(str(millis())+ " "+read);
  }
  
  String[] s = split(read, " ");
  
  if (s.length == 11) {
    Setpoint = float(s[1]);
    Input    = float(s[2]);
    Output   = float(s[3]);
    
    lastP = float(s[4]);
    lastI = float(s[5]);
    lastD = float(s[6]);
    
    SPLabel.setValue(s[1]);
    InLabel.setValue(s[2]);  
    OutLabel.setValue(trim(s[3]));
    PLabel.setValue(trim(s[4]));
    ILabel.setValue(trim(s[5]));
    DLabel.setValue(trim(s[6]));
    
    float f = Float.parseFloat(trim(s[7]));
    float p = (f-1000)/10;
    m1Label.setValue(str(p));
    
    f = Float.parseFloat(trim(s[8]));
    p = (f-1000)/10;
    m2Label.setValue(str(p));
    
    f = Float.parseFloat(trim(s[9]));
    p = (f-1000)/10;
    m3Label.setValue(str(p));
    
    f = Float.parseFloat(trim(s[10]));
    p = (f-1000)/10;
    m4Label.setValue(str(p));
    
    
    if(!madeContact) {
      madeContact=true;
    }
  }
}







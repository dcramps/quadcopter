import processing.net.*;

Server gyroServer;
Client client;
PFont font;
float yaw, pitch, roll;
int frames;

void setup()
{
    size(768, 768, P3D);
    smooth(8);
    frameRate(60);
    yaw = radians(0);
    pitch = radians(0);
    roll = radians(0);
    font = createFont("Arial",12);
    gyroServer = new Server(this, 8000);
}

void draw()
{  
    frames++;
    /*text*/
    background(0);
    fill(255);
    text("Frames: " + frames + "\nYaw: " + (int)degrees(yaw) + "\nPitch: " + (int)degrees(pitch) + "\nRoll: " + (int)degrees(roll),10,20);

    /*red arms*/  
    pushMatrix();
    translate(width/2.0, height/1.25);
    rotateX(pitch);
    rotateZ(-roll);
    rotateY(-yaw);
    fill(255, 0, 0);
    box(350, 25, 25);
    popMatrix();
    
    /*white arms*/
    pushMatrix();
    translate(width/2.0, height/1.25);
    rotateX(pitch);
    rotateZ(-roll);
    rotateY(-yaw);
    fill(255, 255, 255);
    box(25, 24, 350);
    popMatrix();
    
    /*server crap*/
    client = gyroServer.available();
    if(client != null) {
      
      /*the following depends entirely on how you're sending data to the
        server. I use 6 bytes. 2 bytes per axis with some room left over 
        in 1, 3, 5 for some extra bits if needed (empty atm)*/
      if (client.available() % 6 == 0) {
        byte data[] =  new byte[6];
        data[0] = (byte)client.read();
        data[1] = (byte)client.read();
        data[2] = (byte)client.read();
        data[3] = (byte)client.read();
        data[4] = (byte)client.read();
        data[5] = (byte)client.read();
      
        yaw   = radians((data[0] << 8) + (data[1] & 0xFF));
        pitch = radians((data[2] << 8) + (data[3] & 0xFF));
        roll  = radians((data[4] << 8) + (data[5] & 0xFF));
      }
    } else {
      println("No client");
    } 
}

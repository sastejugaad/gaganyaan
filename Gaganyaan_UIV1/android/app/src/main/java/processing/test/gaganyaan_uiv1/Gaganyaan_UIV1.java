package processing.test.gaganyaan_uiv1;

import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import hypermedia.net.UDP; 
import hypermedia.net.*; 
import processing.opengl.*; 
import toxi.geom.*; 
import toxi.processing.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class Gaganyaan_UIV1 extends PApplet {


PShape lander;
//import processing.serial.*;






// NOTE: requires ToxicLibs to be installed in order to run properly.
// 1. Download from http://toxiclibs.org/downloads
// 2. Extract into [userdir]/Processing/libraries
//    (location may be different on Mac/Linux)
// 3. Run and bask in awesomeness

ToxiclibsSupport gfx;

UDP udp;
//Serial port;                         // The serial port
char[] teapotPacket = new char[14];  // InvenSense Teapot packet
int serialCount = 0;                 // current packet byte position
int aligned = 0;
int interval = 0;
float x=0;
float y=0;
float z=0;
float w =0;

//DA - added the variable below to fix byte alignment over the Serial port
boolean gotFirstDataByte = false;

float[] q = new float[4];
Quaternion quat = new Quaternion(1, 0, 0, 0);

float[] gravity = new float[3];
float[] euler = new float[3];
float[] ypr = new float[3];

public void setup() {
    //size(300, 300, OPENGL);
    
      udp = new UDP( this, 6000 ); // change the port number here eg:- 5000,6000 etc.
  udp.listen( true );
    gfx = new ToxiclibsSupport(this);

    // setup lights and antialiasing
    lights();
    
    
    lander = loadShape("gg.obj"); //loads 3D model of the lander
    lander.scale(5);
  
    // display serial port list for debugging/clarity
   // println(Serial.list());

    // get the first available port (use EITHER this OR the specific port code below)
    // DA - changed the port to second one in the list (for my laptop)
    //String portName = Serial.list()[0];
    
    // get a specific serial port (use EITHER this OR the first-available code above)
    //String portName = "COM4";
    
    // open the serial port
    //DA - changed the serial data rate to 38400
    //port = new Serial(this, portName, 115200);
    
    // send single character to trigger DMP init/start
    // (expected by MPU6050_DMP6 example Arduino sketch)
    //port.write('r');
    //DA - I added two more of these because the program seems to hang otherwise
    //port.write('r');
    //port.write('r');
}

public void draw() {
    if (millis() - interval > 1000) {
        // resend single character to trigger DMP init/start
        // in case the MPU is halted/reset while applet is running
       // port.write('r');
        interval = millis();
    }
    
    // black background
   // background(0);
    
    // translate everything to the middle of the viewport
    pushMatrix();
    background(102, 153, 255);
    rect(0, 0, 220, 40, 0, 10, 10, 0);
   // fill(0);
    ambientLight(200, 200, 200);
   // directionalLight(50,50,50, width, -height, width/2);
    
   // directionalLight(153,255,102, 0, 0, -1);
  //directionalLight(palette[1], 0, 1, 0);
 // directionalLight(palette[2], 1, 0, 0);
  directionalLight(200,200,200, 0, 1, 0);
    //spotLight(255, 255, 255, width/2, 960/2, 400, 0, 0, -1, PI/4, 2);
    
    //specularMaterial(200, 200, 200, 50);
   // shininess(80);
    translate(width / 2, height / 2);

    // 3-step rotation from yaw/pitch/roll angles (gimbal lock!)
    // ...and other weirdness I haven't figured out yet
    //rotateY(-ypr[0]);
    //rotateZ(-ypr[1]);
    //rotateX(-ypr[2]);

    // toxiclibs direct angle/axis rotation from quaternion (NO gimbal lock!)
    // (axis order [1, 3, 2] and inversion [-1, +1, +1] is a consequence of
    // different coordinate system orientation assumptions between Processing
    // and InvenSense DMP)
    float[] axis = quat.toAxisAngle();
    rotate(axis[0], axis[1], axis[3], -axis[2]);
    shape(lander); 

    
    popMatrix();
}



public void receive( byte[] data, String ip, int port ) {    
  // get the "real" message =
  data = subset(data, 0, data.length);
  String message = new String( data );
  String[] list = split(message, ',');
  
  w =Float.parseFloat(list[0]);
  x =Float.parseFloat(list[1]);
  y =Float.parseFloat(list[2]);
  z =Float.parseFloat(list[3]);
    
    
      q[0]=w;
                    q[1]=x;
                    q[2]=y;
                    q[3]=z;
                    for (int i = 0; i < 4; i++) if (q[i] >= 2) q[i] = -4 + q[i];
                    quat.set(q[0], q[1], q[2], q[3]);
  //println( "Incoming: \""+calib+message+"\" from "+ip+" on port "+port );
  
 // println( "From: "+ip+"  on port: "+port );
  //println( "Calibration: "+calib);
  //println( "Incoming: "+message );
}
  public void settings() {  fullScreen(P3D);  smooth(); }
}

import processing.serial.*;
import java.awt.datatransfer.*;
import java.awt.Toolkit;
import processing.opengl.*;
import saito.objloader.*;
import g4p_controls.*;

GButton button_forward;
GButton button_backward;
GButton button_turn_right;
GButton button_turn_left;
GButton button_stop;

GButton button_continue;
GButton button_right_curve;
GButton button_left_curve;

Float x = 0.0f;
Float y = 0.0f;
Float heading_gyro = 0.0f;

int iteration  = 0;
Float[] x_history = new Float[100000];
Float[] y_history = new Float[100000];

Float zoom = 500.0f;
Float offset_x = 500.0f;
Float offset_y = 500.0f;

PFont font;

// Serial port state.
Serial       port;

void handleButtonEvents(GButton button, GEvent event)
{
  byte buffer[] = new byte[6];
  buffer[0] = 255-256;
  buffer[1] = 0; // 00 stop
  buffer[2] = 00; // x (cm)
  buffer[3] = 0; // y (cm)
  buffer[4] = 0; // w (°)
  buffer[5] = 0; // checksum
  if (button == button_forward && event == GEvent.CLICKED)
  {
    buffer[1] = 1; // 01 forward
    buffer[2] = 30; // x (cm)
  }
  else if (button == button_backward && event == GEvent.CLICKED)
  {
    buffer[1] = 2; // 02 backward
    buffer[2] = 30; // x (cm)
  }
  else if (button == button_turn_right && event == GEvent.CLICKED)
  {
    buffer[1] = 3; // 03 right
    buffer[4] = 90; // w (°)
  }
  else if (button == button_turn_left && event == GEvent.CLICKED)
  {
    buffer[1] = 4; // 04 left
    buffer[4] = 90; // w (°)
  }
  else if (button == button_continue && event == GEvent.CLICKED)
  {
    buffer[1] = 11;
    buffer[2] = 30;
  }  
  else if (button == button_right_curve && event == GEvent.CLICKED)
  {
    buffer[1] = 13;
    buffer[4] = 90;
  }  
  else if (button == button_left_curve && event == GEvent.CLICKED)
  {
    buffer[1] = 14;
    buffer[4] = 30;
  }  
  else if (button == button_stop && event == GEvent.CLICKED)
  {

  }
  for(int i=1; i<5; ++i)
    buffer[5] += buffer[i];
  port.write(buffer);
}
  
void setup()
{
  size(1000, 1000, OPENGL);
  frameRate(30);
  button_continue = new GButton(this, 90, 10, 80, 30, "Continue");
  button_right_curve = new GButton(this, 170, 10, 80, 30, "RightCurve");
  button_left_curve = new GButton(this, 10, 10, 80, 30, "LeftCurve");
  button_forward = new GButton(this, 90, 40, 80, 30, "Forward");
  button_backward = new GButton(this, 90, 100, 80, 30, "Backward");
  button_turn_right = new GButton(this, 170, 70, 80, 30, "Right Turn");
  button_turn_left = new GButton(this, 10, 70, 80, 30, "Left Turn");
  button_stop = new GButton(this, 90, 70, 80, 30, "Stop");
  font = createFont("ArialMT", 48, true);
  // Open port.
  port = new Serial(this, "COM3", 115200);
  port.bufferUntil('\n');
  
}
 
void draw()
{
  background(255);
  stroke(0,0,0);
  strokeWeight(1);
  if(iteration>0)
  {
      // heading
      stroke(0,0,255);
      strokeWeight(2);
      line(offset_x-y_history[iteration-1]*zoom,offset_y-x_history[iteration-1]*zoom,offset_x-y_history[iteration-1]*zoom+1000.0*cos((heading_gyro+90.0)*3.1415/180.0),offset_y-x_history[iteration-1]*zoom-1000.0*sin((heading_gyro+90.0)*3.1415/180.0)); 
    
      // robot
      stroke(0,0,255);
      strokeWeight(2);
      fill(100,100,255);
      ellipseMode(CENTER);
      ellipse(offset_x-y_history[iteration-1]*zoom,offset_y-x_history[iteration-1]*zoom,100,100);
  }
    for (int i = 0; i < iteration-1; i++) 
    {
      // position
      stroke(0,0,0);
      strokeWeight(3);
      point(offset_x-y_history[i]*zoom,offset_y-x_history[i]*zoom,0); 
    }
  
  if(iteration>0)
  {
    


      
      // Afficheur
    {
          String x_str =  "X: " + (x_history[iteration-1]) + " m";
          stroke(0,0,0);
          fill(0,0,0);
          textFont(font, 28);
          text(x_str, 10, 920, 0);
          String y_str =  "Y: " + (y_history[iteration-1]) + " m";
          stroke(0,0,0);
          fill(0,0,0);
          textFont(font, 28);
          text(y_str, 10, 950, 0);
          String w_str =  "W: " + (heading_gyro) + " deg.";
          stroke(0,0,0);
          fill(0,0,0);
          textFont(font, 28);
          text(w_str, 10, 980, 0);          
    }
  }
  
  stroke(0,0,0);
  strokeWeight(1);
  line (0, offset_x, 0, 1000, offset_x, 0);
  line (offset_y, 0, 0, offset_y, 1000, 0);
    
}

void serialEvent(Serial p) 
{
  String incoming = p.readString();
    println(incoming);
  
  if ((incoming.length() > 8))
  {
    String[] list = split(incoming, " ");
      if ( list.length >= 8  ) 
      {
        heading_gyro = float(list[7])/1000.0f;
        x = float(list[8])/1000.0f;
        y = float(list[9])/1000.0f;
        
        x_history[iteration%100000] = x;
        y_history[iteration%100000] = y;
       ++iteration;
    }
  }
}
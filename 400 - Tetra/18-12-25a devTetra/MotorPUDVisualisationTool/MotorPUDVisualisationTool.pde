import processing.serial.*;
import java.awt.datatransfer.*;
import java.awt.Toolkit;
import processing.opengl.*;
import saito.objloader.*;
import g4p_controls.*;

Float xspeed_target = 0.0f;
Float xspeed_actual = 0.0f;
Float xpwm = 0.0f;
Float wspeed_target = 0.0f;
Float wspeed_actual = 0.0f;
Float wpwm = 0.0f;
Float wspeed_gyro = 0.0f;
Float heading_gyro = 0.0f;

int iteration  = 0;
Float[] xspeed_target_history = new Float[1000];
Float[] xspeed_actual_history = new Float[1000];
Float[] xpwm_history = new Float[1000];
Float[] wspeed_target_history = new Float[1000];
Float[] wspeed_actual_history = new Float[1000];
Float[] wpwm_history = new Float[1000];
Float[] wspeed_gyro_history = new Float[1000];
Float[] heading_gyro_history = new Float[1000];

Float zoom = 1.0f;
Float offset_x = 250.0f;
Float offset_w = 750.0f;

PFont font;

// Serial port state.
Serial       port;

void setup()
{
  size(1000, 1000, OPENGL);
  frameRate(30);
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
  line (0, offset_x, 0, 1000, offset_x, 0);
  line (0, offset_w, 0, 1000, offset_w, 0);
  if(iteration>=1000)
  {
    for (int i = 0; i < 1000; i++) 
    {
      // xspeed
      stroke(0,0,0);
      strokeWeight(2);
      point(i,offset_x-xspeed_target_history[i]*250.0,0); 
      // xspeed
      stroke(0,0,255);
      strokeWeight(3);
      point(i,offset_x-xspeed_actual_history[i]*250.0,0); 
      // xPWM
      stroke(255,0,0);
      strokeWeight(2);
      point(i,offset_x-xpwm_history[i]*0.5,0); 
      
      // wspeed
      stroke(0,0,0);
      strokeWeight(2);
      point(i,offset_w-wspeed_target_history[i]*5.0,0); 
      // wspeed
      stroke(0,0,255);
      strokeWeight(3);
      point(i,offset_w-wspeed_actual_history[i]*5.0,0); 
      // wspeed gyro
      stroke(0,255,0);
      strokeWeight(3);
      point(i,offset_w-wspeed_gyro_history[i]*5.0,0);       
      // wPWM
      stroke(255,0,0);
      strokeWeight(2);
      point(i,offset_w-wpwm_history[i]*0.5,0); 

    }
  }
  else
  {
    for (int i = 0; i < iteration; i++) 
    {
      // xspeed
      stroke(0,0,0);
      strokeWeight(2);
      point(i,offset_x-xspeed_target_history[i]*250.0,0); 
      // xspeed
      stroke(0,0,255);
      strokeWeight(3);
      point(i,offset_x-xspeed_actual_history[i]*250.0,0); 
      // xPWM
      stroke(255,0,0);
      strokeWeight(2);
      point(i,offset_x-xpwm_history[i]*0.5,0); 
      
      // wspeed
      stroke(0,0,0);
      strokeWeight(2);
      point(i,offset_w-wspeed_target_history[i]*5.0,0); 
      // wspeed
      stroke(0,0,255);
      strokeWeight(3);
      point(i,offset_w-wspeed_actual_history[i]*5.0,0); 
      // wspeed gyro
      stroke(0,255,0);
      strokeWeight(3);
      point(i,offset_w-wspeed_gyro_history[i]*5.0,0);       
      // wPWM
      stroke(255,0,0);
      strokeWeight(2);
      point(i,offset_w-wpwm_history[i]*0.5,0); 
    }
  }
  
  if(iteration>0)
  {
  //  String pitch =  "Pitch: " + (int)(ang_history[(iteration-1)%1000].x/1000.0) + "deg.";
  //  stroke(255,0,0);
  //  fill(255,0,0);
  //  textFont(font, 30);
  //  text(pitch, 100, 900, 0);
  //  String roll =  "Roll: " + (int)(ang_history[(iteration-1)%1000].y/1000.0) + "deg.";
  //  stroke(0,255,0);
  //  fill(0,255,0);
  //  textFont(font, 30);
  //  text(roll, 400, 900, 0);
  //  String yaw =  "Yaw: " + (int)(ang_history[(iteration-1)%1000].z/1000.0) + "deg.";
  //  stroke(0,0,255);
  //  fill(0,0,255);
  //  textFont(font, 30);
  //  text(yaw, 700, 900, 0);
    
  //  String yawgb =  "GyroBias: " + (int)(gyro_yaw_bias_history[(iteration-1)%1000]*1000.0) + "Raw";
  //  stroke(100,100,255);
  //  fill(100,100,255);
  //  textFont(font, 20);
  //  text(yawgb, 100, 960, 0);

  //  String yawgv =  "GyroVariance: " + (int)(gyro_yaw_deviation_history[(iteration-1)%1000]*1000.0) + "Raw";
  //  stroke(100,100,255);
  //  fill(100,100,255);
  //  textFont(font, 20);
  //  text(yawgv, 400, 960, 0);
    
  //  String yawg =  "Yaw(Gyro): " + (int)(ang2_history[(iteration-1)%1000].z/1000.0) + "deg.";
  //  stroke(100,100,255);
  //  fill(100,100,255);
  //  textFont(font, 30);
  //  text(yawg, 700, 960, 0);
    
  //  {
  //      String gravity_str =  "Gravity Field: " + gravity_history[(iteration-1)%1000] + "g";
  //      stroke(200,0,0);
  //      fill(200,0,0);
  //      textFont(font, 20);
  //      text(gravity_str, 10, 220, 0);

  //      String mfied_str =  "Mag Field: " + mfield_history[(iteration-1)%1000] + "gauss";
  //      stroke(0,0,200);
  //      fill(0,0,200);
  //      textFont(font, 20);
  //      text(mfied_str, 310, 220, 0);

  //      String mdip_str =  "Mag Dip Angle: " + mdip_history[(iteration-1)%1000] + "deg.";
  //      stroke(0,0,100);
  //      fill(0,0,100);
  //      textFont(font, 20);
  //      text(mdip_str, 610, 220, 0);
  //}
  //  // curve legend
  //  {
  //      String gravity_str =  "Gravity Field";
  //      stroke(200,0,0);
  //      fill(200,0,0);
  //      textFont(font, 12);
  //      text(gravity_str, 10, 200-gravity_history[0]*100.0-6, 0);
    
  //      String mfied_str =  "Mag Field";
  //      stroke(0,0,200);
  //      fill(0,0,200);
  //      textFont(font, 12);
  //      text(mfied_str, 10, 200-mfield_history[0]*100.0-6, 0);

  //      String mdip_str =  "Mag Dip Angle";
  //      stroke(0,0,100);
  //      fill(0,0,100);
  //      textFont(font, 12);
  //      text(mdip_str, 10, 200+mdip_history[0]*1.0-6, 0);
  //  }
  }
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
        xspeed_target = float(list[0])/1000.0f;
        xspeed_actual = float(list[1])/1000.0f;
        xpwm = float(list[2]);
        wspeed_target = float(list[3])/1000.0f;
        wspeed_actual = float(list[4])/1000.0f;
        wpwm = float(list[5]);
        wspeed_gyro = float(list[6])/1000.0f;
        heading_gyro = float(list[7])/1000.0f;
        
        xspeed_target_history[iteration%1000] = xspeed_target;
        xspeed_actual_history[iteration%1000] = xspeed_actual;
        xpwm_history[iteration%1000] = xpwm;
        wspeed_target_history[iteration%1000] = wspeed_target;
        wspeed_actual_history[iteration%1000] = wspeed_actual;
        wpwm_history[iteration%1000] = wpwm;
        wspeed_gyro_history[iteration%1000] = wspeed_gyro;
        heading_gyro_history[iteration%1000] = heading_gyro;
       
       ++iteration;
    }
  }
}
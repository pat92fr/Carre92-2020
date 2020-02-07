import processing.serial.*;
import java.awt.datatransfer.*;
import java.awt.Toolkit;
import processing.opengl.*;
import saito.objloader.*;
import g4p_controls.*;


Float yaw_from_gyro = 0.0f; // 4eme data dans télémétrie robot
Float yaw_from_table = 0.0f; // 2eme data dans télémétrie robot
Float yaw_ratio = 0.0f;

Float yaw_from_gyro_offset;
Float yaw_from_table_offset;

int iteration  = 0;
Float[] yaw_from_gyro_history = new Float[1000];
Float[] yaw_from_table_history = new Float[1000];
Float[] yaw_ratio_history = new Float[1000];

Float offset = 850.0;
Float zoom = 3.0;

PFont font;

// Serial port state.
Serial       port;
Serial       port2;

void setup()
{
  size(1000, 1000, OPENGL);
  frameRate(30);
  font = createFont("ArialMT", 48, true);

  port = new Serial(this, "COM9", 115200); // Table
  port.bufferUntil('\n');
  port2 = new Serial(this, "COM3", 115200); // Robot
  port2.bufferUntil('\n');
  //
}
 
void draw()
{
  background(255);
  stroke(0,0,0);
  strokeWeight(1);
  line (0, offset, 0, 1000, offset, 0);
  if(iteration>=1000)
  {
    for (int i = 0; i < 1000; i++) 
    {
      // dps consigne
      stroke(0,0,0);
      strokeWeight(2);
      point(i,offset-yaw_from_table_history[i]*zoom,0);  
      // dps
      stroke(0,0,255);
      strokeWeight(3);
      point(i,offset-yaw_from_gyro_history[i]*zoom,0);  
      // dps filtered
      stroke(255,0,0);
      strokeWeight(2);
      point(i,offset-yaw_ratio_history[i]*zoom,0);  
    }
  }
  else
  {
    for (int i = 0; i < iteration; i++) 
    {
      // dps consigne
      stroke(0,0,0);
      strokeWeight(2);
      point(i,offset-yaw_from_table_history[i]*zoom,0);  
      // dps
      stroke(0,0,255);
      strokeWeight(3);
      point(i,offset-yaw_from_gyro_history[i]*zoom,0);  
      // dps filtered
      stroke(255,0,0);
      strokeWeight(2);
      point(i,offset-yaw_ratio_history[i]*zoom,0);   
    }
  }
  
  if(iteration>0)
  {
    // curve legend
    {
        String dps_consigne_str =  "TURNTABLE YAW";
        stroke(0,0,0);
        fill(0,0,0);
        textFont(font, 12);
        text(dps_consigne_str, 10, offset-yaw_from_table_history[0]*zoom-30, 0);
        String dps_str =  "ROBOT YAW";
        stroke(0,0,255);
        fill(0,0,255);
        textFont(font, 12);
        text(dps_str, 10, offset-yaw_from_gyro_history[0]*zoom-15, 0);
        String pid_str =  "RATIO";
        stroke(255,0,0);
        fill(255,0,0);
        textFont(font, 12);
        text(pid_str, 10, offset-yaw_ratio_history[0]*zoom-6, 0);
      }
  // Afficheur
  {
        String dps_consigne_str =  "YAW REF : " + (yaw_from_table_history[(iteration-1)%1000]);
        stroke(0,0,0);
        fill(0,0,0);
        textFont(font, 28);
        text(dps_consigne_str, 10, 150, 0);
        String dps_str =  "YAW GYRO : " + (yaw_from_gyro_history[(iteration-1)%1000]);
        stroke(0,0,255);
        fill(0,0,255);
        textFont(font, 28);
        text(dps_str, 300, 150, 0);
        String pid_str =  "RATIO : " + (yaw_ratio_history[(iteration-1)%1000]);
        stroke(255,0,0);
        fill(255,0,0);
        textFont(font, 28);
        text(pid_str, 670, 150, 0);
    }  
  }
}

void serialEvent(Serial p) 
{
  String incoming = p.readString();
  if(p==port)
  {
    if ((incoming.length() > 2))
    {
      String[] list = split(incoming, " "); //<>//
        if ( list.length >= 2 ) 
        {
          yaw_from_table = float(list[1]);
          yaw_ratio = (yaw_from_table-yaw_from_gyro);
          
          yaw_from_table_history[iteration%1000] = yaw_from_table;
          yaw_from_gyro_history[iteration%1000] = yaw_from_gyro;
          yaw_ratio_history[iteration%1000] = yaw_ratio;
         ++iteration;
      }
    }
  }
  else if(p==port2)
  {
    if ((incoming.length() > 8))
    {
      String[] list = split(incoming, " ");
        if ( list.length >= 11 ) 
        {
          yaw_from_gyro = float(list[4])/1000.0f;
      }
    }    
  }
}
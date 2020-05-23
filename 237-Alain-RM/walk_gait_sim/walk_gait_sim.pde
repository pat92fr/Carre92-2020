import processing.opengl.*;

PFont font;

int windows_size_x = 1000;
int windows_size_y = 1000;
int windows_center_x = 500;
int windows_center_y = 500;
float windows_scale = 1000.0F;

void setup()
{
  size(1000, 1000, OPENGL);
  frameRate(30);
  font = createFont("ArialMT", 48, true);
    
}

// x points to front
// y points to left
// z points to up

float robot_size_x = 0.140F; // distance from CoG to the leg along X 
float robot_size_y = 0.090F; // distance from CoG to the leg along Y 
float robot_size_z = 0.000F; // distance from CoG to the leg along Z

float leg_stdby_pos_x =  0.0F; // distance from leg origin to the foot along X
float leg_stdby_pos_y =  0.0F; // distance from leg origin to the foot along Y
float leg_stdby_pos_z = -0.20F; // distance from leg origin to the foot along Z

float stride_distance = 0.060F; // L
float lift_distance = 0.040F; //
 
 class point3df {
   public float x;
   public  float y;
   public float z;
   
  public point3df()
  {
    x = 0.0;
    y = 0.0;
    z = 0.0;
  }   
 }  
 
 void trajectory(float relative_time_ms, point3df p)
 {
   float relative_time = (relative_time_ms % 4000.0) / 1000.0;
   //print(relative_time);
  // pos_x
  if( ( relative_time >= 0.0F ) && ( relative_time < 1.5F ) ) // phase 2.5 to 4
  {
    p.x = -relative_time / 1.5F * stride_distance / 2.0F;
    p.z = 0.0F;
    //print("stride");
  }
  else if( ( relative_time >= 1.5 ) && ( relative_time < 2.5F ) ) // phase 4
  {
    p.x = (relative_time - 1.5F) * stride_distance - stride_distance / 2.0F;
    p.z = 4.0F;
    //print("swing");
  }
  else if( ( relative_time >= 2.5 ) && ( relative_time < 4.0F ) ) // phase 1 to 2.5
  {
    p.x = - (relative_time - 2.5F) * stride_distance / 2.0 / 1.5F + stride_distance / 2.0F;
    p.z = 0.0F;
    //print("stride");
  }   
 }
 
 
void draw()
{
  background(255);
  
  float absolute_time = millis();
  float relative_time = absolute_time % 4000.0F;
 
  String absolute_time_text =  absolute_time + "ms";
  stroke(255,0,0);
  fill(255,0,0);
  textFont(font, 20);
  text(absolute_time_text, 10, 20, 0);

  String relative_time_text =  relative_time + "ms";
  stroke(255,0,0);
  fill(255,0,0);
  textFont(font, 20);
  text(relative_time_text, 10, 40, 0);
  

  point3df FR_leg_pos = new point3df();
  point3df BR_leg_pos = new point3df();
  point3df BL_leg_pos = new point3df();
  point3df FL_leg_pos = new point3df();
  
  trajectory(relative_time, FR_leg_pos);
  trajectory(relative_time+1000.0F, BR_leg_pos);
  trajectory(relative_time+2000.0F, BL_leg_pos);
  trajectory(relative_time+3000.0F, FL_leg_pos);

  // text feet

  String FR_text =  "Front Right Foot";
  stroke(0,0,0);
  fill(0,0,0);
  textFont(font, 20);
  text(FR_text, 
    windows_center_x+windows_scale*(robot_size_x)+0,
    windows_center_y+windows_scale*(robot_size_y)+30,
    0);

  String FL_text =  "Front Left Foot";
  stroke(0,0,0);
  fill(0,0,0);
  textFont(font, 20);
  text(FL_text, 
    windows_center_x+windows_scale*(robot_size_x)+0,
    windows_center_y+windows_scale*(-robot_size_y)-30,
    0);

  String BR_text =  "Rear Right Foot";
  stroke(0,0,0);
  fill(0,0,0);
  textFont(font, 20);
  text(BR_text, 
    windows_center_x+windows_scale*(-robot_size_x)+0,
    windows_center_y+windows_scale*(robot_size_y)+30,
    0);

  String BL_text =  "Rear Left Foot";
  stroke(0,0,0);
  fill(0,0,0);
  textFont(font, 20);
  text(BL_text, 
    windows_center_x+windows_scale*(-robot_size_x)+0,
    windows_center_y+windows_scale*(-robot_size_y)-30,
    0);

  String COG_text =  "CoG";
  stroke(0,0,0);
  fill(0,0,0);
  textFont(font, 20);
  text(COG_text, 
    windows_center_x-15,
    windows_center_y-30,
    0);
    
  // draw feet
  strokeWeight(10); 
  if( FR_leg_pos.z == 0.0F)
    stroke(255,0,0);
  else 
    stroke(0,255,0);
  point(
    windows_center_x+windows_scale*(robot_size_x+FR_leg_pos.x),
    windows_center_y+windows_scale*(robot_size_y+FR_leg_pos.y),
    0.0F
  );  
  if( BR_leg_pos.z == 0.0F)
    stroke(255,0,0);
  else 
    stroke(0,255,0);
  point(
    windows_center_x+windows_scale*(-robot_size_x+BR_leg_pos.x),
    windows_center_y+windows_scale*(robot_size_y+BR_leg_pos.y),
    0.0F
  );  
  if( BL_leg_pos.z == 0.0F)
    stroke(255,0,0);
  else 
    stroke(0,255,0);
  point(
    windows_center_x+windows_scale*(-robot_size_x+BL_leg_pos.x),
    windows_center_y+windows_scale*(-robot_size_y+BL_leg_pos.y),
    0.0F
  );  
  if( FL_leg_pos.z == 0.0F)
    stroke(255,0,0);
  else 
    stroke(0,255,0);
  point(
    windows_center_x+windows_scale*(robot_size_x+FL_leg_pos.x),
    windows_center_y+windows_scale*(-robot_size_y+FL_leg_pos.y),
    0.0F
  );  

  // CoG
  stroke(0,0,0);
  strokeWeight(5); 
  point(
    windows_center_x+windows_scale*(0.000F),
    windows_center_y+windows_scale*(0.000F),
    0.0F
  );  
  
  // Stability margin
  stroke(0,0,0);
  fill(255,255,255);
  strokeWeight(1); 
  ellipse(
    windows_center_x+windows_scale*(0.000F),
    windows_center_y+windows_scale*(0.000F),
    48.0F,
    48.0F
  );  
  
  // ground contact shape
  stroke(0,0,255);
  strokeWeight(2); 
  if( FL_leg_pos.z > 0.0F)
  {
    line (
      windows_center_x+windows_scale*(robot_size_x+FR_leg_pos.x),
      windows_center_y+windows_scale*(robot_size_y+FR_leg_pos.y),
      0.0F,
      windows_center_x+windows_scale*(-robot_size_x+BR_leg_pos.x),
      windows_center_y+windows_scale*(robot_size_y+BR_leg_pos.y),
      0.0F
    );
    line (
      windows_center_x+windows_scale*(-robot_size_x+BR_leg_pos.x),
      windows_center_y+windows_scale*(robot_size_y+BR_leg_pos.y),
      0.0F,
      windows_center_x+windows_scale*(-robot_size_x+BL_leg_pos.x),
      windows_center_y+windows_scale*(-robot_size_y+BL_leg_pos.y),
      0.0F    
    );
    line (
      windows_center_x+windows_scale*(-robot_size_x+BL_leg_pos.x),
      windows_center_y+windows_scale*(-robot_size_y+BL_leg_pos.y),
      0.0F  ,
      windows_center_x+windows_scale*(robot_size_x+FR_leg_pos.x),
      windows_center_y+windows_scale*(robot_size_y+FR_leg_pos.y),
      0.0F
    );
  }
  else if( BL_leg_pos.z > 0.0F)
  {
    line (
      windows_center_x+windows_scale*(robot_size_x+FR_leg_pos.x),
      windows_center_y+windows_scale*(robot_size_y+FR_leg_pos.y),
      0.0F,
      windows_center_x+windows_scale*(-robot_size_x+BR_leg_pos.x),
      windows_center_y+windows_scale*(robot_size_y+BR_leg_pos.y),
      0.0F
    );
    line (
      windows_center_x+windows_scale*(-robot_size_x+BR_leg_pos.x),
      windows_center_y+windows_scale*(robot_size_y+BR_leg_pos.y),
      0.0F,
      windows_center_x+windows_scale*(robot_size_x+FL_leg_pos.x),
      windows_center_y+windows_scale*(-robot_size_y+FL_leg_pos.y),
      0.0F    
    );
    line (
      windows_center_x+windows_scale*(robot_size_x+FL_leg_pos.x),
      windows_center_y+windows_scale*(-robot_size_y+FL_leg_pos.y),
      0.0F  ,
      windows_center_x+windows_scale*(robot_size_x+FR_leg_pos.x),
      windows_center_y+windows_scale*(robot_size_y+FR_leg_pos.y),
      0.0F
    );    
  }
  else if( FR_leg_pos.z > 0.0F)
  {
    line (
      windows_center_x+windows_scale*(-robot_size_x+BR_leg_pos.x),
      windows_center_y+windows_scale*(robot_size_y+BR_leg_pos.y),
      0.0F,
      windows_center_x+windows_scale*(-robot_size_x+BL_leg_pos.x),
      windows_center_y+windows_scale*(-robot_size_y+BL_leg_pos.y),
      0.0F
    );
    line (
      windows_center_x+windows_scale*(-robot_size_x+BL_leg_pos.x),
      windows_center_y+windows_scale*(-robot_size_y+BL_leg_pos.y),
      0.0F,
      windows_center_x+windows_scale*(robot_size_x+FL_leg_pos.x),
      windows_center_y+windows_scale*(-robot_size_y+FL_leg_pos.y),
      0.0F    
    );
    line (
      windows_center_x+windows_scale*(robot_size_x+FL_leg_pos.x),
      windows_center_y+windows_scale*(-robot_size_y+FL_leg_pos.y),
      0.0F  ,
      windows_center_x+windows_scale*(-robot_size_x+BR_leg_pos.x),
      windows_center_y+windows_scale*(robot_size_y+BR_leg_pos.y),
      0.0F
    );     
  }
  else if( BR_leg_pos.z > 0.0F)
  {
    line (
      windows_center_x+windows_scale*(robot_size_x+FR_leg_pos.x),
      windows_center_y+windows_scale*(robot_size_y+FR_leg_pos.y),
      0.0F,
      windows_center_x+windows_scale*(-robot_size_x+BL_leg_pos.x),
      windows_center_y+windows_scale*(-robot_size_y+BL_leg_pos.y),
      0.0F
    );
    line (
      windows_center_x+windows_scale*(-robot_size_x+BL_leg_pos.x),
      windows_center_y+windows_scale*(-robot_size_y+BL_leg_pos.y),
      0.0F,
      windows_center_x+windows_scale*(robot_size_x+FL_leg_pos.x),
      windows_center_y+windows_scale*(-robot_size_y+FL_leg_pos.y),
      0.0F    
    );
    line (
      windows_center_x+windows_scale*(robot_size_x+FL_leg_pos.x),
      windows_center_y+windows_scale*(-robot_size_y+FL_leg_pos.y),
      0.0F  ,
      windows_center_x+windows_scale*(robot_size_x+FR_leg_pos.x),
      windows_center_y+windows_scale*(robot_size_y+FR_leg_pos.y),
      0.0F
    );    
  }
  
  
}
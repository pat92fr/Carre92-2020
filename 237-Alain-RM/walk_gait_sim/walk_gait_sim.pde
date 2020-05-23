import processing.opengl.*;

class point3df {
  public float x;
  public float y;
  public float z;

  public point3df()
  {
    x = 0.0;
    y = 0.0;
    z = 0.0;
  }
}

PFont font;

int windows_size_x = 1000; // pixels
int windows_size_y = 1000; // pixels
int windows_center_x = 500; // pixels
int windows_center_y = 500; // pixels
float windows_scale = 2000.0F; // pixels per meter
float slow_motion_factor = 5.0F; // divide real-time by 5

// x points to front
// y points to left
// z points to up

float robot_size_x = 0.140F; // distance from CoG to the leg along X 
float robot_size_y = 0.090F; // distance from CoG to the leg along Y 
float robot_size_z = 0.000F; // distance from CoG to the leg along Z

float leg_stdby_pos_x =  0.0F; // distance from leg origin to the foot along X
float leg_stdby_pos_y =  0.0F; // distance from leg origin to the foot along Y
float leg_stdby_pos_z = -0.20F; // distance from leg origin to the foot along Z

float stride_distance = 0.060F; // m
float lift_distance = 0.040F; // m
float gait_period = 4.0F; // secondes

void trajectory(float relative_time_ms, point3df p)
{
  float relative_time = (relative_time_ms % 4000.0) / 1000.0;
  //print(relative_time);
  // pos_x
  if ( ( relative_time >= 0.0F ) && ( relative_time < 1.5F ) ) // phase 2.5 to 4
  {
    p.x = -relative_time / 1.5F * stride_distance / 2.0F;
    p.z = 0.0F;
    //print("stride");
  } else if ( ( relative_time >= 1.5 ) && ( relative_time < 2.5F ) ) // phase 4
  {
    p.x = (relative_time - 1.5F) * stride_distance - stride_distance / 2.0F;
    p.z = 4.0F;
    //print("swing");
  } else if ( ( relative_time >= 2.5 ) && ( relative_time < 4.0F ) ) // phase 1 to 2.5
  {
    p.x = - (relative_time - 2.5F) * stride_distance / 2.0 / 1.5F + stride_distance / 2.0F;
    p.z = 0.0F;
    //print("stride");
  }
}


void setup()
{
  size(1000,1000,OPENGL);
  frameRate(30);
  font = createFont("ArialMT", 48, true);
}

void draw()
{
  float absolute_time = millis() / slow_motion_factor;
  float relative_time = absolute_time % 4000.0F;

  point3df FR_leg_pos = new point3df();
  point3df BR_leg_pos = new point3df();
  point3df BL_leg_pos = new point3df();
  point3df FL_leg_pos = new point3df();

  trajectory(relative_time, FR_leg_pos);
  trajectory(relative_time+1000.0F, BR_leg_pos);
  trajectory(relative_time+2000.0F, BL_leg_pos);
  trajectory(relative_time+3000.0F, FL_leg_pos);


  float x1 = 0.0F;
  float y1 = 0.0F;
  float x2 = 0.0F;
  float y2 = 0.0F;
  float x3 = 0.0F;
  float y3 = 0.0F;
  if ( FL_leg_pos.z > 0.0F)
  {
    x1 = robot_size_x+FR_leg_pos.x;
    y1 = robot_size_y+FR_leg_pos.y;
    x2 = -robot_size_x+BR_leg_pos.x;
    y2 = robot_size_y+BR_leg_pos.y;
    x3 = -robot_size_x+BL_leg_pos.x;
    y3 = -robot_size_y+BL_leg_pos.y;
  } else if ( BL_leg_pos.z > 0.0F)
  {
    x1 = robot_size_x+FR_leg_pos.x;
    y1 = robot_size_y+FR_leg_pos.y;
    x2 = -robot_size_x+BR_leg_pos.x;
    y2 = robot_size_y+BR_leg_pos.y;
    x3 = robot_size_x+FL_leg_pos.x;
    y3 = -robot_size_y+FL_leg_pos.y;
  } else if ( FR_leg_pos.z > 0.0F)
  {
    x1 = -robot_size_x+BR_leg_pos.x;
    y1 = robot_size_y+BR_leg_pos.y;
    x2 = -robot_size_x+BL_leg_pos.x;
    y2 = -robot_size_y+BL_leg_pos.y;
    x3 = robot_size_x+FL_leg_pos.x;
    y3 = -robot_size_y+FL_leg_pos.y;
  } else if ( BR_leg_pos.z > 0.0F)
  {
    x1 = robot_size_x+FR_leg_pos.x;
    y1 = robot_size_y+FR_leg_pos.y;
    x2 = -robot_size_x+BL_leg_pos.x;
    y2 = -robot_size_y+BL_leg_pos.y;
    x3 = robot_size_x+FL_leg_pos.x;
    y3 = -robot_size_y+FL_leg_pos.y;
  }

  // static cog
  float xcog = 0.0F;
  float ycog = 0.0F;

  // check cog inside ground pattern : http://www.jeffreythompson.org/collision-detection/tri-point.php
  float areaOrig = abs( (x2-x1)*(y3-y1) - (x3-x1)*(y2-y1) );
  float area1 =    abs( (x1-xcog)*(y2-ycog) - (x2-xcog)*(y1-ycog) );
  float area2 =    abs( (x2-xcog)*(y3-ycog) - (x3-xcog)*(y2-ycog) );
  float area3 =    abs( (x3-xcog)*(y1-ycog) - (x1-xcog)*(y3-ycog) );
  float diff = abs( (area1 + area2 + area3 ) - areaOrig );
  float EPSILON = 0.0001;
  boolean inside = diff < EPSILON;
  // compute margin : //https://hackaday.io/project/21904-hexapod-modelling-path-planning-and-control/log/62326-3-fundamentals-of-hexapod-robot
  float h_min = 0.0;
  if (inside)
  {
    float S12=  abs( (x1-xcog)*(y2-ycog)-(x2-xcog)*(y1-ycog) ) / 2.0F;
    float L12 = sqrt( pow((x2-x1), 2) + pow((y2-y1), 2) );
    float h12 = 2.0* S12 / L12;
    float S23=  abs( (x2-xcog)*(y3-ycog)-(x3-xcog)*(y2-ycog) ) / 2.0F;
    float L23 = sqrt( pow((x3-x2), 2) + pow((y3-y2), 2) );
    float h23 = 2.0* S23 / L23;
    float S31=  abs( (x3-xcog)*(y1-ycog)-(x1-xcog)*(y3-ycog) ) / 2.0F;
    float L31 = sqrt( pow((x3-x1), 2) + pow((y3-y1), 2) );
    float h31 = 2.0* S31 / L31;
    h_min = h12;
    h_min = min(h_min, h23);
    h_min = min(h_min, h31);
    print("h: "+h_min+"\n");
  } else
    print("instability\n");

  background(255);

  // ground contact shape
  stroke(0, 255, 0);
  strokeWeight(1); 
  fill(100, 255, 100);
  beginShape();
  vertex(windows_center_x+windows_scale*x1, windows_center_y+windows_scale*y1);
  vertex(windows_center_x+windows_scale*x2, windows_center_y+windows_scale*y2);
  vertex(windows_center_x+windows_scale*x3, windows_center_y+windows_scale*y3);
  endShape(CLOSE);

  // draw feet
  strokeWeight(12); 
  if ( FR_leg_pos.z == 0.0F)
    stroke(0, 100, 0);
  else 
    stroke(255, 0, 0);
  point(
    windows_center_x+windows_scale*(robot_size_x+FR_leg_pos.x), 
    windows_center_y+windows_scale*(robot_size_y+FR_leg_pos.y), 
    1.0F
    );  
  if ( BR_leg_pos.z == 0.0F)
    stroke(0, 100, 0);
  else 
    stroke(255, 0, 0);
  point(
    windows_center_x+windows_scale*(-robot_size_x+BR_leg_pos.x), 
    windows_center_y+windows_scale*(robot_size_y+BR_leg_pos.y), 
    1.0F
    );  
  if ( BL_leg_pos.z == 0.0F)
    stroke(0, 100, 0);
  else 
    stroke(255, 0, 0);
  point(
    windows_center_x+windows_scale*(-robot_size_x+BL_leg_pos.x), 
    windows_center_y+windows_scale*(-robot_size_y+BL_leg_pos.y), 
    1.0F
    );  
  if ( FL_leg_pos.z == 0.0F)
    stroke(0, 100, 0);
  else 
    stroke(255, 0, 0);
  point(
    windows_center_x+windows_scale*(robot_size_x+FL_leg_pos.x), 
    windows_center_y+windows_scale*(-robot_size_y+FL_leg_pos.y), 
    1.0F
    );  

  // CoG
  if ( h_min > 0.0F )
  {
    stroke(0, 0, 0);
  } else
  {
    stroke(255, 0, 0);
  }
  strokeWeight(5); 
  point(
    windows_center_x+windows_scale*(0.000F), 
    windows_center_y+windows_scale*(0.000F), 
    1.0F
    );  

  // Stability margin
  if ( h_min > 0.0F )
  {
    stroke(0, 0, 0);
  } else
  {
    stroke(255, 0, 0);
  }
  fill(255, 255, 255);
  strokeWeight(1); 
  ellipse(
    windows_center_x+windows_scale*(0.000F), 
    windows_center_y+windows_scale*(0.000F), 
    windows_scale*h_min*2.0F, 
    windows_scale*h_min*2.0F
    );  


  // text time
  String absolute_time_text =  absolute_time + "ms";
  stroke(255, 0, 0);
  fill(0, 0, 0);
  textFont(font, 20);
  text(absolute_time_text, 10, 20, 0);

  String relative_time_text =  relative_time + "ms";
  stroke(255, 0, 0);
  fill(0, 0, 0);
  textFont(font, 20);
  text(relative_time_text, 10, 40, 0);

  // text feet
  String FR_text =  "Front Right Foot";
  stroke(0, 0, 0);
  fill(0, 0, 0);
  textFont(font, 20);
  text(FR_text, 
    windows_center_x+windows_scale*(robot_size_x)+0, 
    windows_center_y+windows_scale*(robot_size_y)+30, 
    0);

  String FL_text =  "Front Left Foot";
  stroke(0, 0, 0);
  fill(0, 0, 0);
  textFont(font, 20);
  text(FL_text, 
    windows_center_x+windows_scale*(robot_size_x)+0, 
    windows_center_y+windows_scale*(-robot_size_y)-30, 
    0);

  String BR_text =  "Rear Right Foot";
  stroke(0, 0, 0);
  fill(0, 0, 0);
  textFont(font, 20);
  text(BR_text, 
    windows_center_x+windows_scale*(-robot_size_x)+0, 
    windows_center_y+windows_scale*(robot_size_y)+30, 
    0);

  String BL_text =  "Rear Left Foot";
  stroke(0, 0, 0);
  fill(0, 0, 0);
  textFont(font, 20);
  text(BL_text, 
    windows_center_x+windows_scale*(-robot_size_x)+0, 
    windows_center_y+windows_scale*(-robot_size_y)-30, 
    0);

  String COG_text =  "CoG";
  stroke(0, 0, 0);
  fill(0, 0, 0);
  textFont(font, 20);
  text(COG_text, 
    windows_center_x-15, 
    windows_center_y-30, 
    0);

  String margin_text =  "margin="+String.format("%2.3f%n", h_min);
  if ( h_min > 0.0F )
  {
    stroke(0, 0, 0);
    fill(0, 150, 0);
  } else
  {
    stroke(255, 0, 0);
    fill(150, 0, 0);
    margin_text =  "Falling!";
  }  
  textFont(font, 16);
  text(margin_text, 
    windows_center_x-25, 
    windows_center_y-10, 
    0
    );
}
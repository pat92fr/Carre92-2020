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
float slow_motion_factor = 1.0F; // divide real-time by 5

// x points to front
// y points to left
// z points to up

float robot_size_x = 0.140F; // distance from CoG to the leg along X 
float robot_size_y = 0.090F; // distance from CoG to the leg along Y 
float robot_size_z = 0.000F; // distance from CoG to the leg along Z

float stride_distance = 0.100F; // m
float lift_distance = 0.040F; // m
float gait_period = 5.0F; // secondes

float a = robot_size_x - stride_distance / 2.0F; // c
float b = robot_size_x + stride_distance / 2.0F; // c
float c = robot_size_y; // c
float p = 4.0/9.0*a*a + a*b/3.0 - c*c;
float q = a*( a*a + a*b - 3*c*c)/6.0 - a*c*c + b*c*c/2.0 - a*a*a/27.0;
//float xp = pow(sqrt(q*q-p*p*p)-q,1/3.0)-a/3.0+p/pow(sqrt(q*q-p*p*p)-q,1/3.0);
//float xp = pow(sqrt(abs(q*q-p*p*p))-q,1/3.0)-a/3.0+p/pow(sqrt(abs(q*q-p*p*p))-q,1/3.0);
float xp = a+0.01;
float l = b-xp;

void trajectory(float time_ms, point3df FL, point3df FR, point3df BL, point3df BR)
{
  float relative_time = (time_ms / 1000.0 ) % gait_period;
  //print(relative_time);
  //if ( ( relative_time >= 0.0F ) && ( relative_time < 1.0F ) ) // phase 1
  //{
  //  FL.x = xp - robot_size_x;
  //  FR.x = - stride_distance / 2.0;
  //  BL.x = - stride_distance / 2.0;
  //  BR.x = - xp + robot_size_x - stride_distance / 2.0;
  //}
  if ( ( relative_time >= 0.0F ) && ( relative_time < 1.0F ) ) // phase a : swing BL
  {
    FL.x = xp - robot_size_x;
    FL.z = 0.0F;
    FR.x = - stride_distance / 2.0;
    FR.z = 0.0F;
    BL.x = - stride_distance / 2.0 + l*relative_time;
    BL.z = lift_distance;
    BR.x = - xp + robot_size_x - stride_distance / 2.0;
    BR.z = 0.0F;
  }
  else if ( ( relative_time >= 1.0F ) && ( relative_time < 2.0F ) ) // phase b : swing BR
  {
    FL.x = xp - robot_size_x;
    FL.z = 0.0F;
    FR.x = - stride_distance / 2.0;
    FR.z = 0.0F;
    BL.x = - stride_distance / 2.0 + l;
    BL.z = 0.0F;
    BR.x = - xp + robot_size_x - stride_distance / 2.0 + l * (relative_time-1.0F);
    BR.z = lift_distance;
  }
  else if ( ( relative_time >= 2.0F ) && ( relative_time < 3.0F ) ) // phase c : swing FR
  {
    FL.x = xp - robot_size_x;
    FL.z = 0.0F;
    FR.x = - stride_distance / 2.0 + l * (relative_time-2.0F);
    FR.z = lift_distance;
    BL.x = - stride_distance / 2.0 + l;
    BL.z = 0.0F;
    BR.x = - xp + robot_size_x - stride_distance / 2.0 + l;
    BR.z = 0.0F;
  }  
  else if ( ( relative_time >= 3.0F ) && ( relative_time < 4.0F ) ) // phase d : swing FL
  {
    FL.x = xp - robot_size_x + l * (relative_time-3.0F);
    FL.z = lift_distance;
    FR.x = - stride_distance / 2.0 + l;
    FR.z = 0.0F;
    BL.x = - stride_distance / 2.0 + l;
    BL.z = 0.0F;
    BR.x = - xp + robot_size_x - stride_distance / 2.0 + l;
    BR.z = 0.0F;
  }  
  else if ( ( relative_time >= 4.0F ) && ( relative_time < 5.0F ) ) // phase e : move CoG
  {
    FL.x = xp - robot_size_x + l * (1.0 - relative_time + 4.0F);
    FL.z = 0.0F;
    FR.x = - stride_distance / 2.0 + l * (1.0 - relative_time + 4.0F);
    FR.z = 0.0F;
    BL.x = - stride_distance / 2.0 + l * (1.0 - relative_time + 4.0F);
    BL.z = 0.0F;
    BR.x = - xp + robot_size_x - stride_distance / 2.0 + l * (1.0 - relative_time + 4.0F);
    BR.z = 0.0F;
  }   
}


void setup()
{
  size(1000,1000,OPENGL);
  frameRate(30);
  font = createFont("ArialMT", 48, true);
  
  print("a:"+a+"\n");
  print("b:"+b+"\n");
  print("c:"+c+"\n");
  print("p:"+p+"\n");
  print("q:"+q+"\n");
  print("xp:"+xp+"\n");

}

void draw()
{
  float absolute_time = millis() / slow_motion_factor;
  float relative_time = absolute_time % (gait_period*1000.0F);

  point3df FR_leg_pos = new point3df();
  point3df BR_leg_pos = new point3df();
  point3df BL_leg_pos = new point3df();
  point3df FL_leg_pos = new point3df();

  trajectory(absolute_time, FL_leg_pos, FR_leg_pos, BL_leg_pos, BR_leg_pos);


  float x1 = 0.0F;
  float y1 = 0.0F;
  float x2 = 0.0F;
  float y2 = 0.0F;
  float x3 = 0.0F;
  float y3 = 0.0F;
  float x4 = 0.0F;
  float y4 = 0.0F;
  boolean three_point = true;
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
  else
  {
    x1 = robot_size_x+FR_leg_pos.x;
    y1 = robot_size_y+FR_leg_pos.y;
    x2 = robot_size_x+FL_leg_pos.x;
    y2 = -robot_size_y+FL_leg_pos.y;
    x3 = -robot_size_x+BL_leg_pos.x;
    y3 = -robot_size_y+BL_leg_pos.y;
    x4 = -robot_size_x+BR_leg_pos.x;
    y4 = robot_size_y+BR_leg_pos.y;
    three_point = false;
  }
  // static cog
  float xcog = 0.0F;
  float ycog = 0.0F;

  float h_min = 0.0;
  if(three_point)
  {
    // check cog inside ground pattern : http://www.jeffreythompson.org/collision-detection/tri-point.php
    float areaOrig = abs( (x2-x1)*(y3-y1) - (x3-x1)*(y2-y1) );
    float area1 =    abs( (x1-xcog)*(y2-ycog) - (x2-xcog)*(y1-ycog) );
    float area2 =    abs( (x2-xcog)*(y3-ycog) - (x3-xcog)*(y2-ycog) );
    float area3 =    abs( (x3-xcog)*(y1-ycog) - (x1-xcog)*(y3-ycog) );
    float diff = abs( (area1 + area2 + area3 ) - areaOrig );
    float EPSILON = 0.0001;
    boolean inside = diff < EPSILON;
    // compute margin : //https://hackaday.io/project/21904-hexapod-modelling-path-planning-and-control/log/62326-3-fundamentals-of-hexapod-robot
    h_min = 0.0;
    if (inside)
    {
      float S12=  abs( (x1-xcog)*(y2-ycog)-(x2-xcog)*(y1-ycog) ) / 2.0F;
      float L12 = sqrt( pow((x2-x1), 2) + pow((y2-y1), 2) );
      float h12 = 2.0 * S12 / L12;
      float S23=  abs( (x2-xcog)*(y3-ycog)-(x3-xcog)*(y2-ycog) ) / 2.0F;
      float L23 = sqrt( pow((x3-x2), 2) + pow((y3-y2), 2) );
      float h23 = 2.0* S23 / L23;
      float S31=  abs( (x3-xcog)*(y1-ycog)-(x1-xcog)*(y3-ycog) ) / 2.0F;
      float L31 = sqrt( pow((x3-x1), 2) + pow((y3-y1), 2) );
      float h31 = 2.0* S31 / L31;
      h_min = h12;
      h_min = min(h_min, h23);
      h_min = min(h_min, h31);
      //print("h: "+h_min+"\n");
    } 
    //else
      //print("instability\n");
  }
  else
  {
      float S12=  abs( (x1-xcog)*(y2-ycog)-(x2-xcog)*(y1-ycog) ) / 2.0F;
      float L12 = sqrt( pow((x2-x1), 2) + pow((y2-y1), 2) );
      float h12 = 2.0 * S12 / L12;
      float S23=  abs( (x2-xcog)*(y3-ycog)-(x3-xcog)*(y2-ycog) ) / 2.0F;
      float L23 = sqrt( pow((x3-x2), 2) + pow((y3-y2), 2) );
      float h23 = 2.0* S23 / L23;
      float S34=  abs( (x3-xcog)*(y4-ycog)-(x4-xcog)*(y3-ycog) ) / 2.0F;
      float L34 = sqrt( pow((x3-x4), 2) + pow((y3-y4), 2) );
      float h34 = 2.0* S34 / L34;
      float S41=  abs( (x4-xcog)*(y1-ycog)-(x1-xcog)*(y4-ycog) ) / 2.0F;
      float L41 = sqrt( pow((x4-x1), 2) + pow((y4-y1), 2) );
      float h41 = 2.0* S41 / L41;
      h_min = h12;
      h_min = min(h_min, h23);
      h_min = min(h_min, h34);
      h_min = min(h_min, h41);
  }  
  
  
  
  // drawings
  background(255);

  // ground contact shape
  stroke(0, 255, 0);
  strokeWeight(1); 
  fill(100, 255, 100);
  beginShape();
  vertex(windows_center_x+windows_scale*x1, windows_center_y+windows_scale*y1);
  vertex(windows_center_x+windows_scale*x2, windows_center_y+windows_scale*y2);
  vertex(windows_center_x+windows_scale*x3, windows_center_y+windows_scale*y3);
  if(!three_point)
    vertex(windows_center_x+windows_scale*x4, windows_center_y+windows_scale*y4);
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
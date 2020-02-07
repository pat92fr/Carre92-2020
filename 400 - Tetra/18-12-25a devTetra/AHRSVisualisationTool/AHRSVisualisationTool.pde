import processing.serial.*;
import java.awt.datatransfer.*;
import java.awt.Toolkit;
import processing.opengl.*;
import saito.objloader.*;
import g4p_controls.*;

PVector ang  = new PVector(0.0F,0.0F,0.0F);
PVector ang2  = new PVector(0.0F,0.0F,0.0F);
Float gravity;
Float mfield;
Float gyro_yaw_mean;
Float gyro_yaw_deviation;
Float gyro_yaw_bias;
Float mdip;

int iteration  = 0;
PVector[] ang_history = new PVector[1000];
PVector[] ang2_history = new PVector[1000];
Float[] gravity_history = new Float[1000];
Float[] mfield_history = new Float[1000];
Float[] gyro_yaw_mean_history = new Float[1000];
Float[] gyro_yaw_deviation_history = new Float[1000];
Float[] gyro_yaw_bias_history = new Float[1000];
Float[] mdip_history = new Float[1000];

PFont font;

// Serial port state.
Serial       port;
final String serialConfigFile = "serialconfig.txt";
boolean      printSerial = false;

// UI controls.
GPanel    configPanel;
GDropList serialList;
GLabel    serialLabel;
GCheckbox printSerialCheckbox;

void setup()
{
  size(1000, 1000, OPENGL);
  frameRate(30);
  font = createFont("ArialMT", 48, true);
  
  // Serial port setup.
  // Grab list of serial ports and choose one that was persisted earlier or default to the first port.
  int selectedPort = 0;
  String[] availablePorts = Serial.list();
  if (availablePorts == null) {
    println("ERROR: No serial ports available!");
    exit();
  }
  String[] serialConfig = loadStrings(serialConfigFile);
  if (serialConfig != null && serialConfig.length > 0) {
    String savedPort = serialConfig[0];
    // Check if saved port is in available ports.
    for (int i = 0; i < availablePorts.length; ++i) {
      if (availablePorts[i].equals(savedPort)) {
        selectedPort = i;
      } 
    }
  }
  // Build serial config UI.
  configPanel = new GPanel(this, 10, 10, width-20, 90, "Configuration (click to hide/show)");
  serialLabel = new GLabel(this,  0, 20, 80, 25, "Serial port:");
  configPanel.addControl(serialLabel);
  serialList = new GDropList(this, 90, 20, 200, 200, 6);
  serialList.setItems(availablePorts, selectedPort);
  configPanel.addControl(serialList);
  printSerialCheckbox = new GCheckbox(this, 5, 50, 200, 20, "Print serial data");
  printSerialCheckbox.setSelected(printSerial);
  configPanel.addControl(printSerialCheckbox);
  // Set serial port.
  setSerialPort(serialList.getSelectedText());
}
 
void draw()
{
  background(255);
  stroke(0,0,0);
  strokeWeight(1);
  line (0, 500, 0, 1000, 500, 0);
  line (0, 200, 0, 1000, 200, 0);
  if(iteration>=1000)
  {
    for (int i = 0; i < 1000; i++) 
    {
      // Gravity
      stroke(200,0,0);
      strokeWeight(2);
      point(i,200-gravity_history[i]*100.0,0); // 1d = 1 pixel 
      // MagField
      stroke(0,0,200);
      strokeWeight(2);
      point(i,200-mfield_history[i]*100.0,0); // 1d = 1 pixel 
      // Mag Dip Angle
      stroke(0,0,100);
      strokeWeight(2);
      point(i,200+mdip_history[i]*1.0,0); // 1d = 1 pixel 
      
      // X red
      stroke(255,0,0);
      strokeWeight(3);
      point(i,500-ang_history[i].x/1000.0,0); // 1d = 1 pixel 
      // Y green
      stroke(0,255,0);
      strokeWeight(3);
      point(i,500-ang_history[i].y/1000.0,0); // 1d = 1 pixel 
      // Z blue
      stroke(0,0,255);
      strokeWeight(3);
      point(i,500-ang_history[i].z/1000.0,0); // 1d = 1 pixel 
      // Z gray
      stroke(100,100,255);
      strokeWeight(3);
      point(i,500-ang2_history[i].z/1000.0,0); // 1dps = 1 pixel 
    }
  }
  else
  {
    for (int i = 0; i < iteration; i++) 
    {
      // Gravity
      stroke(200,0,0);
      strokeWeight(2);
      point(i,200-gravity_history[i]*100.0,0); // 1d = 1 pixel 
      // MagField
      stroke(0,0,200);
      strokeWeight(2);
      point(i,200-mfield_history[i]*100.0,0); // 1d = 1 pixel 
      // Mag Dip Angle
      stroke(0,0,100);
      strokeWeight(2);
      point(i,200+mdip_history[i]*1.0,0); // 1d = 1 pixel 
      
      
      // X red
      stroke(255,0,0);
      strokeWeight(3);
      point(i,500-ang_history[i].x/1000.0,0); // 1dps = 1 pixel 
      // Y green
      stroke(0,255,0);
      strokeWeight(3);
      point(i,500-ang_history[i].y/1000.0,0); // 1dps = 1 pixel 
      // Z blue
      stroke(0,0,255);
      strokeWeight(3);
      point(i,500-ang_history[i].z/1000.0,0); // 1dps = 1 pixel 
      // Z gray
      stroke(100,100,255);
      strokeWeight(3);
      point(i,500-ang2_history[i].z/1000.0,0); // 1dps = 1 pixel 
    }
  }
  
  if(iteration>0)
  {
    String pitch =  "Pitch: " + (int)(ang_history[(iteration-1)%1000].x/1000.0) + "deg.";
    stroke(255,0,0);
    fill(255,0,0);
    textFont(font, 30);
    text(pitch, 100, 900, 0);
    String roll =  "Roll: " + (int)(ang_history[(iteration-1)%1000].y/1000.0) + "deg.";
    stroke(0,255,0);
    fill(0,255,0);
    textFont(font, 30);
    text(roll, 400, 900, 0);
    String yaw =  "Yaw: " + (int)(ang_history[(iteration-1)%1000].z/1000.0) + "deg.";
    stroke(0,0,255);
    fill(0,0,255);
    textFont(font, 30);
    text(yaw, 700, 900, 0);
    
    String yawgb =  "GyroBias: " + (int)(gyro_yaw_bias_history[(iteration-1)%1000]*1000.0) + "Raw";
    stroke(100,100,255);
    fill(100,100,255);
    textFont(font, 20);
    text(yawgb, 100, 960, 0);

    String yawgv =  "GyroVariance: " + (int)(gyro_yaw_deviation_history[(iteration-1)%1000]*1000.0) + "Raw";
    stroke(100,100,255);
    fill(100,100,255);
    textFont(font, 20);
    text(yawgv, 400, 960, 0);
    
    String yawg =  "Yaw(Gyro): " + (int)(ang2_history[(iteration-1)%1000].z/1000.0) + "deg.";
    stroke(100,100,255);
    fill(100,100,255);
    textFont(font, 30);
    text(yawg, 700, 960, 0);
    
    {
        String gravity_str =  "Gravity Field: " + gravity_history[(iteration-1)%1000] + "g";
        stroke(200,0,0);
        fill(200,0,0);
        textFont(font, 20);
        text(gravity_str, 10, 220, 0);

        String mfied_str =  "Mag Field: " + mfield_history[(iteration-1)%1000] + "gauss";
        stroke(0,0,200);
        fill(0,0,200);
        textFont(font, 20);
        text(mfied_str, 310, 220, 0);

        String mdip_str =  "Mag Dip Angle: " + mdip_history[(iteration-1)%1000] + "deg.";
        stroke(0,0,100);
        fill(0,0,100);
        textFont(font, 20);
        text(mdip_str, 610, 220, 0);
  }
    // curve legend
    {
        String gravity_str =  "Gravity Field";
        stroke(200,0,0);
        fill(200,0,0);
        textFont(font, 12);
        text(gravity_str, 10, 200-gravity_history[0]*100.0-6, 0);
    
        String mfied_str =  "Mag Field";
        stroke(0,0,200);
        fill(0,0,200);
        textFont(font, 12);
        text(mfied_str, 10, 200-mfield_history[0]*100.0-6, 0);

        String mdip_str =  "Mag Dip Angle";
        stroke(0,0,100);
        fill(0,0,100);
        textFont(font, 12);
        text(mdip_str, 10, 200+mdip_history[0]*1.0-6, 0);
    }
}
}

void serialEvent(Serial p) 
{
  String incoming = p.readString();
  if (printSerial) {
    println(incoming);
  }
  
  if ((incoming.length() > 8))
  {
    String[] list = split(incoming, " ");
      if ( list.length >= 11 ) 
      {
        
        ang = new PVector( float(list[1]), float(list[2]), float(list[3]) );
        ang2 = new PVector( float(list[4]), float(list[4]), float(list[4]) );
        gravity = float(list[5]);
        mfield = float(list[6]);
        gyro_yaw_mean = float(list[7])/1000.0f;
        gyro_yaw_deviation = float(list[8])/1000.0f;
        gyro_yaw_bias = float(list[9])/1000.0f;
        mdip = float(list[10]);
        
       ang_history[iteration%1000] = new PVector(ang.x, ang.y, ang.z);
       ang2_history[iteration%1000] = new PVector(ang2.x, ang2.y, ang2.z);
        gravity_history[iteration%1000] = gravity/1000.0;
        mfield_history[iteration%1000] = mfield/1000.0;
        gyro_yaw_mean_history[iteration%1000] = gyro_yaw_mean/1000.0;
        gyro_yaw_deviation_history[iteration%1000] = gyro_yaw_deviation/1000.0;
        gyro_yaw_bias_history[iteration%1000] = gyro_yaw_bias/1000.0;
        mdip_history[iteration%1000] = mdip/1000.0;
       
       ++iteration;
    }
  }
}

// Set serial port to desired value.
void setSerialPort(String portName) {
  // Close the port if it's currently open.
  if (port != null) {
    port.stop();
  }
  try {
    // Open port.
    port = new Serial(this, portName, 115200);
    port.bufferUntil('\n');
    // Persist port in configuration.
    saveStrings(serialConfigFile, new String[] { portName });
  }
  catch (RuntimeException ex) {
    // Swallow error if port can't be opened, keep port closed.
    port = null; 
  }
}

// UI event handlers

void handlePanelEvents(GPanel panel, GEvent event) {
  // Panel events, do nothing.
}

void handleDropListEvents(GDropList list, GEvent event) { 
  // Drop list events, check if new serial port is selected.
  if (list == serialList) {
    setSerialPort(serialList.getSelectedText()); 
  }
}

void handleToggleControlEvents(GToggleControl checkbox, GEvent event) { 
  // Checkbox toggle events, check if print events is toggled.
  if (checkbox == printSerialCheckbox) {
    printSerial = printSerialCheckbox.isSelected(); 
  }
}
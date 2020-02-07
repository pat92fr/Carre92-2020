import processing.serial.*;
import java.awt.datatransfer.*;
import java.awt.Toolkit;
import processing.opengl.*;
import saito.objloader.*;
import g4p_controls.*;

float m[]  = {0.0F,0.0F,0.0F};
float m_min[]  = {0.0F,0.0F,0.0F};
float m_max[]  = {0.0F,0.0F,0.0F};
float m_offset[]  = {0.0F,0.0F,0.0F};
float m_amplitude[]  = {0.0F,0.0F,0.0F};
float m_scaled[]  = {0.0F,0.0F,0.0F};
String m_name[]  = {"mx","my","mz"};

int iteration  = 0;
PVector[] vecs = new PVector[20000];

PFont font;

boolean      printOffsetCorrection = true;


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
  translate(500,500);
  //scale(0.1, -0.1, 0.1); //Flip Y (mag Z axis)
  stroke(0,0,0);
  strokeWeight(1);
  line (0, 0, 0, 500, 0, 0);
  line (0, 0, 0, -500, 0, 0);
  line (0, 0, 0, 0, 500, 0);
  line (0, 0, 0, 0, -500, 0);

  PVector voffset = new PVector(m_offset[0],m_offset[1],m_offset[2]);
  
  for (int i = 0; i < iteration; i++) {
    PVector vi = new PVector(vecs[i].x,vecs[i].y,vecs[i].z);
  PVector v;
  if(printOffsetCorrection)
    v = vi.sub(voffset);
    else
    v = vi;
    
    // XY red
    stroke(255,0,0);
    strokeWeight(3);
    point(v.x/10, v.y/10, 0);

    // YZ blue
    stroke(0,255,0);
    strokeWeight(3);
    point(v.y/10, v.z/10, 0);

    // XZ green
    stroke(0,0,255);
    strokeWeight(3);
    point(v.x/10, v.z/10, 0);
  }
  
  String offset =  "Offset (x/y/z) : " + m_offset[0] + ", " + m_offset[1] + ", " + m_offset[2];
  stroke(0,0,0);
  fill(0,0,0);
  textFont(font, 20);
  text(offset, 100, 480, 0);

  {
    // actual calibrated magnometer XY values
    float cm[] = { m[0]-m_offset[0], m[1]-m_offset[1]};
    float heading;
    if(cm[0]==0)
    {
      if(cm[1]>0)
      {
        heading = 90;
      }
      else
      {
        heading = 270;
      }
    }
    else
    {
      heading = atan(cm[1]/cm[0])*180.0/PI;
      if(cm[0]>0 && cm[0]>=0)
      {
        heading = atan(cm[1]/cm[0])*180.0/PI;
      }
      else if(cm[0]<0 && cm[0]>=0)
      {
        heading = 180-atan(cm[1]/-cm[0])*180.0/PI;
      }
      else if(cm[0]<0 && cm[0]<0)
      {
        heading = 180+atan(cm[1]/cm[0])*180.0/PI;
      }
      else if(cm[0]>0 && cm[0]<0)
      {
        heading = -atan(cm[1]/cm[0])*180.0/PI;
      }
      if(heading>360)
        heading -= 360;
      if(heading<0)
        heading += 360;
    }
    String heading_str =  "Heading XY : " + (int)heading + "°";
    stroke(0,0,0);
    fill(100,0,0);
    textFont(font, 20);
    text(heading_str, 300, -480, 0);
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
      if ( list.length == 9 ) 
      {
        
        m[0] = float(list[6]);
        m[1] = float(list[7]);
        m[2] = float(list[8]);
        if(iteration<20000)
          vecs[iteration] = new PVector(m[0], m[1], m[2]);
        
          if(iteration==0)
          {
            for(int index=0; index<3; ++index)
            {
              m_min[index]  = m[index];
              m_max[index]  = m[index];
            }
            ++iteration;
          }
          else 
          {
            for(int index=0; index<3; ++index)
            {
              if(m_min[index]>m[index]) m_min[index]  = m[index];
              if(m_max[index]<m[index]) m_max[index]  = m[index];
              m_offset[index] = (m_max[index]+m_min[index])/2.0;
              m_amplitude[index] = (m_max[index]-m_min[index])/2.0;
              m_scaled[index] = (m[index] - m_offset[index])/m_amplitude[index];
            }
            ++iteration;
          }
        for(int index=0; index<3; ++index)
        {
          print("[");
          print(iteration);
          print("] ");
          print(m_name[index]);
          print(": ");
          print(m_min[index]);
          print(" , ");
          print(m_max[index]);
          print(" , ");
          print(m_offset[index]);
          print(" , ");
          print(m_amplitude[index]);
          print(" > ");
          print(m_scaled[index]);
          println();
        }

        
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
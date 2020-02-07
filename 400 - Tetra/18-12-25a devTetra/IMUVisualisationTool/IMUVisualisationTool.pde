import processing.serial.*;
import java.awt.datatransfer.*;
import java.awt.Toolkit;
import processing.opengl.*;
import saito.objloader.*;
import g4p_controls.*;

PVector a  = new PVector(0.0F,0.0F,0.0F);
PVector g  = new PVector(0.0F,0.0F,0.0F);
PVector m  = new PVector(0.0F,0.0F,0.0F);
PVector gvariance  = new PVector(0.0F,0.0F,0.0F);

int iteration  = 0;
PVector[] a_history = new PVector[1000];
PVector[] g_history = new PVector[1000];
PVector[] m_history = new PVector[1000];
PVector[] gvariance_history = new PVector[1000];

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
  line (0, 250, 0, 1000, 250, 0);
  line (0, 500, 0, 1000, 500, 0);
  line (0, 750, 0, 1000, 750, 0);
  if(iteration>1)
    {
        String gvariancey_str =  "GYRO DEVIATION : " + (gvariance_history[(iteration-1)%1000]) + " mRaw";
        stroke(0,0,255);
        fill(0,0,255);
        textFont(font, 18);
        text(gvariancey_str, 500, 550, 0);
    }  
  if(iteration>1000)
  {
  
    for (int i = 0; i < 1000; i++) 
    {
      // X red
      stroke(255,0,0);
      strokeWeight(3);
      point(i,250-a_history[i].x/10.0,0); // 1g = 100 pixels
      point(i,500-g_history[i].x/1000.0,0); // 1dps = 1 pixel 
      point(i,750-m_history[i].x/4.0,0); // 
      // Y green
      stroke(0,255,0);
      strokeWeight(3);
      point(i,250-a_history[i].y/10.0,0); // 1g = 100 pixels
      point(i,500-g_history[i].y/1000.0,0); // 1dps = 1 pixel 
      point(i,750-m_history[i].y/4.0,0); // 
      // Z blue
      stroke(0,0,255);
      strokeWeight(3);
      point(i,250-a_history[i].z/10.0,0); // 1g = 100 pixels
      point(i,500-g_history[i].z/1000.0,0); // 1dps = 1 pixel 
      point(i,750-m_history[i].z/4.0,0); // 
    }
  }
  else
  {
for (int i = 0; i < iteration; i++) 
    {
      // X red
      stroke(255,0,0);
      strokeWeight(3);
      point(i,250-a_history[i].x/10.0,0); // 1g = 100 pixels
      point(i,500-g_history[i].x/1000.0,0); // 1dps = 1 pixel 
      point(i,750-m_history[i].x/2.0,0); // 
      // Y green
      stroke(0,255,0);
      strokeWeight(3);
      point(i,250-a_history[i].y/10.0,0); // 1g = 100 pixels
      point(i,500-g_history[i].y/1000.0,0); // 1dps = 1 pixel 
      point(i,750-m_history[i].y/2.0,0); // 
      // Z blue
      stroke(0,0,255);
      strokeWeight(3);
      point(i,250-a_history[i].z/10.0,0); // 1g = 100 pixels
      point(i,500-g_history[i].z/1000.0,0); // 1dps = 1 pixel 
      point(i,750-m_history[i].z/2.0,0); // 
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
      if ( list.length == 12 ) 
      {
        
        a = new PVector( float(list[0]), float(list[1]), float(list[2]) );
        g = new PVector( float(list[3]), float(list[4]), float(list[5]) );
        m = new PVector( float(list[6]), float(list[7]), float(list[8]) );
        gvariance = new PVector( float(list[9]), float(list[10]), float(list[11]) );
        
       a_history[iteration%1000] = new PVector(a.x, a.y, a.z);
       g_history[iteration%1000] = new PVector(g.x, g.y, g.z);
       m_history[iteration%1000] = new PVector(m.x, m.y, m.z);
       gvariance_history[iteration%1000] = new PVector(gvariance.x, gvariance.y, gvariance.z);
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
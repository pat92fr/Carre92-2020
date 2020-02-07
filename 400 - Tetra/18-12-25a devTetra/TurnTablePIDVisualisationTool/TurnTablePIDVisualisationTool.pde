import processing.serial.*;
import java.awt.datatransfer.*;
import java.awt.Toolkit;
import processing.opengl.*;
import saito.objloader.*;
import g4p_controls.*;

Float dps_consigne;
Float dps;
Float dps_filtered;
Float pid;
Float gyro_rate_mean = 0.0f;

int iteration  = 0;
Float[] dps_consigne_history = new Float[1000];
Float[] dps_history = new Float[1000];
Float[] dps_filtered_history = new Float[1000];
Float[] gyro_yaw_bias_history = new Float[1000];
Float[] pid_history = new Float[1000];
Float[] gyro_rate_mean_history = new Float[1000];

Float offset = 850.0;
Float zoom = 3.0;

PFont font;

// Serial port state.
Serial       port;
Serial       port2;
final String serialConfigFile = "serialconfig.txt";
final String serialConfigFile2 = "serialconfig2.txt";
boolean      printSerial = false;

// UI controls.
GPanel    configPanel;
GDropList serialList;
GLabel    serialLabel;
GCheckbox printSerialCheckbox;
GSlider slider;

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
  port2 = new Serial(this, "COM3", 115200);
  port2.bufferUntil('\n');
  //
  slider = new GSlider(this,10,900,980,20,18);
  slider.setLimits(10.0, 0.0, 260.0);
  slider.setNumberFormat(G4P.DECIMAL, 2);
  slider.setOpaque(false);
  slider.addEventHandler(this, "slider1_change1");
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
      point(i,offset-dps_consigne_history[i]*zoom,0);  
      // dps
      stroke(0,0,200);
      strokeWeight(1);
      point(i,offset-dps_history[i]*zoom,0);  
      // dps filtered
      stroke(0,0,255);
      strokeWeight(3);
      point(i,offset-dps_filtered_history[i]*zoom,0);  
      // pwm
      stroke(0,255,0);
      strokeWeight(3);
      point(i,offset-pid_history[i]*zoom,0);  
      // robot dps
      stroke(255,0,0);
      strokeWeight(3);
      point(i,offset-gyro_rate_mean_history[i]*zoom/1000.0,0);   
    }
  }
  else
  {
    for (int i = 0; i < iteration; i++) 
    {
      // dps consigne
      stroke(0,0,0);
      strokeWeight(2);
      point(i,offset-dps_consigne_history[i]*zoom,0);  
      // dps
      stroke(0,0,200);
      strokeWeight(1);
      point(i,offset-dps_history[i]*zoom,0);  
      // dps
      stroke(0,0,255);
      strokeWeight(3);
      point(i,offset-dps_filtered_history[i]*zoom,0);  
      // pwm
      stroke(0,255,0);
      strokeWeight(3);
      point(i,offset-pid_history[i]*zoom,0);  
      // robot dps
      stroke(255,0,0);
      strokeWeight(3);
      point(i,offset-gyro_rate_mean_history[i]*zoom/1000.0,0);  

    }
  }
  
  if(iteration>0)
  {
    // curve legend
    {
        String dps_consigne_str =  "TARGET DPS";
        stroke(0,0,0);
        fill(0,0,0);
        textFont(font, 12);
        text(dps_consigne_str, 10, offset-dps_consigne_history[0]*zoom-30, 0);
        String dps_str =  "TURNTABLE ACTUAL DPS";
        stroke(0,0,255);
        fill(0,0,255);
        textFont(font, 12);
        text(dps_str, 10, offset-dps_filtered_history[0]*zoom-15, 0);
        String pid_str =  "TURNTABLE MOTOR (PWM)";
        stroke(0,255,0);
        fill(0,255,0);
        textFont(font, 12);
        text(pid_str, 10, offset-pid_history[0]*zoom-6, 0);
        String robot_str =  "ROBOT MOTOR (PWM)";
        stroke(255,0,0);
        fill(255,0,0);
        textFont(font, 12);
        text(robot_str, 10, offset-gyro_rate_mean_history[0]*zoom-6, 0);
      }
  // Afficheur
  {
        String dps_consigne_str =  "TARGET DPS : " + (dps_consigne_history[(iteration-1)%1000]);
        stroke(0,0,0);
        fill(0,0,0);
        textFont(font, 28);
        text(dps_consigne_str, 10, 150, 0);
        String dps_str =  "TURNTABLE DPS : " + (dps_filtered_history[(iteration-1)%1000]);
        stroke(0,0,255);
        fill(0,0,255);
        textFont(font, 28);
        text(dps_str, 300, 150, 0);
        String pid_str =  "ROBOT DPS : " + (gyro_rate_mean_history[(iteration-1)%1000])/1000.0f;
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
  if (printSerial) {
    println(incoming);
  }
  if(p==port)
  {
    if ((incoming.length() > 8))
    {
      String[] list = split(incoming, " ");
        if ( list.length >= 4 ) 
        {
          dps_consigne = float(list[0]);
          dps = float(list[1]);
          dps_filtered = float(list[2]);
          pid = float(list[3]);
          
          dps_consigne_history[iteration%1000] = dps_consigne;
          dps_history[iteration%1000] = dps;
          dps_filtered_history[iteration%1000] = dps_filtered;
          pid_history[iteration%1000] = pid;
          gyro_rate_mean_history[iteration%1000] = gyro_rate_mean;
         
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
          gyro_rate_mean = float(list[7]); //<>//
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

void slider1_change1(GSlider source, GEvent event)
{
  if(source==slider)
  {
    String str = ""+ slider.getValueI() + "\n"; 
    port.write(str);
    println(str);
  }
}
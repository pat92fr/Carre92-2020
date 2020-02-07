import processing.serial.*;
import java.awt.datatransfer.*;
import java.awt.Toolkit;
import processing.opengl.*;
import saito.objloader.*;
import g4p_controls.*;

PVector g  = new PVector(0.0F,0.0F,0.0F);
PVector g_offset  = new PVector(0.0F,0.0F,0.0F);
PVector g_offset_square  = new PVector(0.0F,0.0F,0.0F);
PVector g_variance  = new PVector(0.0F,0.0F,0.0F);

float alpha = 0.0005;

int iteration  = 0;
PVector[] g_history = new PVector[20000];

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
  line (-500, -250, 0, 500, -250, 0);
  line (-500, 0, 0, 500, 0, 0);
  line (-500, 250, 0, 500, 250, 0);
  
  line (0, 0, 0, -500, 0, 0);
  line (0, 0, 0, 0, 500, 0);
  line (0, 0, 0, 0, -500, 0);

  for (int i = 0; i < iteration; i++) 
  {
    // X red
    stroke(255,0,0);
    strokeWeight(3);
    point((i%1000-500),(g_history[i].x-g_offset.x)-250,0);
    // Y green
    stroke(0,255,0);
    strokeWeight(3);
    point((i%1000-500),(g_history[i].y-g_offset.y)+0,0);
    // Z blue
    stroke(0,0,255);
    strokeWeight(3);
    point((i%1000-500),(g_history[i].z-g_offset.z)+250,0);
  }
  
  String offset =  "Offset (x/y/z) : " + g_offset.x + ", " + g_offset.y + ", " + g_offset.z;
  stroke(0,0,0);
  fill(0,0,0);
  textFont(font, 20);
  text(offset, 20, 440, 0);

  String variance =  "Variance (x/y/z) : " + g_variance.x + ", " + g_variance.y + ", " + g_variance.z;
  stroke(0,0,0);
  fill(0,0,0);
  textFont(font, 20);
  text(variance, 20, 480, 0);

  //{
  //  // actual calibrated magnometer XY values
  //  float cm[] = { m[0]-m_offset[0], m[1]-m_offset[1]};
  //  float heading;
  //  if(cm[0]==0)
  //  {
  //    if(cm[1]>0)
  //    {
  //      heading = 90;
  //    }
  //    else
  //    {
  //      heading = 270;
  //    }
  //  }
  //  else
  //  {
  //    heading = atan(cm[1]/cm[0])*180.0/PI;
  //    if(cm[0]>0 && cm[0]>=0)
  //    {
  //      heading = atan(cm[1]/cm[0])*180.0/PI;
  //    }
  //    else if(cm[0]<0 && cm[0]>=0)
  //    {
  //      heading = 180-atan(cm[1]/-cm[0])*180.0/PI;
  //    }
  //    else if(cm[0]<0 && cm[0]<0)
  //    {
  //      heading = 180+atan(cm[1]/cm[0])*180.0/PI;
  //    }
  //    else if(cm[0]>0 && cm[0]<0)
  //    {
  //      heading = -atan(cm[1]/cm[0])*180.0/PI;
  //    }
  //    if(heading>360)
  //      heading -= 360;
  //    if(heading<0)
  //      heading += 360;
  //  }
  //  String heading_str =  "Heading XY : " + (int)heading + "°";
  //  stroke(0,0,0);
  //  fill(100,0,0);
  //  textFont(font, 20);
  //  text(heading_str, 300, -480, 0);
  //}

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
        // decoding "ax ay az gx gy gz mx my mz"
        g = new PVector( float(list[3]), float(list[4]), float(list[5]) );
        
        if(iteration<20000)
          g_history[iteration] = new PVector(g.x, g.y, g.z);
          ++iteration;
        
          if(iteration==100)
          {
            g_offset = new PVector(0.0f, 0.0f, 0.0f);
            g_offset_square = new PVector(0.0f, 0.0f, 0.0f);
            for(int it=0; it<iteration; ++it)
            {
              g_offset.add( g_history[it] );
              PVector g_square = new PVector( g_history[it].x*g_history[it].x, g_history[it].y*g_history[it].y, g_history[it].z*g_history[it].z);
              g_offset_square.add( g_square );
            }
            g_offset.div( 100.0f );
            g_offset_square.div( 100.0f );
            
          }
          else if(iteration>100) 
          {
            g_offset.x = g.x*(alpha) + (1-alpha)*g_offset.x;
            g_offset.y = g.y*(alpha) + (1-alpha)*g_offset.y;
            g_offset.z = g.z*(alpha) + (1-alpha)*g_offset.z;
            g_offset_square.x = g.x*g.x*(alpha) + (1-alpha)*g_offset_square.x;
            g_offset_square.y = g.y*g.y*(alpha) + (1-alpha)*g_offset_square.y;
            g_offset_square.z = g.z*g.z*(alpha) + (1-alpha)*g_offset_square.z;
            g_variance = new PVector(sqrt(g_offset_square.x-g_offset.x*g_offset.x),sqrt(g_offset_square.y-g_offset.y*g_offset.y),sqrt(g_offset_square.z-g_offset.z*g_offset.z));
          }
        {
          print("[");
          print(iteration);
          print("] ");
          print(g_offset.x);
          print(" , ");
          print(g_offset.y);
          print(" , ");
          print(g_offset.z);
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
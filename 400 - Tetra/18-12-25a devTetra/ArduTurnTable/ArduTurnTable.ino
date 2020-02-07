#include <EEPROM.h>
#include <Encoder.h>

// Encoder pulses to mm
Encoder myEnc(2, 3);
long divider = 1920;
long newPosition = 0;
long oldPosition  = 0;

// Standard PWM DC control
int M1PWM = 9;     //M1 Speed Control
int M1DIR = 7;    //M1 Direction Control
int _nD2 = 4;

void stop(void)
{
  digitalWrite(M1PWM, LOW);
}

void turn(char a)
{
  analogWrite(M1PWM, a);     //PWM Speed Control
}

void setup() {
  pinMode(M1DIR,OUTPUT);
  digitalWrite(M1DIR, HIGH);
  pinMode(M1PWM,OUTPUT);
  digitalWrite(M1PWM, HIGH);
  pinMode(_nD2,OUTPUT);
  digitalWrite(_nD2,HIGH); // default to on
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
}

String inString = "";    // string to hold input
long dps_finale = 0;
long dps_consigne = 0;
float dps = 0.0f;
float dps_filtered= 0.0f;
float kp = 2400, ki = 1, kd = 0, windup_limit = 20000; // 1000,30,0
float p = 0, i = 0, d = 0, pid = 0;
float e = 0;
float old_e = 0;
 
void loop()
{
  newPosition = myEnc.read();
  dps = (float)(newPosition-oldPosition)*360.0*50.0/(float)divider;
  dps_filtered = 0.8 * dps_filtered + 0.2*dps;
  oldPosition = newPosition;
  Serial.print(dps_consigne);
  Serial.print(" ");
  Serial.print(dps);
  Serial.print(" ");
  Serial.print(dps_filtered);
  Serial.print(" ");
  Serial.print(pid/1000);
  Serial.println();
  if(dps_finale>0 && dps_finale <=1000)
  {
    if(dps_consigne<dps_finale)
    ++dps_consigne;
    if(dps_consigne>dps_finale)
    --dps_consigne;
    e = (float)dps_consigne-dps_filtered;
    p = e*kp;
    i += e*ki;
    if(i>windup_limit) i=windup_limit;
    if(i<-windup_limit) i=-windup_limit;
    d = (e-old_e)*kd;
    pid = 0.0*pid+1.0*(p + i + d);
    old_e = e;
    if(pid<=0)
      stop();
    else if(pid/1000>255) // max power
      turn(255);
    else
     turn(pid/1000);
  }
  else
  {
    dps_consigne = 0;
    stop();
  }
  delay(20);
  
// Read serial input:
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
    // if you get a newline, print the string, then the string's value:
    if (inChar == '\n') {
      dps_finale = inString.toInt();
      Serial.print("dps_consigne:");
      Serial.println(dps_finale);
      // clear the string for new input:
      inString = "";
    }
  }
}

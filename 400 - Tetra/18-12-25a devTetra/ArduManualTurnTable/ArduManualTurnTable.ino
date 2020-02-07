#include <Encoder.h>

// Encoder pulses to mm
Encoder myEnc(2, 3);
long divider = 1920;
long currentPosition = 0;
float currentAngle = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
}

String inString = "";    // string to hold input

 
void loop()
{
  currentPosition = myEnc.read();
  currentAngle = (float)currentPosition*360.0/(float)divider;
  Serial.print(currentPosition);
  Serial.print(" ");
  Serial.print(currentAngle);
  Serial.println();
  delay(20);
 
}

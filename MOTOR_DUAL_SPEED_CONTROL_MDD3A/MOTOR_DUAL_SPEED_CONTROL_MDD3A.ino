#include "CytronMotorDriver.h"

CytronMD motor1(PWM_PWM, 6, 7);   
CytronMD motor2(PWM_PWM, 5, 4);

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  inputString.reserve(200);
}

int s = 0;
void loop() {
  // put your main code here, to run repeatedly:
  if (stringComplete) {
//    Serial.println(inputString);
    // clear the string:
    stringComplete = false;
    s = inputString.toInt();
    Serial.println(s);
    inputString = "";
  }

  s = s > 255 ? 255 : s;
  s = s < -255 ? -255: s;

  motor1.setSpeed(s);
  motor2.setSpeed(s);
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
      return;
    }
    inputString += inChar;
  }
}

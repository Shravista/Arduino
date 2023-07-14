#include "CytronMotorDriver.h"
#include <util/atomic.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#define MOTOR1_CHA 2
#define MOTOR1_CHB 3
#define MOTOR2_CHA 18
#define MOTOR2_CHB 19

#define PPR1 560.0f
#define GEAR_RATIO1 20.0f

#define PPR2 1680.0f
#define GEAR_RATIO2 60.0f

CytronMD motor1(PWM_PWM, 6, 7);   
CytronMD motor2(PWM_PWM, 5, 4);
Encoder MA(MOTOR1_CHA,MOTOR1_CHB);

Encoder MB(MOTOR2_CHA,MOTOR2_CHB);

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

int pos1_last = 0;
int pos2_last = 0;
volatile long int pos1 = 0;
volatile long int pos2 = 0;
int t1 = 0, t2 = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  inputString.reserve(200);

  pinMode(MOTOR1_CHA, INPUT);
  pinMode(MOTOR1_CHB, INPUT);

  pinMode(MOTOR2_CHA, INPUT);
  pinMode(MOTOR2_CHB, INPUT);

//  attachInterrupt(digitalPinToInterrupt(MOTOR1_CHA), readEncoder1, RISING);
//  attachInterrupt(digitalPinToInterrupt(MOTOR2_CHA), readEncoder2, RISING);

//  attachInterrupt(digitalPinToInterrupt(MOTOR1_CHA), readEncoder1, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(MOTOR1_CHB), readEncoder1, CHANGE);
//  
//  attachInterrupt(digitalPinToInterrupt(MOTOR2_CHA), readEncoder2, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(MOTOR2_CHB), readEncoder1, CHANGE);
  t1 = t2 = millis();
}

int s = 0;
float n1 = 0.0, n1_last = 0.0;
float n2 = 0.0, n2_last = 0.0;

void loop() {
  // put your main code here, to run repeatedly:
  t1 = millis();
  int M1_pos = MA.read();
  int M2_pos = MB.read();
  int dt = (t1-t2);
  t2 = t1;

  n1 = (float) M1_pos/(PPR1);
  n2 = (float) M2_pos/(PPR2);

  float rpm1 = (n1 - n1_last)/dt*60000;
  float rpm2 = (n2 - n2_last)/dt*60000;

  if (stringComplete) {
    stringComplete = false;
    s = inputString.toInt();
    inputString = "";
  }

  s = s > 255 ? 255 : s;
  s = s < -255 ? -255: s;

  motor1.setSpeed(s);
  motor2.setSpeed(s);
//  pos1 = MA.read();
  String str = "pos1 = " + String(n1) + " RPM1 = " + String(rpm1) + " pos2 = " + String(n2) + " RPM2 = " + String(rpm2);
  Serial.println(str);

  n1_last = n1;
  n2_last = n2;
  delay(10);
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

//void readEncoder1(){
//  int b = digitalRead(MOTOR1_CHB);
//  if (b > 0){
//    pos1++;
//  } else {
//    pos1--;
//  }
//}
//
//void readEncoder2(){
//  int b = digitalRead(MOTOR2_CHB);
//  if (b > 0){
//    pos2++;
//  } else {
//    pos2--;
//  }
//}

//void readEncoder1() {
//  int MSB = digitalRead(MOTOR1_CHA);
//  int LSB = digitalRead(MOTOR1_CHB);
//
//  int encoded = (MSB << 1)| LSB;
//  int sum = (pose1_last << 2) | encoded;
//
//  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) pos1++;
//  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) pos1--;
//
//  pose1_last = encoded;
//}
//
//void readEncoder2() {
//  int MSB = digitalRead(MOTOR2_CHA);
//  int LSB = digitalRead(MOTOR2_CHB);
//
//  int encoded = (MSB << 1)| LSB;
//  int sum = (pose2_last << 2) | encoded;
//
//  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) pos2++;
//  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) pos2--;
//
//  pose2_last = encoded;
//}

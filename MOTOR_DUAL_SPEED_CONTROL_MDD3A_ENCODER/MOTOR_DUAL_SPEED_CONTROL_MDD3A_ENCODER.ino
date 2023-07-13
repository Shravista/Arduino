#include "CytronMotorDriver.h"
#include <util/atomic.h>
//#define ENCODER_OPTIMIZE_INTERRUPTS
//#include <Encoder.h>

#define MOTOR1_CHA 2
#define MOTOR1_CHB 3
#define MOTOR2_CHA 18
#define MOTOR2_CHB 19

CytronMD motor1(PWM_PWM, 6, 7);   
CytronMD motor2(PWM_PWM, 5, 4);
//Encoder MA(MOTOR1_CHA,MOTOR1_CHB);

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

volatile int pose_last = 0;
volatile long int pos1 = 0;
volatile long int pos2 = 0;

long int 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  inputString.reserve(200);

  pinMode(MOTOR1_CHA, INPUT);
  pinMode(MOTOR1_CHB, INPUT);

  pinMode(MOTOR2_CHA, INPUT);
  pinMode(MOTOR2_CHB, INPUT);

  attachInterrupt(digitalPinToInterrupt(MOTOR1_CHA), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_CHA), readEncoder2, RISING);
}

int s = 0;
void loop() {
  // put your main code here, to run repeatedly:
  int M1_pos = 0;
  int M2_pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    M1_pos = pos1;
    M2_pos = pos2;
  }
  if (stringComplete) {
//    Serial.println(inputString);
    // clear the string:
    stringComplete = false;
    s = inputString.toInt();
//    Serial.println(s);
    inputString = "";
  }

  s = s > 255 ? 255 : s;
  s = s < -255 ? -255: s;

  motor1.setSpeed(s);
  motor2.setSpeed(s);
//  pos1 = MA.read();
  String str = "pos1 = " + String(M1_pos) + " pos2 = " + String(M2_pos);
  Serial.println(str);
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

void readEncoder1(){
  int b = digitalRead(MOTOR1_CHB);
  if (b > 0){
    pos1++;
  } else {
    pos1--;
  }
}

void readEncoder2(){
  int b = digitalRead(MOTOR2_CHB);
  if (b > 0){
    pos2++;
  } else {
    pos2--;
  }
}

//void readEncoder() {
//  int MSB = digitalRead(MOTOR1_CHA);
//  int LSB = digitalRead(MOTOR1_CHB);
//
//  int encoded = (MSB << 1)| LSB;
//  int sum = (pose_last << 2) | encoded;
//
//  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) pos1++;
//  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) pos1--;
//
//  pose_last = encoded;
//}

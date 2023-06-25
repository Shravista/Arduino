#include <util/atomic.h>

#define MOTOR_EN 10
#define MOTOR_IN1 8
#define MOTOR_IN2 11
#define MOTOR_CHA 2
#define MOTOR_CHB 3

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
volatile int posi = 0;

void setup() {
  // initialize serial:
  Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  pinMode(MOTOR_EN, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_CHA, INPUT);
  pinMode(MOTOR_CHB, INPUT);

  attachInterrupt(digitalPinToInterrupt(MOTOR_CHA), readEncoder, RISING);
}

int speed = 0;

void loop() {
  // print the string when a newline arrives:
  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = posi;
  }
  if (stringComplete) {
    //Serial.println(inputString);
    // clear the string:
    stringComplete = false;
    speed = inputString.toInt();
    //Serial.println(speed);
    inputString = "";
  }
  speed = speed > 255 ? 255 : speed;
  speed = speed < 0 ? 0: speed;
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_EN, speed);

  String msg = String(pos) + ", pwm = " + String(speed);
  Serial.println(msg);
}

void readEncoder(){
  int b = digitalRead(MOTOR_CHB);
  if (b > 0){
    posi++;
  } else {
    posi--;
  }
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
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

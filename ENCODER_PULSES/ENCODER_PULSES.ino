#include <util/atomic.h>

#define MOTOR_CHA 2
#define MOTOR_CHB 3

#define CPR 405.0f

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
volatile long int posi = 0;
long int posCurrent = 0;
long int posPrevious = 0;
long int currentTime = 0;

void setup() {
  // initialize serial:
  Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  pinMode(MOTOR_CHA, INPUT);
  pinMode(MOTOR_CHB, INPUT);

  attachInterrupt(digitalPinToInterrupt(MOTOR_CHA), readEncoder, RISING);
  currentTime = millis();
}

int speed = 0;

void loop() {
  // print the string when a newline arrives:
  long int dt = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    posCurrent = posi;
  }

  String msg = "count = " + String(posCurrent);
  Serial.println(msg);
  delay(10);
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

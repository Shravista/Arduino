#define MOTORA_EN 4
#define MOTORA_IN1 5
#define MOTORA_IN2 7

#define MOTORB_EN 11
#define MOTORB_IN1 6
#define MOTORB_IN2 10

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

void setup() {
  // initialize serial:
  Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  pinMode(MOTORA_EN, OUTPUT);
  pinMode(MOTORA_IN1, OUTPUT);
  pinMode(MOTORA_IN2, OUTPUT);

  pinMode(MOTORB_EN, OUTPUT);
  pinMode(MOTORB_IN1, OUTPUT);
  pinMode(MOTORB_IN2, OUTPUT);
}

int speed = 0;

void loop() {
  // print the string when a newline arrives:
  if (stringComplete) {
    //Serial.println(inputString);
    // clear the string:
    stringComplete = false;
    speed = inputString.toInt();
    Serial.println(speed);
    inputString = "";
  }
  speed = speed > 255 ? 255 : speed;
  speed = speed < 0 ? 0: speed;
  digitalWrite(MOTORA_IN1, HIGH);
  digitalWrite(MOTORA_IN2, LOW);
  analogWrite(MOTORA_EN, speed);

  digitalWrite(MOTORB_IN1, HIGH);
  digitalWrite(MOTORB_IN2, LOW);
  analogWrite(MOTORB_EN, speed);
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

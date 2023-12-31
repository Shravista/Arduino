#define MOTORA_EN 4
#define MOTORA_IN1 5
#define MOTORA_IN2 7

#define MOTORB_EN 11
#define MOTORB_IN1 6
#define MOTORB_IN2 10
#define LED_PIN 12

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

  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // print the string when a newline arrives:
  String MA_dir, MB_dir;
  int MA_pwm, MB_pwm;
  auto val = LOW;
  if (stringComplete) {
    if (inputString.length() == 8) {
      MA_dir = String(inputString[0]);
      MB_dir = String(inputString[4]);  
      String val1, val2;
      for (int i = 0; i < 3; i++){
        val1 += inputString[1+i];
        val2 += inputString[5+i];
      }
      MA_pwm = val1.toInt();
      MB_pwm = val2.toInt();
    }
    val = HIGH;
    stringComplete = false;
    inputString = "";
  }

  if ((MA_dir.length() == 1) && (MB_dir.length() == 1)){
    if ((MA_pwm == 0) || (MB_pwm == 0)) {
      val = LOW;
    }
    digitalWrite(LED_PIN, val);
    setMotorA(MA_dir, MA_pwm);
    setMotorB(MB_dir, MB_pwm);
  }
}

void setMotorA(String dir, int pwm){
  pwm = pwm > 255 ? 255 : pwm;
  pwm = pwm < 0 ? 0 : pwm;

  if (dir == "f"){
    digitalWrite(MOTORA_IN1, LOW);
    digitalWrite(MOTORA_IN2, HIGH);
  } else if (dir == "r"){
    digitalWrite(MOTORA_IN1, HIGH);
    digitalWrite(MOTORA_IN2, LOW);
  } else {
    // default
    digitalWrite(MOTORA_IN1, LOW);
    digitalWrite(MOTORA_IN2, LOW);
  }
  analogWrite(MOTORA_EN, pwm);
}

void setMotorB(String dir, int pwm){
  pwm = pwm > 255 ? 255 : pwm;
  pwm = pwm < 0 ? 0 : pwm;

  if (dir == "f"){
    digitalWrite(MOTORB_IN1, LOW);
    digitalWrite(MOTORB_IN2, HIGH);
  } else if (dir == "r"){
    digitalWrite(MOTORB_IN1, HIGH);
    digitalWrite(MOTORB_IN2, LOW);
  } else {
    // defBult
    digitalWrite(MOTORB_IN1, LOW);
    digitalWrite(MOTORB_IN2, LOW);
  }
  analogWrite(MOTORB_EN, pwm);
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

// define left motor pins
#define MLEFT_CHA 2
#define MLEFT_CHB 7
#define MLEFT_PWM 5
#define MLEFT_IN1 50
#define MLEFT_IN2 51

// define right motor pins
#define MRIGHT_CHA 3
#define MRIGHT_CHB 6
#define MRIGHT_PWM 4
#define MRIGHT_IN1 48
#define MRIGHT_IN2 49

// initialization of position of encoder
int pos_left = 0;
int pos_right = 0;

void setup() {
  Serial.begin(9600);
  pinMode(MLEFT_CHA, INPUT);
  pinMode(MLEFT_CHB, INPUT);
  pinMode(MLEFT_IN1, OUTPUT);
  pinMode(MLEFT_IN2, OUTPUT);
  pinMode(MLEFT_PWM, OUTPUT);

  pinMode(MRIGHT_CHA, INPUT);
  pinMode(MRIGHT_CHB, INPUT);
  pinMode(MRIGHT_IN1, OUTPUT);
  pinMode(MRIGHT_IN2, OUTPUT);
  pinMode(MRIGHT_PWM, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(MLEFT_CHA), readEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(MRIGHT_CHA), readEncoderRight, RISING);

  analogWrite(MLEFT_PWM,0);
  analogWrite(MRIGHT_PWM,0);
  digitalWrite(MLEFT_IN1, LOW);
  digitalWrite(MLEFT_IN2, LOW);
  digitalWrite(MRIGHT_IN1, LOW);
  digitalWrite(MRIGHT_IN2,  LOW);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  setMotor(0.5, MLEFT_PWM, MLEFT_IN1, MLEFT_IN2);
  setMotor(0.5, MRIGHT_PWM, MRIGHT_IN1, MRIGHT_IN2);
  Serial.print(1000);
  Serial.print(" ");
  Serial.print(pos_left);
  Serial.print(" ");
  Serial.print(pos_right);
  Serial.println(" ");
}

void readEncoderLeft(){
  int b = digitalRead(MLEFT_CHB);
  if (b>0){
    pos_left++;
  }else {
    pos_left--;
  }
}

void readEncoderRight(){
  int b = digitalRead(MRIGHT_CHB);
  if (b>0){
    pos_right++;
  }else {
    pos_right--;
  }
}

void setMotor(float pwrFactor, unsigned int pwm, unsigned int in1,unsigned int in2)
{
  float factor = max(min(pwrFactor,1.0f),-1.0f);
   // people wanted it back motion to be slow
  if (factor >=0){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm, (unsigned int) (255 * factor));
  }else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm, (unsigned int) (255 * factor));
  }
}

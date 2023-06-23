// define left motor pins
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#define MLEFT_CHA 2
#define MLEFT_CHB 3
#define MLEFT_PWM 12
#define MLEFT_IN1 10
#define MLEFT_IN2 11

// define right motor pins
#define MRIGHT_CHA 18
#define MRIGHT_CHB 19
#define MRIGHT_PWM 6
#define MRIGHT_IN1 4
#define MRIGHT_IN2 5

#define   M_PI   3.14159265358979323846 /* pi */

Encoder encLeft(MLEFT_CHA,MLEFT_CHB);
Encoder encRight(MRIGHT_CHB, MRIGHT_CHA);

long int pos_left = 0;
long int pos_right = 0;
float x = 0.0;
float y = 0.0;
float theta = 0.0;
long prevT = 0;
float delS_est_left= 0 , vel_left = 0, err_left = 0, velintegrator =0, velest = 0;
void setup() {
  Serial.begin(9600);
  Serial.println("delS_left:,delS_est_left:,velest:");
//  pinMode(MLEFT_CHA, INPUT);
//  pinMode(MLEFT_CHB, INPUT);
  pinMode(MLEFT_IN1, OUTPUT);
  pinMode(MLEFT_IN2, OUTPUT);
  pinMode(MLEFT_PWM, OUTPUT);

//  pinMode(MRIGHT_CHA, INPUT);
//  pinMode(MRIGHT_CHB, INPUT);
  pinMode(MRIGHT_IN1, OUTPUT);
  pinMode(MRIGHT_IN2,OUTPUT);
  pinMode(MRIGHT_PWM, OUTPUT);
  
//  attachInterrupt(digitalPinToInterrupt(MLEFT_CHA), readEncoderLeft, RISING);
//  attachInterrupt(digitalPinToInterrupt(MRIGHT_CHA), readEncoderRight, FALLING);
//
//  analogWrite(MLEFT_PWM,0);
//  analogWrite(MRIGHT_PWM,0);
//  digitalWrite(MLEFT_IN1, LOW);
//  digitalWrite(MLEFT_IN2, LOW);
//  digitalWrite(MRIGHT_IN1, LOW);
//  digitalWrite(MRIGHT_IN2, LOW);
  
}

void loop() {
  setMotor(1.0,MLEFT_PWM,MLEFT_IN1,MLEFT_IN2);
  setMotor(0.0, MRIGHT_PWM,MRIGHT_IN1, MRIGHT_IN2);
  float kp = 0, ki = 0;
  long newPos_left = encLeft.read();
  long t1 = micros();
  long newPos_right = encRight.read();
  long t3 = micros();
  long edge_count_left = newPos_left - pos_left;
  long t4 = micros();
  long edge_count_right = newPos_right - pos_right;
  float delS_left = (float)((edge_count_left/420.0f )* 2* M_PI * 0.0325f);
  float delS_right = (float)((edge_count_right/420.0f )* 2* M_PI * 0.0325f);
  float dt_left = (t1-prevT)*(1e-06);
  prevT = t1;
  float delS = (delS_left + delS_right)/(2.0);
  float del_theta = (delS_right - delS_left)/(0.216);

//  x += delS * cos(theta + del_theta/2);
//  y += delS * sin(theta + del_theta/2);
//  theta = del_theta ;
  kp = 0.07;
  ki = 5;
  delS_est_left += velest * dt_left;
  err_left = delS_left - delS_est_left;
  velintegrator += err_left * ki * dt_left;
  velest = err_left*kp + velintegrator;
    
  if (newPos_left != pos_left || newPos_right != pos_right){
    pos_left = newPos_left;
    pos_right = newPos_right;
    
  }
//  Serial.print(newPos_left);
//  Serial.print(" ");
//  Serial.print(newPos_right);
//  Serial.println(" ");
  Serial.print(delS_left,6);
  Serial.print(" ");
  Serial.print(delS_est_left,6);
  Serial.print(" ");  
  Serial.print(velest,6);
  Serial.print(" ");
  Serial.println(del_theta,6);
//  Serial.print(t4-t3);
//  Serial.println(" ");
//  Serial.print(delS,6);
//  Serial.print(" ");
//  Serial.print(x,6);
//  Serial.print(" ");
//  Serial.print(y,6);
//  Serial.print(" ");
//  Serial.print(del_theta,6);
//  Serial.print(" ");
//  Serial.print(theta,6);
//  Serial.println(" ");
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
    factor = fabs(factor);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm, (unsigned int) (255 * factor));
  }
}

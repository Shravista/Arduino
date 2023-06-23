#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define INTERRUPT_PIN 2 
#define LED_PIN 13 
bool blinkState = false;

#define OUTPUT_READABLE_QUATERNION
#define OUTPUT_READABLE_WORLDACCEL
MPU6050 mpu;

double k;
bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 

Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorInt16 gyroVel;
VectorFloat gravity;

volatile bool mpuInterrupt = false;     
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); 
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
  #endif
  Serial.begin(115200);
  mpu.initialize();
  pinMode(INTERRUPT_PIN,INPUT);
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(187);
  mpu.setYGyroOffset(42);
  mpu.setZGyroOffset(31);
  mpu.setZAccelOffset(5261);
  mpu.setXAccelOffset(-2947);
  mpu.setYAccelOffset(740);
  if (devStatus == 0){
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
  } else{
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  pinMode(LED_PIN,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)){
    #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            Serial.print("quat\t");
//            Serial.print(q.w);
//            Serial.print("\t");
//            Serial.print(q.x);
//            Serial.print("\t");
//            Serial.print(q.y);
//            Serial.print("\t");
//            Serial.println(q.z);
    #endif
    k = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    Serial.println(k/k,6);
//    #ifdef OUTPUT_READABLE_WORLDACCEL
//            // display initial world-frame acceleration, adjusted to remove gravity
//            // and rotated based on known orientation from quaternion
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetAccel(&aa, fifoBuffer);
//            mpu.dmpGetGravity(&gravity, &q);
//            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
//            Serial.print("aworld\t");
//            Serial.print(aaWorld.x);
//            Serial.print("\t");
//            Serial.print(aaWorld.y);
//            Serial.print("\t");
//            Serial.println(aaWorld.z);
//    #endif
      blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
//    mpu.dmpGetGyro(&gyroVel, fifoBuffer);
//    Serial.print("gyroAngl\t");
//    Serial.print(gyroVel.x);
//    Serial.print("\t");
//    Serial.print(gyroVel.y);
//    Serial.print("\t");
//    Serial.println(gyroVel.z);
  }
}

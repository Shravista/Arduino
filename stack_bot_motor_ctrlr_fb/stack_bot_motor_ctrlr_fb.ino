#include <ros.h>
#include "Arduino.h"
#include <ros/time.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <Encoder.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>


// define the interrupt pin for IMU
#define INT_PIN 2

// define left motor pins
#define MLEFT_CHA 3
#define MLEFT_CHB 8
#define MLEFT_PWM 12
#define MLEFT_IN1 10
#define MLEFT_IN2 11

// define right motor pins
#define MRIGHT_CHA 18
#define MRIGHT_CHB 7
#define MRIGHT_PWM 6
#define MRIGHT_IN1 4
#define MRIGHT_IN2 5

// define value pi
#define   M_PI   3.14159265358979323846

// instances of the encoder and imu
Encoder encLeft(MLEFT_CHA, MLEFT_CHB);
Encoder encRight(MRIGHT_CHB, MRIGHT_CHA);
MPU6050 mpu;



// initializing the node, tf and publisher
ros:: NodeHandle nh;

// defining the messages
nav_msgs::Odometry odom_msg;
geometry_msgs::TransformStamped odom_trans;
geometry_msgs::TransformStamped imu_trans;
geometry_msgs::Quaternion quat;
sensor_msgs::Imu imu_msg;

//Intitialization the required variables
double x = 0.0;
double y = 0.0;
double theta = 0.0;
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;
long prevT = 0;

//intialization for mpu6050
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
VectorFloat gravity;
VectorInt16 gyroVel;
float ypr[3], q_mag;

// ISR for the IMU
volatile bool mpuInterrupt = false;     
void dmpDataReady() {
    mpuInterrupt = true;
}

// initialization of position of encoder
long int pos_left = 0;
long int pos_right = 0;

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

void setMotor(const std_msgs::Float32 &pwrFactor, unsigned int pwm, unsigned int in1, unsigned int in2)
{
  float factor = max(min(pwrFactor.data, 1.0f), -1.0f);
  // people wanted it back motion to be slow
  if (factor >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm, (unsigned int) (255 * factor));
  } else {
    factor = fabs(factor);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm, (unsigned int) (255 * factor));
  }
}

void leftWheelCb(const std_msgs::Float32 &pwr) {
  setMotor(pwr, MLEFT_PWM, MLEFT_IN1, MLEFT_IN2);
}

void rightWheelCb(const std_msgs::Float32 &pwr) {
  setMotor(pwr, MRIGHT_PWM, MRIGHT_IN1, MRIGHT_IN2);
}

ros::Publisher odom_pub("odom", &odom_msg);
ros::Publisher imu_pub("imu", &imu_msg);

tf::TransformBroadcaster odom_broadcaster;
tf::TransformBroadcaster imu_broadcaster;

ros::Subscriber<std_msgs::Float32>sub_left("wheel_power_left", &leftWheelCb);
ros::Subscriber<std_msgs::Float32>sub_right("wheel_power_right", &rightWheelCb);

void setup() {
  //  pinMode(MLEFT_CHA, INPUT);
  //  pinMode(MLEFT_CHB, INPUT);
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); 
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
  #endif
  
  mpu.initialize();

//  initializing the pinmodes
  pinMode(INT_PIN,INPUT);
  pinMode(MLEFT_IN1, OUTPUT);
  pinMode(MLEFT_IN2, OUTPUT);
  pinMode(MLEFT_PWM, OUTPUT);
  pinMode(MRIGHT_IN1, OUTPUT);
  pinMode(MRIGHT_IN2, OUTPUT);
  pinMode(MRIGHT_PWM, OUTPUT);
  
  devStatus = mpu.dmpInitialize();

//  calibrated offsets for gyro & accel
  mpu.setXGyroOffset(187);
  mpu.setYGyroOffset(42);
  mpu.setZGyroOffset(31);
  mpu.setZAccelOffset(5261);
  mpu.setXAccelOffset(-2947);
  mpu.setYAccelOffset(740);
  
  analogWrite(MLEFT_PWM, 0);
  analogWrite(MRIGHT_PWM, 0);
  digitalWrite(MLEFT_IN1, LOW);
  digitalWrite(MLEFT_IN2, LOW);
  digitalWrite(MRIGHT_IN1, LOW);
  digitalWrite(MRIGHT_IN2,  LOW);

  nh.initNode();
  
  if (devStatus == 0){
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
  } else{
    nh.logerror(F("DMP Initialization failed (code "));
    nh.logerror(devStatus);
    nh.logerror(F(")"));
  }
  
  odom_broadcaster.init(nh);
  imu_broadcaster.init(nh);
  nh.subscribe(sub_left);
  nh.subscribe(sub_right);
  nh.advertise(odom_pub);
  nh.advertise(imu_pub);
}

void loop() {
  if (!dmpReady) {
    nh.logerror("imu is not ready");
    return;
  }
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)){
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal,&q);
    mpu.dmpGetGyro(&gyroVel, fifoBuffer);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  } else{
    nh.logerror("Error occured while reading the imu");
  }
  //  Encoder calculations
  long newPos_left = encLeft.read();
  unsigned long t2 = millis();
  long newPos_right = encRight.read();
  unsigned long t3 = millis();
  unsigned long del_t_left = t2 - prevT;
  unsigned long del_t_right = t3 - prevT;
  long edge_count_left = newPos_left - pos_left;
  long edge_count_right = newPos_right - pos_right;
  double delS_left = (double)((edge_count_left / 420.0f ) * 2 * M_PI * 0.0325f);
  double delS_right = (double)((edge_count_right / 420.0f ) * 2 * M_PI * 0.0325f);
  double vl, vr;
  vl = delS_left / del_t_left;
  vr = delS_right / del_t_right;
  double delS = (delS_left + delS_right) / (2.0);
  
  //  quaternion calculation
  q_mag = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  quat.x = q.x/q_mag;
  quat.y = q.y/q_mag;
  quat.z = q.z/q_mag;
  quat.w = q.w/q_mag;
  
  double del_theta = (delS_right - delS_left) / (0.216);
  if (newPos_left != pos_left || newPos_right != pos_right) {
    pos_left = newPos_left;
    pos_right = newPos_right;
  }
  // pose calculation
  x += delS * cos(theta);
  y += delS * sin(theta);

  //  velocity calculation
  vx = (vl + vr) * cos(theta) / 2;
  vy = (vl + vr) * sin(theta ) / 2;
  vth = -gyroVel.z;

//  orientation calculation
  theta = ypr[0]*180/M_PI;
  
  // publishing transoform  over odom tf
  odom_trans.header.stamp = nh.now();
  odom_trans.header.frame_id = "/odom";
  odom_trans.child_frame_id = "/base_link";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = quat;

  odom_broadcaster.sendTransform(odom_trans);
//  publishing the transform over imu tf
  imu_trans.header.stamp = nh.now();
  imu_trans.header.frame_id = "/base_link";
  imu_trans.child_frame_id = "/imu_link";

  imu_trans.transform.translation.x = 0.0;
  imu_trans.transform.translation.y = 0.0;
  imu_trans.transform.translation.z = 0.048;
  imu_trans.transform.rotation.x = 0.0;
  imu_trans.transform.rotation.y = 0.0;
  imu_trans.transform.rotation.z = 0.0;
  imu_trans.transform.rotation.w = 1.0;

  imu_broadcaster.sendTransform(imu_trans);

  //  publishing the messages into the odom topic
  odom_msg.header.stamp = nh.now();
  odom_msg.header.frame_id = "/odom";
  odom_msg.child_frame_id = "/base_link";

  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = quat;

  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.angular.z = vth;

  odom_pub.publish(&odom_msg);

//  publishing the message into the imu topic
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = "/imu";
  
  imu_msg.orientation = quat;
  imu_msg.angular_velocity.x = gyroVel.x;
  imu_msg.angular_velocity.y = gyroVel.y;
  imu_msg.angular_velocity.z = gyroVel.z;
  imu_msg.linear_acceleration.x = aaWorld.x;
  imu_msg.linear_acceleration.y = aaWorld.y;
  imu_msg.linear_acceleration.z = aaWorld.z;

  imu_pub.publish(&imu_msg);

  nh.spinOnce();
  prevT = t3;

  delay(300); 
}

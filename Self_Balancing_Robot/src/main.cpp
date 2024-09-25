#include <Arduino.h>
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <driver/ledc.h>  // Ensure ESP32 LEDC functions are included

// Function prototypes
void rotateMotor(int speed1, int speed2);

// Define MPU6050 and necessary variables
MPU6050 mpu;
#define INTERRUPT_PIN 15
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
VectorInt16 gy = {-3940,	-1193,	600 }; // Updated MPU values
volatile bool mpuInterrupt = false;
void IRAM_ATTR dmpDataReady() { mpuInterrupt = true; }

// PID variables
double setpointPitchAngle = -2.2;
double pitchGyroAngle = 0;
double pitchPIDOutput = 0;
double setpointYawRate = 0;
double yawGyroRate = 0;
double yawPIDOutput = 0;
PID pitchPID(&pitchGyroAngle, &pitchPIDOutput, &setpointPitchAngle, 30, 105, 0.8, DIRECT);
PID yawPID(&yawGyroRate, &yawPIDOutput, &setpointYawRate, 0.5, 0.5, 0, DIRECT);

// MPU6050 offset values
void setupMPUOffsets() {
    mpu.setXAccelOffset(-3940); 
    mpu.setYAccelOffset(-1193); 
    mpu.setZAccelOffset(600);   
    mpu.setXGyroOffset(100);
    mpu.setYGyroOffset(-6);
    mpu.setZGyroOffset(-24);
}

// Motor control pins
int enableMotor1 = 16;
int motor1Pin1 = 5;
int motor1Pin2 = 18;
int motor2Pin1 = 19;
int motor2Pin2 = 23;
int enableMotor2 = 17;

void setupPID() {
  pitchPID.SetOutputLimits(-255, 255);
  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetSampleTime(10);
  yawPID.SetOutputLimits(-255, 255);
  yawPID.SetMode(AUTOMATIC);
  yawPID.SetSampleTime(10);
}

void setupMotors() {
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  ledcAttachPin(enableMotor1, 0);
  ledcSetup(0, 5000, 8);  // 5kHz PWM, 8-bit resolution
  ledcAttachPin(enableMotor2, 1);
  ledcSetup(1, 5000, 8);  // 5kHz PWM, 8-bit resolution
  rotateMotor(0, 0);  // Ensure rotateMotor function is correctly declared
}

void setupMPU() {
  Wire.begin(21, 22);  // SDA, SCL
  Wire.setClock(400000); // 400kHz I2C clock
  mpu.initialize();
  setupMPUOffsets(); // Set the MPU offsets
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.println("DMP Initialization failed");
  }
}

void setup() {
  Serial.begin(115200);
  setupMotors();   
  setupMPU();
  setupPID();
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
}

void loop() {
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetGyro(&gy, fifoBuffer);
    yawGyroRate = gy.z;
    pitchGyroAngle = ypr[1] * 180/M_PI;
    pitchPID.Compute();
    yawPID.Compute();
    rotateMotor(pitchPIDOutput + yawPIDOutput, pitchPIDOutput - yawPIDOutput);
  }
}

void rotateMotor(int speed1, int speed2) {
  if (speed1 < 0) {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
  } else {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
  }
  if (speed2 < 0) {
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  } else {
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  }
  speed1 = abs(speed1);
  speed2 = abs(speed2);
  speed1 = constrain(speed1, 0, 255);
  speed2 = constrain(speed2, 0, 255);
  ledcWrite(0, speed1);
  ledcWrite(1, speed2);
}
// .-3940.00000,	-1193.00000,	600.00000,	100.00000,	-6.00000,	-24.00000

//-3890.00000,	-1197.00000,	596.00000,	99.00000,	-5.00000,	-25.00000
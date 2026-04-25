/*
 * 4-Wheel Drive Robot - FORWARD ONLY
 * Arduino MEGA
 * 2x Dual Motor Drivers + MPU6050
 */

#include <Wire.h>

#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT   0x3B
#define GYRO_XOUT    0x43

// Motor pins (same for Uno/Mega)
#define FL_IN1  2
#define FL_IN2  3
#define FL_EN   9
#define FR_IN1  4
#define FR_IN2  5
#define FR_EN   10
#define RL_IN1  6
#define RL_IN2  7
#define RL_EN   11
#define RR_IN1  8
#define RR_IN2  12
#define RR_EN   13

// MPU6050 on Mega: SDA=D20, SCL=D21 (handled by Wire library automatically)

int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;
float gyroZ_offset = 0;
float yaw = 0;
unsigned long lastTime = 0;
float dt;
int motorSpeed = 200;

void setup() {
  Serial.begin(115200);
  
  // Wire.begin() auto-uses D20(SDA)/D21(SCL) on Mega
  Wire.begin();
  
  // Init MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(100);
  
  // Calibrate gyro Z
  Serial.println("Calibrating... keep still");
  long sumZ = 0;
  for (int i = 0; i < 500; i++) {
    readMPU6050();
    sumZ += gyroZ;
    delay(3);
  }
  gyroZ_offset = sumZ / 500.0;
  Serial.println("Ready!");
  
  // Motor pins
  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT); pinMode(FL_EN, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT); pinMode(FR_EN, OUTPUT);
  pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT); pinMode(RL_EN, OUTPUT);
  pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT); pinMode(RR_EN, OUTPUT);
  
  stopAll();
}

void loop() {
  readMPU6050();
  updateYaw();
  forwardStraight(motorSpeed);
  
  static unsigned long t = 0;
  if (millis() - t > 500) {
    Serial.print("Yaw: "); Serial.println(yaw, 1);
    t = millis();
  }
}

void readMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  accelX = (Wire.read() << 8) | Wire.read();
  accelY = (Wire.read() << 8) | Wire.read();
  accelZ = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  gyroX = (Wire.read() << 8) | Wire.read();
  gyroY = (Wire.read() << 8) | Wire.read();
  gyroZ = (Wire.read() << 8) | Wire.read();
}

void updateYaw() {
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  lastTime = now;
  yaw += ((gyroZ - gyroZ_offset) / 131.0) * dt;
}

void motor(int in1, int in2, int en, int spd) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(en, constrain(spd, 0, 255));
}

void forwardStraight(int baseSpeed) {
  float Kp = 2.0;
  int correction = (int)(Kp * yaw);
  correction = constrain(correction, -80, 80);
  
  motor(FL_IN1, FL_IN2, FL_EN, baseSpeed - correction);
  motor(FR_IN1, FR_IN2, FR_EN, baseSpeed + correction);
  motor(RL_IN1, RL_IN2, RL_EN, baseSpeed - correction);
  motor(RR_IN1, RR_IN2, RR_EN, baseSpeed + correction);
}

void stopAll() {
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW); analogWrite(FL_EN, 0);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, LOW); analogWrite(FR_EN, 0);
  digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, LOW); analogWrite(RL_EN, 0);
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, LOW); analogWrite(RR_EN, 0);
}
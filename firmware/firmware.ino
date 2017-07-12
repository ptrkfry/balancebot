#include<Wire.h>
#include "stepper_class.h"

// Definitions for Stepper2 class
Stepper2 steppers(4, 5, 6, 8, 9, 10); 
int enablePin_1 = 6, enablePin_2 = 10;
// end definitions for Stepper2 class

// Definitions for IMU
const int MPU=0x68;  // I2C address of the MPU-6050
bool debugIMU = false;
long lastTime=0;
float delta_t=0;

float GyX,GyY,GyZ; //rates measured using Gyro
float AcX,AcY,AcZ; // accelerations measured using Accelerometer
float AngleGyroX=0, AngleGyroY=0, AngleGyroZ=0; // angle from integration of gyro data
float AngleAccelX, AngleAccelY; //angles measured using accelerometer
float AngleComplX=0, AngleComplY=0; //filtered angles

//Biases
float accXBias=0.3;
float accYBias=-0.15;
float accZBias=-0.4;
float gyroXBias=-0.83;
float gyroYBias=1.8;
float gyroZBias=-6.75;
// end definitions for IMU

// Definitions for controller
long errorSteps = 0;
double heightCenterOfGravity = 43.5;
int steps = 16; // full=1, half = 2, quarter = 4
double distancePerStep = 1.27549 / steps;
double kp = 2;
// end definitions for controller

void setup(){
  // Steppers
  steppers.setStepsToGo_1(0);
  steppers.setStepsToGo_2(0);

  // enable motors
  pinMode(enablePin_1,OUTPUT);
  digitalWrite(enablePin_1,HIGH);
  pinMode(enablePin_2,OUTPUT);
  digitalWrite(enablePin_2,HIGH);
  
  // IMU
  setupMPU6050();
  //setSensitivityOfMPU6050();

  
  Serial.begin(115200);
  lastTime = micros();
}

void loop()
{
  // Get IMU data and compute error
  if(micros() - lastTime >= 40000) // 25 Hz
  {
    getAccelAndGyro();
    errorSteps = computeErrorSteps(AngleComplX);
    if(abs(errorSteps) <= 1*steps)
    {
      errorSteps = 0;
    }
    steppers.setStepsToGo_1(errorSteps);
    steppers.setStepsToGo_2(errorSteps);
  }

  // Step both motors
//  for(int i = 0; i < steps; i++)
//  {
//    Serial.print("Motor 1 errorsteps: ");
//    Serial.println(steppers.getStepsToGo_1());
//    Serial.print("Motor 2 errorsteps: ");
//    Serial.println(steppers.getStepsToGo_2());
    steppers.stepMotors();
//  }
}

// Get values from registers of MPU6050 and convert accelerations to m/s^2 and then compute angle
void getAccelAndGyro()
{
  int16_t AcY_raw, AcZ_raw, GyX_raw;

  Wire.beginTransmission(MPU);
  Wire.write(0x3D);  // set pointer to register 0x3D (ACCEL_YOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 2, true);  // request a total of 2 registers (ACCEL_YOUT_H and ACCEL_YOUT_L)
  AcY_raw = Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)     

  Wire.beginTransmission(MPU);
  Wire.write(0x3F);  // set pointer to register 0x3F (ACCEL_ZOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 2, true);  // request a total of 2 registers (ACCEL_ZOUT_H and ACCEL_ZOUT_L)
  AcZ_raw = Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  Wire.beginTransmission(MPU);
  Wire.write(0x43);  // set pointer to register 0x43 (GYRO_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 2, true);  // request a total of 2 registers (GYRO_XOUT_H and GYRO_XOUT_L)
  GyX_raw = Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  
  delta_t = (micros()-lastTime)/1000000.0;
  lastTime = micros();

  //convert to real Values
  AcY = AcY_raw * 0.000598755;
  AcZ = AcZ_raw * 0.000598755;

  GyX = GyX_raw * 0.00762939;

  // Substract bias
  AcY = AcY - accYBias;
  AcZ = AcZ - accZBias;
  
  GyX = GyX - gyroXBias;

  computeAngles();
}

void computeAngles()
{
  //convert accelerometer measurement to angles
  AngleAccelX = atan2(AcY,AcZ) * 180/PI;

  // complementary filter (high pass for Gyro angles, low pass for Accelerometer angles)
  AngleComplX = 0.98*(AngleComplX + GyX * delta_t) + 0.02 *(AngleAccelX);  
}

long computeErrorSteps(double angle)
{
  return round(kp * heightCenterOfGravity * sin(angle/180.0*M_PI) / distancePerStep);
}

void setupMPU6050()
{
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void setSensitivityOfMPU6050()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);  //into which register to write (0x1B gyro sensitivity, 0x1C accelerometer sensitivity)
  Wire.write(0x00); //write byte to register
  Wire.endTransmission(false);
}


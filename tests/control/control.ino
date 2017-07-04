#include<Wire.h>
#include "stepper_class.h"

// Definitions for Steppers
Stepper2 steppers(4, 5, 6, 8, 9, 10); // int directionPin_1, int stepPin_1, int enablePin_1, int directionPin_2, int stepPin_2, int enablePin_2)
// end definitions for steppers

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

int counter = 0;
// end definitions for IMU

// Definitions for controller
double errorSteps = 0;
double heightCenterOfGravity = 43.5;
double distancePerStep = 0.079718;
// end definitions for controller

void setup(){
  // Steppers
  steppers.setDirection_1(0);
  steppers.setDirection_2(1);
  
  // IMU
  setupMPU6050();
  //setSensitivityOfMPU6050();

  
  Serial.begin(115200);
  lastTime = micros();
}

void loop()
{
//  // 1
//  steppers.stepMotors();
//  if (steppers.getStepsToGo_1() == 0 && steppers.getStepsToGo_2() == 0)
//  {
//      steppers.setStepsToGo_1(10000);
//      steppers.setStepsToGo_2(10000);
//  }
//  // end 1

  // 2
  //getAccelAndGyro(); //get values from registers of MPU6050 and convert Accelerations to m/s^2 and then to angles
  //errorSteps = computeErrorSteps(AngleComplX);
  steppers.stepMotors();
  errorSteps=10000;
  steppers.setStepsToGo_1(abs(errorSteps));
  steppers.setStepsToGo_2(abs(errorSteps));
  delayMicroseconds(2450);

//  if(errorSteps >= 0)
//  {
//    steppers.setDirection_1(0);
//    steppers.setDirection_2(1);
//  }
//  else
//  {
//    steppers.setDirection_1(1);
//    steppers.setDirection_2(0);
//  }


  // end 2
  
//  // DEBUG
//  if (counter%30==0 && debugIMU)
//  {
//    // Angle around X from filtered measurements
//    Serial.print("\nAngleComplX: ");
//    Serial.print(AngleComplX);
//    Serial.print(", errorSteps: ");
//    Serial.println(errorSteps);
//  }    
//  counter ++;
//  // end DEBUG
}

void getAccelAndGyro()
{
  int16_t AcX_raw,AcY_raw,AcZ_raw,Tmp,GyX_raw,GyY_raw,GyZ_raw;
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  //read registers
  AcX_raw=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY_raw=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ_raw=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX_raw=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY_raw=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ_raw=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  delta_t=(micros()-lastTime)/1000000.0;
  lastTime=micros();

  //convert to real Values
  AcX=AcX_raw*0.000598755;
  AcY=AcY_raw*0.000598755;
  AcZ=AcZ_raw*0.000598755;

  GyX=GyX_raw*0.00762939;
  GyY=GyY_raw*0.00762939;
  GyZ=GyZ_raw*0.00762939;

  // Substract bias
  AcX=AcX-accXBias;
  AcY=AcY-accYBias;
  AcZ=AcZ-accZBias;

  GyX=GyX-gyroXBias;
  GyY=GyY-gyroYBias;
  GyZ=GyZ-gyroZBias;

  computeAngles();
}

void computeAngles()
{
  //convert accelerometer measurement to angles
  AngleAccelX=atan2(AcY,AcZ) * 180/PI;
  AngleAccelY=atan2(AcX,AcZ) * 180/PI;
  //AccelAngleZ=atan(-AcX/AcZ) * 180/PI;

  // integrate Gyro measurement
  AngleGyroX = AngleGyroX + GyX * delta_t;
  AngleGyroY = AngleGyroY + GyY * delta_t;
  AngleGyroZ = AngleGyroZ + GyZ * delta_t;

  // complementary filter (high pass for Gyro angles, low pass for Accelerometer angles)
  AngleComplX= 0.98*(AngleComplX + GyX * delta_t) + 0.02 *(AngleAccelX); 
  AngleComplY= 0.98*(AngleComplY + GyY * delta_t) + 0.02 *(AngleAccelY); 
}

int computeErrorSteps(double angle)
{
  return round(heightCenterOfGravity * sin(angle/180.0*M_PI) / distancePerStep);
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


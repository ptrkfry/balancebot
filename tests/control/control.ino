#include<Wire.h>
#include <AccelStepper.h>

// Definitions for AccelStepper
AccelStepper left(1, 5, 4); // pin 5 = step, pin 4 = direction, enable pins have to be set manually
AccelStepper right(1, 9, 8); // pin 9 = step, pin 8 = direction
int enablePin_1 = 6, enablePin_2 = 10;
float speed_steppers=500.0;
// end definitions for AccelStepper

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
  left.setMaxSpeed(speed_steppers);
  right.setMaxSpeed(speed_steppers);
  left.setSpeed(speed_steppers);
  right.setSpeed(speed_steppers);
  left.setAcceleration(13000.0);
  right.setAcceleration(13000.0);

  left.move(3000);
  right.move(-3000);

  // enable motors
  pinMode(enablePin_1,OUTPUT);
  digitalWrite(enablePin_1,LOW);
  pinMode(enablePin_2,OUTPUT);
  digitalWrite(enablePin_2,LOW);
  
  // IMU
  setupMPU6050();
  //setSensitivityOfMPU6050();

  
  Serial.begin(115200);
  lastTime = micros();
}

void loop()
{
  int time_start_loop = micros();

  int time_start = micros();
  if(micros()-lastTime > 20000)
  {

    
    getAccelAndGyro(); //get values from registers of MPU6050 and convert Accelerations to m/s^2 and then to angles
    

    errorSteps = computeErrorSteps(AngleComplX);

//    Serial.print(", angleXcompl: ");
//    Serial.println(AngleComplX);
  }
  int time_end = micros()-time_start;
  Serial.print("time_getAccelAndGyro: ");
  Serial.print(time_end);
  time_start = micros();

  if(left.distanceToGo() == 0)
  {
    digitalWrite(6,HIGH);
  }
  if(right.distanceToGo() == 0)
  {
    digitalWrite(10,HIGH);
  }

  time_end = micros()-time_start;
  Serial.print(", time_check_dist: ");
  Serial.print(time_end);
  time_start = micros();

  left.run();
  right.run();

  time_end = micros()-time_start;
  Serial.print(", time_run: ");
  Serial.print(time_end);
  
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

  int time_end_loop = micros()-time_start_loop;
  Serial.print(", time_loop: ");
  Serial.println(time_end_loop);
}


void getAccelAndGyro()
{
  int16_t AcX_raw, AcZ_raw, GyX_raw;

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // set pointer to register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 2, true);  // request a total of 2 registers (ACCEL_XOUT_H and ACCEL_XOUT_L)
  AcX_raw = Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     

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
  AcX = AcX_raw * 0.000598755;
  AcZ = AcZ_raw * 0.000598755;

  GyX = GyX_raw * 0.00762939;

  // Substract bias
  AcX = AcX - accXBias;
  AcZ = AcZ - accZBias;
  
  GyX = GyX - gyroXBias;

  computeAngles();
}

void computeAngles()
{
  //convert accelerometer measurement to angles
  AngleAccelX=atan2(AcY,AcZ) * 180/PI;

  // complementary filter (high pass for Gyro angles, low pass for Accelerometer angles)
  AngleComplX= 0.98*(AngleComplX + GyX * delta_t) + 0.02 *(AngleAccelX);  
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


#include<Wire.h>
#include <AccelStepper.h>
#include <PID_v1.h>

// Definitions for AccelStepper
AccelStepper left(1, 5, 4); // pin 5 = step, pin 4 = direction, enable pins have to be set manually
AccelStepper right(1, 9, 8); // pin 9 = step, pin 8 = direction
int enablePin_1 = 6, enablePin_2 = 10;
float speed_steppers = 4000.0;
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
double AngleComplX=0, AngleComplY=0; //filtered angles

//Biases
float accXBias=0.3;
float accYBias=-0.15;
float accZBias=-0.4;
float gyroXBias=-0.83;
float gyroYBias=1.8;
float gyroZBias=-6.75;
// end definitions for IMU

// Definitions for RC
double setpointAngle = 0;
double maxSetpointAngle = 5;
const int numberOfChannels=6;
volatile long currentTime;
volatile long delta;
volatile long lastPulse=0;
volatile int pulseLengths[numberOfChannels+1]; //framespace uses one space too
volatile int channel=0;
volatile bool start=false;
// end definitions RC

// Definitions for controller
long errorSteps = 0;
double heightCenterOfGravity = 43.5;
int steps = 4; // full=1, half = 2, quarter = 4
double distancePerStep = 1.27549 / steps;

// PID
double kp = 0.595206 * steps; // 0.595206 is computed from geometry (0.595206 full steps error for 1° error)
double ki = 0;
double kd = 0;
double controlInputPID;
PID PIDcontroller(&AngleComplX, &controlInputPID, &setpointAngle, kp, ki, kd, DIRECT);
// end definitions for controller


void setup(){
  // Steppers
  left.setMaxSpeed(speed_steppers);
  right.setMaxSpeed(speed_steppers);
  left.setSpeed(speed_steppers);
  right.setSpeed(speed_steppers);
  left.setAcceleration(13000.0); // just set really high
  right.setAcceleration(13000.0);

  // enable motors
  pinMode(enablePin_1,OUTPUT);
  digitalWrite(enablePin_1,LOW);
  pinMode(enablePin_2,OUTPUT);
  digitalWrite(enablePin_2,LOW);
  
  // IMU
  setupMPU6050();
  //setSensitivityOfMPU6050();

  Serial.begin(115200);

  // RC
  attachInterrupt(digitalPinToInterrupt(2),pulseDetected,RISING);

  // PID
  PIDcontroller.SetMode(AUTOMATIC);
  PIDcontroller.SetOutputLimits(-200,200);
  
  lastTime = micros();
}

void loop()
{
  // Get IMU data and compute error
  if(micros() - lastTime >= 20000) // 50 Hz
  {
    getAccelAndGyro();

    computeSetpointAngle();
    
    //errorSteps = computeErrorSteps(AngleComplX);
    
    // Compute the control input (PID takes AngleComplX and setpointAngle and computes controlInputPID)
    PIDcontroller.Compute();
//    Serial.print("AngleError:");
//    Serial.print(setpointAngle - AngleComplX);
//    Serial.print(", controlInputPID: ");
//    Serial.print(controlInputPID);
    errorSteps = round(controlInputPID);
  
    
    if(abs(errorSteps) <= 1 * steps)
    {
      errorSteps = 0;
    }
//    Serial.print(", errorSteps: ");
//    Serial.println(errorSteps);
    left.move(errorSteps);
    right.move(-errorSteps);
  }

  // Enable or disable motors
  if(left.distanceToGo() == 0)
  {
    digitalWrite(6, HIGH);
  }
  else
  {
    digitalWrite(6, LOW);
  }
  if(right.distanceToGo() == 0)
  {
    digitalWrite(10, HIGH);
  }
  else
  {
    digitalWrite(10, LOW);
  }

  // Step both motors
  left.run();
  right.run();
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

long computeErrorSteps(double& angle)
{
  return round(kp * heightCenterOfGravity * sin(angle/180.0 * M_PI) / distancePerStep);
}

void computeSetpointAngle()
{
  setpointAngle = maxSetpointAngle/500.0 * (pulseLengths[2]-1500.0);
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

//ISR
void  pulseDetected()
{
  currentTime = micros();
  delta = currentTime-lastPulse;
  lastPulse=currentTime;
  if(delta>10000)
  {
    channel=0;
    start=true;
  }
  else
  {
    channel=channel+1;
  }
  
  if(start)
  {
    pulseLengths[channel]=delta;
  }
}


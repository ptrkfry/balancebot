#include<Wire.h>

const int MPU=0x68;  // I2C address of the MPU-6050

float GyX,GyY,GyZ; //rates measured using Gyro
float AcX,AcY,AcZ; // accelerations measured using Accelerometer
float AngleGyroX=0, AngleGyroY=0, AngleGyroZ=0; // angle from integration of gyro data
float AngleAccelX,AngleAccelY; //angles measured using accelerometer
float AngleComplX=0,AngleComplY=0; //filtered angles

//Biases
float accXBias=0.3;
float accYBias=-0.15;
float accZBias=-0.4;
float gyroXBias=-0.83;
float gyroYBias=1.8;
float gyroZBias=-6.75;

long lastTime=0;
float delta_t=0;

int counter=0;
bool writeData=false;

void setup(){
  setupMPU6050();
  //setSensitivityOfMPU6050();
  Serial.begin(115200);
  lastTime=micros();
}

void loop()
{
  getAccelAndGyro(); //get values from registers of MPU6050 and convert Accelerations to m/s^2 and then to angles

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

  if(counter%30==0)
  {
    if(writeData)
    {
      Serial.print(AngleAccelX);
      Serial.print("\t");
      Serial.print(AngleGyroX);
      Serial.print("\t");   
      Serial.println(AngleComplX);   
    }
    else 
    {
      // Serial output
      Serial.print("\n\n");
      Serial.print("Delta t: ");
      Serial.print(delta_t,6);
      
      // Angle around X from Accelerometer measurement
      Serial.print("\nAngleAccelX: ");
      Serial.print(AngleAccelX);
      // Angle around Y from Accelerometer measurement
      Serial.print("\nAngleAccelY: ");
      Serial.print(AngleAccelY);
  
      // Angle around X from integration of gyro
      Serial.print("\nGyroAngleX raw: ");
      Serial.print(AngleGyroX);
      // Angle around Y from integration of gyro
      Serial.print("\nGyroAngleY raw: ");
      Serial.print(AngleGyroY);
      // Angle around Z from integration of gyro
      Serial.print("\nGyroAngleZ raw: ");
      Serial.print(AngleGyroZ);
  
      // Angle around X from filtered measurements
      Serial.print("\nAngleComplX: ");
      Serial.print(AngleComplX);
      // Angle around Y from filtered measurements
      Serial.print("\nAngleComplY: ");
      Serial.print(AngleComplY);
    }
  }
  counter ++;
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


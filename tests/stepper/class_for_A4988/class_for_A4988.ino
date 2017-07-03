#include "stepper_class.h"

Stepper2 stepper_1(1,4,5,6); 
Stepper2 stepper_2(2,8,9,10); 

void setup()
{
  stepper_1.setDirection(0);
  stepper_2.setDirection(0);

  stepper_1.setStepsToGo(15000);
  stepper_2.setStepsToGo(15000);

  Serial.begin(9600);
}

void loop()
{
  stepper_1.stepMotor();
  stepper_2.stepMotor();
  
//  if(stepper_1.getStepsToGo() > 0)
//  {
//    stepper_1.stepMotor();
//  }
//  else
//  {
//   stepper_2.stepMotor();
//  }
}



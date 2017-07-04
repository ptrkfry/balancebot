#include "stepper_class.h"

Stepper2 steppers(4, 5, 6, 8, 9, 10); 

void setup()
{
  steppers.setDirection_1(0);
  steppers.setDirection_2(1);

  steppers.setStepsToGo_1(10000);
  steppers.setStepsToGo_2(10000);

  Serial.begin(115200);
}

void loop()
{
  steppers.stepMotors();
  if (steppers.getStepsToGo_1() == 0 && steppers.getStepsToGo_2() == 0)
  {
      steppers.setStepsToGo_1(10000);
      steppers.setStepsToGo_2(10000);
  }
//  if (steppers.getStepsToGo_1() == 0)
//  {
//    steppers.setStepsToGo_1(6400);
//    delay(500);
//  }
//  if (steppers.getStepsToGo_2() == 0)
//  {
//    steppers.setStepsToGo_2(3200);
//    delay(500);
//  }
}



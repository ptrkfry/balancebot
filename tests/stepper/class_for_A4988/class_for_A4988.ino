#include "stepper_class.h"

Stepper2 steppers(4, 5, 6, 8, 9, 10); 

void setup()
{
  steppers.setDirection_1(0);
  steppers.setDirection_2(1);

  steppers.setStepsToGo_1(15000);
  steppers.setStepsToGo_2(15000);

  Serial.begin(9600);
}

void loop()
{
  steppers.stepMotors();
  if (steppers.getStepsToGo_1() == 0)
  {
    steppers.setStepsToGo_1(5000);
    delay(500);
  }
}



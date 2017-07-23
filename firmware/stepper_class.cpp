#include "stepper_class.h"

Stepper2::Stepper2(int directionPin_1, int stepPin_1, int enablePin_1,
    int directionPin_2, int stepPin_2, int enablePin_2)
{
  setPins_1(directionPin_1, stepPin_1, enablePin_1);
  setPins_2(directionPin_2, stepPin_2, enablePin_2);
}

void Stepper2::setStepsToGo_1(long stepsToGo)
{
    stepsToGo_1 = stepsToGo;
}

void Stepper2::setStepsToGo_2(long stepsToGo)
{
    stepsToGo_2 = stepsToGo;
}

void Stepper2::setDirection_1(int dirct)
{
    direction_1 = dirct;
}

void Stepper2::setDirection_2(int dirct)
{
    direction_2 = dirct;
}

void Stepper2::stepMotors()
{
//    // don't run any motor
//    if ((stepsToGo_1 <= 0) && (stepsToGo_2 <= 0))
//    {
//      return;
//    }
//    // only run motor 1
//    if((abs(stepsToGo_1) > 0) && (abs(stepsToGo_2) <= 0))
//    {
//        // enable motor
//      digitalWrite(enablePin_1, LOW);
//  
//      // set direction
//      digitalWrite(directionPin_1, direction_1);
//  
//      // do one step
//      digitalWrite(stepPin_1, HIGH); // Output high
//      delayMicroseconds(20); // Wait
//      digitalWrite(stepPin_1, LOW); // Output low
//      delayMicroseconds(20); // Wait
//  
//      // disable motor
//      digitalWrite(enablePin_1, HIGH);
//  
//      stepsToGo_1-=1;
//    }
//    // only run motor 2
//    if((abs(stepsToGo_1) <= 0) && (abs(stepsToGo_2) > 0))
//    {
//        // enable motor
//      digitalWrite(enablePin_2, LOW);
//  
//      // set direction
//      digitalWrite(directionPin_2, direction_2);
//  
//      // do one step
//      digitalWrite(stepPin_2, HIGH); // Output high
//      delayMicroseconds(20); // Wait
//      digitalWrite(stepPin_2, LOW); // Output low
//      delayMicroseconds(20); // Wait
//  
//      // disable motor
//      digitalWrite(enablePin_2, HIGH);
//  
//      stepsToGo_2-=1;
//    }
    // run both motors
    if((abs(stepsToGo_1) > 0) && (abs(stepsToGo_2) > 0))
    {
      // enable motor
      digitalWrite(enablePin_1, LOW);
      digitalWrite(enablePin_2, LOW);
  
      // set direction
      if(getStepsToGo_1()<0)
      {
        digitalWrite(directionPin_1, HIGH);
        stepsToGo_1+=1;
      }
      else
      {
        digitalWrite(directionPin_1, LOW);
        stepsToGo_1-=1;
      }
      
      if(getStepsToGo_2()<0)
      {
        digitalWrite(directionPin_2, LOW);
        stepsToGo_2+=1;
      }
      else
      {
        digitalWrite(directionPin_2, HIGH);
        stepsToGo_2-=1;
      }
  
      // do one step
      digitalWrite(stepPin_1, HIGH); // Output high
      digitalWrite(stepPin_2, HIGH); // Output high
      delayMicroseconds(50); // Wait
      digitalWrite(stepPin_1, LOW); // Output low
      digitalWrite(stepPin_2, LOW); // Output low
      delayMicroseconds(50); // Wait
  
      // disable motor
      digitalWrite(enablePin_1, HIGH);
      digitalWrite(enablePin_2, HIGH);
    }
}

void Stepper2::disableMotors()
{
  digitalWrite(enablePin_1, HIGH);
  digitalWrite(enablePin_2, HIGH);
}

void Stepper2::setPins_1(int directionPin, int stepPin, int enablePin)
{
    stepPin_1 = stepPin;
    pinMode(stepPin_1, OUTPUT);
    directionPin_1 = directionPin;
    pinMode(directionPin_1, OUTPUT);
    enablePin_1 = enablePin;
    pinMode(enablePin_1, OUTPUT);
    //disable per default
    digitalWrite(enablePin_1, HIGH);
}

void Stepper2::setPins_2(int directionPin, int stepPin, int enablePin)
{
    stepPin_2 = stepPin;
    pinMode(stepPin_2, OUTPUT);
    directionPin_2 = directionPin;
    pinMode(directionPin_2, OUTPUT);
    enablePin_2 = enablePin;
    pinMode(enablePin_2, OUTPUT);
    //disable per default
    digitalWrite(enablePin_2, HIGH);
}

long Stepper2::getStepsToGo_1()
{
  return stepsToGo_1;
}

long Stepper2::getStepsToGo_2()
{
  return stepsToGo_2;
}



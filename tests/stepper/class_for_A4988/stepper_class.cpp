#include "stepper_class.h"

Stepper2::Stepper2(int index, int directionPin, int stepPin, int enablePin)
{
  index_ = index;
  setPins(directionPin, stepPin, enablePin);
}

void Stepper2::setStepsToGo(long stepsToGo)
{
    stepsToGo_ = stepsToGo;
}

void Stepper2::setDirection(int dirct)
{
    direction_ = dirct;
}

void Stepper2::stepMotor()
{
//    Serial.print("Motor ");
//    Serial.print(index_);
//    Serial.print(", stepsToGo: ");
//    Serial.println(stepsToGo_);
    
    if (stepsToGo_ == 0)
    {
        Serial.println("no steps to go, return");
        digitalWrite(enablePin_, HIGH);
        return;
    }
    stepsToGo_-=1;
    
    // enable motor
    digitalWrite(enablePin_, LOW);

    // set direction
    digitalWrite(directionPin_, direction_);

    // do one step
    digitalWrite(stepPin_, HIGH); // Output high
    delayMicroseconds(20); // Wait
    digitalWrite(stepPin_, LOW); // Output low
    delayMicroseconds(20); // Wait

    // disable motor
    digitalWrite(enablePin_, HIGH);
}

void Stepper2::setPins(int directionPin, int stepPin, int enablePin)
{
    stepPin_ = stepPin;
    pinMode(stepPin_, OUTPUT);
    directionPin_ = directionPin;
    pinMode(directionPin_, OUTPUT);
    enablePin_ = enablePin;
    pinMode(enablePin_, OUTPUT);
    //disable per default
    digitalWrite(enablePin_, HIGH);
}

long Stepper2::getStepsToGo()
{
  return stepsToGo_;
}



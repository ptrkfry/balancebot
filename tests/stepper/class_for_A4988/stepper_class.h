#ifndef Stepper2_h
#define Stepper2_h

#include <Arduino.h>

class Stepper2
{
public:
    Stepper2(int index, int directionPin, int stepPin, int enablePin);
    void setStepsToGo(long stepsToGo);
    long getStepsToGo();
    void setDirection(int dirct);
    void stepMotor();
private:
    void setPins(int directionPin, int stepPin, int enablePin);
    int stepPin_, directionPin_, enablePin_;
    int index_;
    int direction_;
    long stepsToGo_;
};
#endif

#ifndef Stepper2_h
#define Stepper2_h

#include <Arduino.h>

class Stepper2
{
public:
    Stepper2(int directionPin_1, int stepPin_1, int enablePin_1,
    int directionPin_2, int stepPin_2, int enablePin_2);
    void setStepsToGo_1(long stepsToGo);
    void setStepsToGo_2(long stepsToGo);
    long getStepsToGo_1();
    long getStepsToGo_2();
    void setDirection_1(int dirct);
    void setDirection_2(int dirct);
    void stepMotors();
private:
    void setPins_1(int directionPin, int stepPin, int enablePin);
    void setPins_2(int directionPin, int stepPin, int enablePin);
    int stepPin_1, directionPin_1, enablePin_1;
    int stepPin_2, directionPin_2, enablePin_2;
    int direction_1 = 1, direction_2 = 0;
    long stepsToGo_1 = 0, stepsToGo_2 = 0;
};
#endif

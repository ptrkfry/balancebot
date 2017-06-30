#include <AccelStepper.h>
#include <MultiStepper.h>

AccelStepper stepper1(1,13,12); //use pin 12 and 13 for dir and step, 1 is the "external driver" mode
AccelStepper stepper2(1,14,15);
MultiStepper multi;

void setup() 
{
  stepper1.setMaxSpeed(3000); //for multistepper there is no acceleration--> max speed = speed
  stepper2.setMaxSpeed(3000);

  multi.addStepper(stepper1);
  multi.addStepper(stepper2);
  long positions[2]; //array to store current target stepper positions (positions[0] is for stepper1, positions[1] is for stepper2
  positions[0]=200;
  positions[1]=600;
  multi.moveTo(positions); //Set the target positions of all managed steppers
}

void loop() 
{
  stepper1.run(); //run the stepper. Both steppers are run so that the target position is achieved by both motors at the same time --> linear movement
  //OR
  //steppers.runSpeedToPosition(); // Blocks until all are in position
}

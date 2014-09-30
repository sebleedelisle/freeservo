#include <AccelStepper.h>

const int stepPin = 12; 
const int dirPin = 13; 

AccelStepper accel(AccelStepper::DRIVER, stepPin, dirPin); 

void setup() { 
    accel.setMaxSpeed(700.0);
    accel.setAcceleration(10000.0);
    accel.moveTo(1000);
  
}


void loop()  {
  
  if (accel.distanceToGo() == 0)
	accel.moveTo(-accel.currentPosition());
    accel.run();
}

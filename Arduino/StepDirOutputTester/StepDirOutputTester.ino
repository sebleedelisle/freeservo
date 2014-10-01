#include <AccelStepper.h>

const int stepPin = 12; 
const int dirPin = 13; 

AccelStepper accel(AccelStepper::DRIVER, stepPin, dirPin); 

void setup() { 
    
    accel.setMaxSpeed(300.0);
    accel.setAcceleration(300.0);
    accel.moveTo(-500);
  
}


void loop()  {
  
  if (accel.distanceToGo() == 0) {
	accel.moveTo(-accel.currentPosition());
    if(accel.currentPosition()<0) delay(500);
  }
    accel.run();
}

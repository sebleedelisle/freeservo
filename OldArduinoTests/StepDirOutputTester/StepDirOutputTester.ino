#include <AccelStepper.h>

const int stepPin = 12;
const int dirPin = 13;

AccelStepper accel(AccelStepper::DRIVER, stepPin, dirPin);

boolean forwards = false;


void setup() {

  accel.setMaxSpeed(200.0);
  accel.setAcceleration(1000.0);
  accel.moveTo(1000);

}


void loop()  {

  if (accel.distanceToGo() == 0) {
    forwards = !forwards;
    if (forwards) accel.move(2050);
    else accel.move(-50);
    //if(accel.currentPosition()<0) delay(500);
  }
  accel.run();
}

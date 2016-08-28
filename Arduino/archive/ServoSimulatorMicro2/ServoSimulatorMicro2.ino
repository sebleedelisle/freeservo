#include <digitalIOPerformance.h>


const int encoderAPin = 4;
const int encoderBPin = 5;

//int32_t motorPosition = 0;


void setup() {
  // put your setup code here, to run once:
  pinMode(encoderAPin, OUTPUT);
  pinMode(encoderBPin, OUTPUT);
  digitalWrite(encoderAPin, LOW);
  digitalWrite(encoderBPin, LOW);

}

void loop() {
  // put your main code here, to run repeatedly:

  for (long motorPosition = 0; motorPosition <= 300000; motorPosition++) {

    digitalWrite(encoderAPin, ((motorPosition + 1) >> 1) & 0b1 );
    digitalWrite(encoderBPin, ((motorPosition) >> 1) & 0b1);

    //delayMicroseconds(3);

  }

  delay(500);

  for (long motorPosition = 300000; motorPosition >= 0; motorPosition--) {

    digitalWrite(encoderAPin, ((motorPosition + 1 ) >> 1) & 0b1 );
    digitalWrite(encoderBPin, ((motorPosition) >> 1) & 0b1);

    //delayMicroseconds(3);

  }
  delay(500);

}

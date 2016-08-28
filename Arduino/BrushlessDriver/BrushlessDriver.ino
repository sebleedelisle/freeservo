#include <Adafruit_NeoPixel.h>

// PHASES :
//      A     B     C     H1   H2   H3   binary
// 0    o     -     +     1    0    1      5
// 1    +     -     o     1    0    0      1
// 2    +     o     -     1    1    0      3
// 3    o     +     -     0    1    0      2
// 4    -     +     o     0    1    1      6
// 5    -     o     +     0    0    1      4

// look up table of which step for which hall sensor pattern
const int stepForSensors[6] = {1, 3, 2, 5, 0, 4};

Adafruit_NeoPixel strip = Adafruit_NeoPixel(12, 23, NEO_GRB + NEO_KHZ800);

const int motorPins[]  =   {7, 8, 9};     // these are just digital
const int enablePins[] =   {5, 6, 10};      // these are PWM
const int hallSensorPins[] = {19, 20, 21}; // any digital pin

const int commutations[6][3] = { { 0, -1, 1}, {1, -1, 0}, {1, 0, -1}, { 0, 1, -1} , { -1, 1, 0}, { -1, 0, 1}};

int motorSpeed = 100;
int binaryHallSensors = 1;
int currentStep = 0;


void setup() {

  for (int i = 0; i < 3; i++) {

    pinMode(enablePins[i], OUTPUT);
    pinMode(motorPins[i], OUTPUT);

    // if you wanna chnage the PWM frequency, do it here :
    //  analogWriteFrequency(enablePins[i], 3000);

  }

  analogWriteResolution(11);

  for (int i = 0; i < 3; i++) {
    pinMode(hallSensorPins[i], INPUT_PULLUP);
    
    attachInterrupt(hallSensorPins[i], updateHallSensors, CHANGE); 
  }

  //Serial.begin(9600);
  strip.begin();
  strip.show();
}


// the loop routine runs over and over again forever:

void loop() {

  //drive motor
  updateMotor();


  for (int i = 0; i < 6; i++) {
    strip.setPixelColor(i, (i == currentStep) ? 0x100000 : 0x000000);
  }

  //int binaryHallSensors = (analogRead(A6)>30)<<2;
  strip.setPixelColor(6, (binaryHallSensors >> 2) * 10);
  strip.setPixelColor(7, (binaryHallSensors >> 1 & 1) * 10);
  strip.setPixelColor(8, (binaryHallSensors & 1) * 10 );

  //  Serial.print(" "); Serial.print(analogRead(hallSensorPins[0])); Serial.print(" "); Serial.print(analogRead(hallSensorPins[1])); Serial.print(" "); Serial.println(analogRead(hallSensorPins[2]));

  strip.show();


}

void updateMotor() {
  motorSpeed = map(analogRead(A0), 0, 1023, -2047, 2047);
  
  for (int i = 0; i < 3; i++) {
    //set enable pin
    int enablepin = enablePins[i];
    int motorpin = motorPins[i];

    // if the commutation for this phase is :
    //     -1 then it needs to go to ground
    //      0 no connection so enable pin false (otherwise enable pin is set to the pwm power)
    //      1 then connected to voltage. 

    analogWrite(enablepin, commutations[currentStep][i] == 0 ? 0 : abs(motorSpeed));

    //set output signal (note that it's reversed if going backwards)
    digitalWrite(motorpin, commutations[currentStep][i] > 0 ? (motorSpeed > 0) : (motorSpeed < 0));

  }

}


void updateHallSensors() { 
  
  binaryHallSensors = (digitalRead(hallSensorPins[2])  << 2) | (digitalRead(hallSensorPins[1]) << 1) | (digitalRead(hallSensorPins[0]));
  
  binaryHallSensors = constrain(binaryHallSensors, 1, 6);
  currentStep = stepForSensors[((binaryHallSensors - 1)) % 6];

  
  
}



//#define USE_MOTOR_SHIELD
//#define USE_4_PWM
//#define USE_2_PWM
#define USE_ARDUMOTO

#define USE_ENCODER_LIBRARY



#include <EEPROM.h>
#include "EEPROMAnything.h"
#include "digitalWriteFast.h"

#include <PID_v1.h>


volatile double position, targetPositionDouble, motorPower, errorValue, maxValue = 10000;
volatile long targetPositionLong = 0;

#include <stdlib.h>
#include "constants.h"
#include <Encoder.h>
#include "ServoEncoder.h"


#include "MotorDrives.h"

const int warningMargin = 10; 
const int errorMargin = 512; // the number of ticks out of place before the servo goes
                             // into error.
                             
int counter = 0; 

bool servoError = false;
volatile int stepState = 0; 
volatile int dirState = 0; 

long startOffset = 0;

String command = "";
boolean commandComplete = false;

//double Kp = 0.14, Ki = 0.03, Kd = 0.0002;
//double Kp = 0.88, Ki = 0.02, Kd = 0.0007;
//double Kp = 0.77, Ki = 0.37, Kd = 0.0005;
//double Kp = 0.3, Ki = 0.05, Kd = 0.0003;
double Kp = 2, Ki = 0.1, Kd = 0.001;
//double Kp = 2, Ki = 8, Kd = 0.01;

const byte eepromValidateData = 1;
const byte eepromDataAddr = 32;

PID myPID(&position, &motorPower, &targetPositionDouble, Kp, Ki, Kd, DIRECT);

//////////////////////////////////////////////////////////////////////////////////
// Interrupt service routine for pin change interrupt #2
ISR(PCINT2_vect) 
{
  register byte copyPortD = PIND; // capture port value asap
  if(copyPortD & (1<<DBIT_STEP)) // Ensure this is a RISING edge of step pin
  {
    if(copyPortD & (1<<DBIT_DIR))  // Check the direction pin
      ++targetPositionLong; 
    else
      --targetPositionLong; 
  }
}

void setup() {

  command.reserve(256);

  byte eepromAddr = eepromDataAddr;
  if( EEPROM.read(eepromAddr++)==eepromValidateData ){
    eepromAddr+=EEPROM_readAnything(eepromAddr,Kp);
    eepromAddr+=EEPROM_readAnything(eepromAddr,Ki);
    eepromAddr+=EEPROM_readAnything(eepromAddr,Kd);    
  }


  while( !Serial );
  Serial.begin(9600);
  sendPIDOverSerial();

  //TCCR3B &= (0xff & 0x1); // change pwm frequency to 40k or something
  //TCCR4B &= (0xff & 0x1); // change pwm frequency to 40k or something

  initEncoder(); 

  position = targetPositionDouble = motorPower = 0;
  targetPositionLong = 0;

  pinMode(errorLightPin, OUTPUT); 
  pinMode(warnLightPin, OUTPUT); 
  pinMode(okLightPin, OUTPUT); 
  // pinMode(resetPin, INPUT_PULLUP); 
  pinMode(stepPin, INPUT_PULLUP); 
  pinMode(dirPin, INPUT_PULLUP);

  
  pinMode(ampErrorPin, INPUT); 

  ///////////////////////////////////////////////////////
  // configure interrupt on change for step pin
  PCMSK2 = _BV(PCINT20);   // Configure pin change interrupt 2 on change in PCINT20/PD4 only
  PCICR |= _BV(PCIE2);      // Enable pin change interrupt 2 
  ///////////////////////////////////////////////////////


  // MAKE TONE ON RESET
  pinMode(buzzerPin, OUTPUT); 
  tone(buzzerPin, 1000); 


  initMotor();

  initialisePID();   







  //---------------------------------------------- Set PWM frequency for D5 & D6 -------------------------------

  //TCCR0B = TCCR0B & B11111000 | B00000001;    // set timer 0 divisor to     1 for PWM frequency of 62500.00 Hz
  //TCCR0B = TCCR0B & B11111000 | B00000010;    // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
  //  TCCR0B = TCCR0B & B11111000 | B00000011;    // set timer 0 divisor to    64 for PWM frequency of   976.56 Hz
  //TCCR0B = TCCR0B & B11111000 | B00000100;    // set timer 0 divisor to   256 for PWM frequency of   244.14 Hz
  //TCCR0B = TCCR0B & B11111000 | B00000101;    // set timer 0 divisor to  1024 for PWM frequency of    61.04 Hz


  //---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------

  //TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz



}


void loop() {

  updateEncoder(); 

  //targetPositionLong = round(((cos((millis()-startOffset) * 0.0008f)) -1) * 100.0f);

  targetPositionDouble=targetPositionLong; // Copy the integer value updated by the ISR into the float value used by PID
  counter ++; 
  
  if(myPID.Compute()){
    if(counter>33) { // update 30 times a second 
      // send position over serial
      Serial.print("POS:"); 
       Serial.print(targetPositionDouble); 
       Serial.print(","); 
       Serial.println(position); 
       counter = 0; 
    }
  }

  errorValue = targetPositionDouble - position; 
  if(errorValue<0) errorValue*=-1; 

  
  if (!servoError) {
    setMotorPower(motorPower);
    digitalWrite(errorLightPin, LOW);  
    //Serial.println(motorPower); 
  } 
  else {
    setMotorPower(0);

    // this line should flash the red light if we have an amp error or just make it steady on 
    // if it's a normal servo error. Although - does the ampError reset or stay permanently on? 

    if(!digitalRead(ampErrorPin) && (millis()%200<100)) digitalWrite(errorLightPin, LOW);  
    else digitalWrite(errorLightPin, HIGH); 
    
  }

  if ((!servoError) && (errorValue > errorMargin)) {
    servoError = true;
    // tone(buzzerPin, 1000, 10000);


  } 

  if ( (errorValue>warningMargin)) {
    digitalWrite(warnLightPin, HIGH);
  } 
  else {
    digitalWrite(warnLightPin, LOW);
  }

  if(errorValue <= 2) {
    digitalWrite(okLightPin, HIGH);
  } 
  else{ 
    digitalWrite(okLightPin, LOW);   
  }

  // reset if the numbers get too high!
  if(targetPositionLong>maxValue) {
    targetPositionDouble-=maxValue; 
    targetPositionLong-=maxValue; 
    position-=maxValue; 
  }
  
  if(targetPositionLong<-maxValue) {
    targetPositionDouble+=maxValue; 
    targetPositionLong+=maxValue; 
    position+=maxValue; 
  }
  checkSerial();
  
  //Serial.println(targetPositionLong);
  
  //  if((servoError) && (digitalRead(resetPin) == LOW)) {
  //     reset();  
  //  }
  

}

void reset() { 

  servoError = false; 
  startOffset = millis(); 
  encoder.write(0); 
  position = targetPositionDouble = motorPower = 0; 
  targetPositionLong = 0;

}

void initialisePID() { 
  //encoder.write(0); 
  position = targetPositionDouble = motorPower = 0; 
  targetPositionLong = 0;
  myPID.SetOutputLimits(-100, 100); 
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetTunings(Kp, Ki, Kd);
  targetPositionLong = targetPositionDouble;
  sendPIDOverSerial();



}






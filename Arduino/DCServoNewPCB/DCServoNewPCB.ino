  
#define USE_ENCODER_LIBRARY
//#define ENCODER_OPTIMIZE_INTERRUPTS

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

bool sendPos = false; 

volatile bool servoError = false;
volatile bool initialised = false; 
volatile int stepState = 0; 
volatile int dirState = 0; 
byte stepMultiplier = 1; // this should probably be adjustable

long startOffset = 0;

String command = "";
boolean commandComplete = false;


//double Kp = 0.14, Ki = 0.03, Kd = 0.0002;
//double Kp = 0.88, Ki = 0.02, Kd = 0.0007;
//double Kp = 0.77, Ki = 0.37, Kd = 0.0005;
//double Kp = 0.3, Ki = 0.05, Kd = 0.0003;
//double Kp = 100, Ki = 6, Kd = 0.4;
//double Kp = 2, Ki = 8, Kd = 0.01;
double Kp = 15.5, Ki = 5.71, Kd = 0.0184;

// CHANGE THIS TO RESET THE EEPROM DATA
const byte eepromValidateData = 6;
const byte eepromDataAddr = 32;


PID myPID(&position, &motorPower, &targetPositionDouble, Kp, Ki, Kd, DIRECT);



void setup() {

  // set PWM frequency to 31250
  setPwmFrequency(9, 1);
  setPwmFrequency(10, 1);
  
  // set PWM frequency to 3906
//  setPwmFrequency(9, 8);
//  setPwmFrequency(10, 8);
 
  
  command.reserve(256);

  readEepromData(); 

  initSerial();
  sendPIDOverSerial();  

  initEncoder(); 

  position = targetPositionDouble = motorPower = 0;
  targetPositionLong = 0;

  pinMode(errorLightPin, OUTPUT); 
  pinMode(warnLightPin, OUTPUT); 
  pinMode(okLightPin, OUTPUT); 


  pinMode(stepPin, INPUT_PULLUP); 
  pinMode(dirPin, INPUT_PULLUP);



  pinMode(ampErrorPin, INPUT); 

  ///////////////////////////////////////////////////////
  // configure interrupt on change for step pin
  PCMSK2 |= (1<<4);
  PCICR |= _BV(PCIE2);  
  ///////////////////////////////////////////////////////

  initMotor();
  initPID();   
  initBuzzer(); 
  
  startUpNotify(); 


  initialised = true; 


}


void loop() {

  updateEncoder(); 

  // UNCOMMENT THIS TO TEST THE SERVO WITHOUT STEP/DIR
  //targetPositionLong = round(((cos((millis()-startOffset) * 0.0008f)) -1) * 1000.0f);

  updatePID(); 
  updateServo(); 
  checkSerial();


}


inline void updateServo() { 
  
   if (!servoError) {
    setMotorPower(motorPower);
    digitalWrite(errorLightPin, LOW);  
    digitalWrite(errorLightPin, !digitalRead(ampErrorPin)); 
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
    tone(buzzerPin, 500, 10000);
    
    reset();


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


}

inline void updatePID() { 

  // reset if the numbers get too high!
  //  if(targetPositionLong>maxValue) {
  //    
  //    targetPositionLong-=maxValue; 
  //    targetPositionDouble-=maxValue; 
  //    position-=maxValue; 
  //  }
  //  
  //  if(targetPositionLong<-maxValue) {
  //    targetPositionLong+=maxValue; 
  //    targetPositionDouble+=maxValue; 
  //    position+=maxValue; 
  //  }

  targetPositionDouble=targetPositionLong; // Copy the integer value updated by the ISR into the float value used by PID


  if(myPID.Compute()){
    if(counter>33) { // update 30 times a second 
      // send position over serial
      if(sendPos) {
        Serial.print("POS:"); 
        Serial.print(targetPositionDouble); 
        Serial.print(","); 
        Serial.println(position); 
      }
      counter = 0; 
    }
    counter ++;
  }

  errorValue = targetPositionDouble - position; 
  if(errorValue<0) errorValue*=-1; 

}


void initPID() { 
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


void reset() { 


  startOffset = millis(); 
  encoder.write(0); 
  position = targetPositionDouble = motorPower = 0; 
  targetPositionLong = 0;
  servoError = false; 
}




inline void readEepromData() { 

  byte eepromAddr = eepromDataAddr;

  if( EEPROM.read(eepromAddr++)==eepromValidateData ){
    eepromAddr+=EEPROM_readAnything(eepromAddr,Kp);
    eepromAddr+=EEPROM_readAnything(eepromAddr,Ki);
    eepromAddr+=EEPROM_readAnything(eepromAddr,Kd);    
    eepromAddr+=EEPROM_readAnything(eepromAddr,motorDeadZone);    
  }

}
inline void writeEepromData() { 
   byte eepromAddr = eepromDataAddr;
    EEPROM.write(eepromAddr++,eepromValidateData);
    eepromAddr+=EEPROM_writeAnything(eepromAddr,Kp);
    eepromAddr+=EEPROM_writeAnything(eepromAddr,Ki);
    eepromAddr+=EEPROM_writeAnything(eepromAddr,Kd);    
    eepromAddr+=EEPROM_writeAnything(eepromAddr,motorDeadZone);    
  
}

inline void initBuzzer() { 
  
  pinMode(buzzerPin, OUTPUT); 
  pinMode(buzzerGroundPin, OUTPUT); 
  digitalWrite(buzzerGroundPin, LOW); 


}

inline void startUpNotify() { 
    
  for(int i =0 ; i<10; i++) { 
    bool val = ((i%2) ==0); 
    digitalWrite(errorLightPin, val); 
    digitalWrite(warnLightPin, val); 
    digitalWrite(okLightPin, val); 
    tone(buzzerPin,500,100); 

    delay(100); 
  }
}
  
  
//////////////////////////////////////////////////////////////////////////////////
// Interrupt service routine for pin change interrupt #1

ISR(PCINT2_vect)
{
  if(servoError | !initialised) return; 
  register byte copyPortD = PIND; // capture port value asap
  if(copyPortD & (1<<DBIT_STEP) ) // Ensure this is a RISING edge of step pin
  {
    if(copyPortD & (1<<DBIT_DIR))  // Check the direction pin
      targetPositionLong+=stepMultiplier; 
    else
      targetPositionLong-=stepMultiplier; 
  }
}



//
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}




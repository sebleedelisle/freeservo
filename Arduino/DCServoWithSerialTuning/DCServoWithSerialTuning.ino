//#define USE_MOTOR_SHIELD
//#define USE_PAUL_MOTOR_DRIVE
#define USE_RC_SERVO
#define USE_ENCODER_LIBRARY
//#define USE_7SEG_DISPLAY
//#define USE_NEOPIXEL

#include <EEPROM.h>
#include "EEPROMAnything.h"
#include "digitalWriteFast.h"

#include <PID_v1.h>


volatile double position, targetPosition, motorPower;
  
#include<stdlib.h>
#include "constants.h"
#include "Encoder.h"

#ifdef USE_RC_SERVO
#include <Servo.h> 
#endif

#ifdef USE_7SEG_DISPLAY
#include "Wire.h"
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"
Adafruit_7segment matrix1 = Adafruit_7segment();
Adafruit_7segment matrix2 = Adafruit_7segment();
double lastDisplayedPos;
#endif

#ifdef USE_NEOPIXEL
#include <Adafruit_NeoPixel.h>
#define NEOPIXEL_PIN 11
int numLeds = 24;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(numLeds, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
#endif

#include "MotorDrives.h"

int errorMargin = 2000; // the number of ticks out of place before the servo goes
// into error.

bool servoError = false;
volatile int stepState = 0; 
volatile int dirState = 0; 

long startOffset = 0;

String command = "";
boolean commandComplete = false;

//double Kp = 0.14, Ki = 0.03, Kd = 0.0002;
//double Kp = 0.88, Ki = 0.02, Kd = 0.0007;
double Kp = 0.77, Ki = 0.37, Kd = 0.0005;
const byte eepromValidateData = 3;
const byte eepromDataAddr = 32;

PID myPID(&position, &motorPower, &targetPosition, Kp, Ki, Kd, DIRECT);


void setup() {

  command.reserve(256);

  byte eepromAddr = eepromDataAddr;
  if( EEPROM.read(eepromAddr++)==eepromValidateData ){
    eepromAddr+=EEPROM_readAnything(eepromAddr,Kp);
    eepromAddr+=EEPROM_readAnything(eepromAddr,Ki);
    eepromAddr+=EEPROM_readAnything(eepromAddr,Kd);    
  }

  while( !Serial );
  
  //Serial.begin(9600);
  sendPIDOverSerial();

  //TCCR3B &= (0xff & 0x1); // change pwm frequency to 40k or something
  //TCCR4B &= (0xff & 0x1); // change pwm frequency to 40k or something

  initEncoder(); 

  position = targetPosition = motorPower = 0;

  pinMode(errorLightPin, OUTPUT); 
  pinMode(okLightPin, OUTPUT); 
  pinMode(resetPin, INPUT_PULLUP); 
  pinMode(stepPin, INPUT_PULLUP); 
  pinMode(dirPin, INPUT_PULLUP);
  // PCintPort::attachInterrupt(stepPin, &stepInterruptFired, CHANGE);
  attachInterrupt(4, stepInterruptFired, CHANGE);  
 // attachInterrupt(5, dirInterruptFired, FALLING);  
 
 pinMode(buzzerPin, OUTPUT); 
 //tone(buzzerPin, 1000); 
  
  #ifdef USE_7SEG_DISPLAY
  initDisplay(); 
  #endif
  
  #ifdef USE_NEOPIXEL
  strip.begin(); 
  strip.show(); 
  #endif
  
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
TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
//TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz



}


void loop() {
  
  updateEncoder(); 
 
  myPID.Compute();

  if (!servoError) {
    setMotorPower(motorPower);
    //Serial.println(motorPower); 
  } //else setMotorPower(0);
  
//targetPosition = round(((cos((millis()-startOffset) * 0.0008f)) -1) * 1000.0f);

  if ((!servoError) && (abs(position - targetPosition) > errorMargin)) {
    servoError = true;
    tone(buzzerPin, 1000, 10000);

  }
  if (servoError) { // && ((millis() % 500) < 250)) || (abs(targetPosition - position)>256)) {
    digitalWrite(errorLightPin, HIGH);
  } else {
    digitalWrite(errorLightPin, LOW);
  }
  
  
  if(abs(targetPosition - position)<=10) {
     digitalWrite(okLightPin, HIGH);
  } else{ 
     digitalWrite(okLightPin, LOW);   
  }


  checkSerial();
  if((servoError) && (digitalRead(resetPin) == LOW)) {
     reset();  
    
  }
  
  //Serial.println(digitalRead(dirPin)); 
  
  digitalWrite(13, digitalRead(resetPin)); 
  
  #ifdef USE_7SEG_DISPLAY
  updateDisplay(); 
  #endif
  
  #ifdef USE_7SEG_DISPLAY
  updatePixelDisplay(); 
  #endif
}

void reset() { 
   
    servoError = false; 
    startOffset = millis(); 
    encoder.write(0); 
    position = targetPosition = motorPower = 0; 
  
}

void initialisePID() { 
  //encoder.write(0); 
  position = targetPosition = motorPower = 0; 
  myPID.SetOutputLimits(-100, 100); // 80% max power
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetTunings(Kp, Ki, Kd);
  

 
  
}
void checkSerial() {

  if ( commandComplete ) {
    char commandStr[command.length() + 1];
    char *ptr;

    command.toCharArray( commandStr, command.length() + 1 );

    Kp = strtod(commandStr, &ptr);    
    ptr++;
    Ki = strtod(ptr, &ptr);
    ptr++;
    Kd = strtod(ptr, &ptr);
    myPID.SetTunings(Kp, Ki, Kd);

    byte eepromAddr = eepromDataAddr;
    EEPROM.write(eepromAddr++,eepromValidateData);
    eepromAddr+=EEPROM_writeAnything(eepromAddr,Kp);
    eepromAddr+=EEPROM_writeAnything(eepromAddr,Ki);
    eepromAddr+=EEPROM_writeAnything(eepromAddr,Kd);    
    //sendPIDOverSerial();
    //targetPosition = random(0,8000);

    commandComplete = false;
    command = "";

  }
}



void sendPIDOverSerial(){
  
  char buf[12];
  dtostrf( Kp, 12, 8, buf );
  Serial.print("PID:");
  Serial.print( buf );
  Serial.print( ',' );
  dtostrf( Ki, 12, 8, buf );
  Serial.print( buf );
  Serial.print( ',' );
  dtostrf( Kd, 12, 8, buf );
  Serial.print( buf );
  Serial.println();

}

// only falling now
void stepInterruptFired() { 
  //stepState = digitalRead(stepPin); 
  //dirState = digitalReadFast(dirPin); 
  //if(!stepState) { 
  if(!digitalReadFast(stepPin)) return; 
  if(digitalReadFast(dirPin)) targetPosition++; 
    else targetPosition--; 
  //} 
  
} 

//void dirInterruptFired() { 
// dirState = digitalRead(dirPin); 
  
//}

void serialEvent() {
  if (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      commandComplete = true;
    } else {
      command += inChar;
    }
  }
}




#include <SPI.h>
#include <i2c_t3.h>
#include "config.h"

boolean buttonPressed = false;

#include "LedControl.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306T3.h>
#include "QuadDecode_def.h"
#include "TrimPot.h"

//#include <PID_v1.h>

TrimPot pPot = TrimPot(pidTrimPotPins[0]);
TrimPot iPot = TrimPot(pidTrimPotPins[1]);
TrimPot dPot = TrimPot(pidTrimPotPins[2]);

volatile double position, motorPower, errorValue;
long targetPositionLong = 0;
volatile int targetPositionChange = 0;
//volatile bool isDirectionForward = false;
//volatile int targetPositionSub = 0;

QuadDecode<1> xPosn;  // Template using FTM1 - encoder inputs 3 and 4

volatile int coilOffset = 0;

#include "Motors.h"

//double Kp = 15.5, Ki = 5.71, Kd = 0.0184;
//double Kp = 0.14, Ki = 0.03, Kd = 0.0002;
//double Kp = 0.88, Ki = 0.02, Kd = 0.0007;
//double Kp =4, Ki =.0005, Kd = 0.001;//0.0184;
//1/5000 = 0.0002



#include "PID.h"
#include "Display.h"

void setup()
{

  Serial.begin(115200);

  analogReference(EXTERNAL);

  pinMode(stepPin, INPUT);
  pinMode(dirPin, INPUT);
  pinMode(resetSwitch, INPUT_PULLUP);

  xPosn.setup();      // Start Quad Decode position count
  xPosn.start();      // Start Quad Decode position count
  attachInterrupt(stepPin, readStep, RISING);
  pPot.setLimits(10, 0);
  iPot.setLimits(0.1, 0); // was 0.01
  dPot.setLimits(100, 0);

  initMotor();

  initPID();
  initDisplay();

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  updateHallSensors();
}

void loop() {

  //targetPositionLong = millis()*4;//
  //targetPositionLong = -round(((cos((millis()) * 0.001f)) - 1) * 20000.0f);

  //    Serial.print("M : ");
  //    Serial.println(motorPower);


  //currentStep = map(pPot.currentValue, 0, 1000, 12,0)%6;
  updateDisplay();

  if (pPot.update()) showTempDisplay(DISPLAY_MODE_P);
  if (iPot.update()) showTempDisplay(DISPLAY_MODE_I);
  if (dPot.update()) showTempDisplay(DISPLAY_MODE_D);

  Kp = pPot.getValue();
  Ki = iPot.getValue();
  Kd = dPot.getValue();



  coilOffset = map(analogRead(potPin1), 0, 1023, 200, -200);

  //digitalWrite(13, errorValue > 20);
  delay(20);

}

inline void readStep() {
  //long pos = targetPositionLong;
  if (digitalReadFast(dirPin)) {
    targetPositionChange++;
  } else {
    targetPositionChange--;
  }

}

//inline void changeDirForward() {
//  isDirectionForward = true;
//}
//
//inline void changeDirBackward() {
//  isDirectionForward = false;
//}



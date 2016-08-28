#include "LedControl.h"

// LedControl is the library for running the
// 7 segment LED display. Set it up with the
// pins for DIN, CLK and LOAD/CS

LedControl lc = LedControl(15, 17, 16, 1);

const int resetSwitch = 18;

void setup() {

  Serial.begin(115200);
  init7SegDisplay();
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  delay(100);
  digitalWrite(13, HIGH);
  delay(100);

  pinMode(resetSwitch, INPUT_PULLUP);

}


void loop() {

  showNumber(analogRead(A0));

  for (int i = 0; i < 127; i++) {
  


    while (digitalRead(resetSwitch)) {
      delay(100);

    }

    showNumber(i);
    lc.setChar(0, 7, (char)i, false);

    Serial.println((char)(byte)i);
  while (!digitalRead(resetSwitch)) {
      delay(100);

    }
  }


}




void init7SegDisplay() {

  /*
   The MAX72XX is in power-saving mode on startup,
   we have to do a wakeup call
   */
  lc.shutdown(0, false);
  /* Set the brightness to a medium values */
  lc.setIntensity(0, 5);
  /* and clear the display */
  lc.clearDisplay(0);

}

long power10[] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000};

inline void showNumber(int32_t number) {


  boolean negative = false;


  if (number < 0) {
    number = abs(number);
    negative = true;

  }
  boolean minusShown = false;

  for (int digit = 0; digit < 8; digit++) {
    long powerval = power10[digit]; //pow(10,digit);
    if ((digit > 0) && (number < powerval)) {
      if (negative && !minusShown) {
        lc.setChar(0, digit, '-', false);
        minusShown = true;
      }
      else {
        lc.setChar(0, digit, ' ', false);
      }

    } else {
      lc.setDigit(0, digit, (number / powerval) % 10, false);
    }
  }

}

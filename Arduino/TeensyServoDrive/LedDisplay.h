
LedControl lc = LedControl(ledCounterPins[0], ledCounterPins[1], ledCounterPins[2], 1);


String displayModeNames[] = {"P05", "ERROR", "FORCE", "PId P", "PID 1", "PID D", "Hall", "tAR9Et"};

const int DISPLAY_MODE_POSITION = 0;
const int DISPLAY_MODE_ERROR = 1;
const int DISPLAY_MODE_MOTOR = 2;
const int DISPLAY_MODE_P = 3;
const int DISPLAY_MODE_I = 4;
const int DISPLAY_MODE_D = 5;
const int DISPLAY_MODE_HALL = 6;
const int DISPLAY_MODE_TARGET = 7;

int displayMode = DISPLAY_MODE_POSITION;
int tempDisplayMode = -1;
unsigned long tempDisplayModeStartTime = 0;
const int numDisplayModes = 8;

long power10[] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000};

inline void showNumber(int32_t number); 
inline void showFloat(float number, int decimalPlaces);
inline void showLevel(float level, float range);
inline void showText(String text, boolean clear  = true);

void updateDisplay() { 

 if ((digitalRead(resetSwitch) == LOW) && (!buttonPressed)) {

    displayMode++;
    if (displayMode >= numDisplayModes) displayMode = 0;
    //displayMode = DISPLAY_MODE_HALL;
    //forwards = !forwards;
    buttonPressed = true;
  } else if (digitalRead(resetSwitch) == HIGH) {
    buttonPressed = false;
  }

  if (tempDisplayMode > -1) {
    if ((unsigned long)(millis() - tempDisplayModeStartTime > 2000)) tempDisplayMode = -1;
  }

  
  if (buttonPressed ) {
    showText(displayModeNames[displayMode]);
    // showNumber(currentMotorOutput);
    //  showText(currentStep, false);

  } else {



    switch ((tempDisplayMode == -1) ? displayMode : tempDisplayMode) {
      case DISPLAY_MODE_POSITION  :
        showNumber(position);
        showText("P", false);
        break;
        
     case DISPLAY_MODE_TARGET  :
        showNumber(targetPositionLong);
        showText("t", false);
        break;

      case DISPLAY_MODE_ERROR :
        if(errorValue<50)
          showLevel(targetPositionLong - position, 16);
        else 
          showNumber(targetPositionLong - position);
        showText("E", false);
        break;

      case DISPLAY_MODE_MOTOR :
        showLevel(motorPower, 100);
        showText("F", false);
        break;

      case DISPLAY_MODE_P :
        showFloat(Kp, 2);
        showText("P", false);
        break;

      case DISPLAY_MODE_I :
        showFloat(Ki, 5);
        showText("I", false);
        break;
      case DISPLAY_MODE_D :
        showFloat(Kd, 2);
        showText("d", false);
        break;

      case DISPLAY_MODE_HALL :
        //showFloat(Kd, 2);
        //showText("d", false);
        lc.clearDisplay(0);
        showNumber(currentStep);

        for (int i = 0; i < 3; i++) {
          if (motorOutputPhases[i] < 0)
            lc.setChar(0, 3 - i+4,  '-', false);
          else if (motorOutputPhases[i] > 0)
            lc.setChar(0, 3 - i+4,  'H', false);
          else
            lc.setChar(0, 3 - i+4,  'O', false);
        }
        
        lc.setChar(0, 4, (binaryHallSensors & 1) ? 'O' : '0', false);
        lc.setChar(0, 3, ((binaryHallSensors >> 1) & 1) ? 'O' : '0', false);
        lc.setChar(0, 2, ((binaryHallSensors >> 2) & 1) ? 'O' : '0', false);
         lc.setDigit(0, 1, stepForSensors[binaryHallSensors-1], false);
        break;
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
  lc.setIntensity(0, 2);
  /* and clear the display */
  lc.clearDisplay(0);

}

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

inline void showText(String text, boolean clear) {

  if (clear) lc.clearDisplay(0);
  int pos = 0;
  while (text[pos] != 0) {

    lc.setChar(0, 7 - pos, text.charAt(pos), false);
    pos++;
  }


}

 inline void showFloat(float number, int decimalPlaces = 2) {

  number *= power10[decimalPlaces];

  boolean negative = false;

  if (number < 0) {
    number = abs(number);
    negative = true;

  }
  boolean minusShown = false;

  for (int digit = 0; digit < 8; digit++) {
    long powerval = power10[digit]; //pow(10,digit);
    if ((digit > 0) && (number < powerval) && (digit > decimalPlaces)) {
      if (negative && !minusShown) {
        lc.setChar(0, digit, '-', false);
        minusShown = true;
      }
      else {
        lc.setChar(0, digit, ' ', false);
      }

    } else {
      lc.setDigit(0, digit, (int)(number / powerval) % 10, (digit == decimalPlaces));
    }
  }

}

inline void showLevel(float level, float range) {

  int displaylevel = map(abs(level), 0, range, 0, 16);
  lc.clearDisplay(0); 
  
  for (int i = 0; i < 8; i++) {
    if (displaylevel > (i*2)+1) lc.setCustom7Seg(0, i, 0b00110110 & ((level<0) ? 0b00100010 : 0b00010100), false);
    else if (displaylevel == (i*2)+1) lc.setCustom7Seg(0, i, 0b00110000 & ((level<0) ? 0b00100010 : 0b00010100), false);
  }


}



void showTempDisplay(int displaymode) {

  tempDisplayMode = displaymode;
  tempDisplayModeStartTime = millis();


}



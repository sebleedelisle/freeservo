
#ifdef USE_7SEG_DISPLAY
boolean errorDisplayed = false;

void updateDisplay() {
  //targetPosition = -6450210;

  if (servoError)  {

    if (!errorDisplayed) {
      matrix1.clear();
      matrix2.clear();
      matrix1.writeDigitRaw(4, 0b1111011); // e
      matrix2.writeDigitRaw(0, 0b1010000); // r
      matrix2.writeDigitRaw(1, 0b1010000); // r
      matrix2.writeDigitRaw(2, 0b1011100); // o
      matrix2.writeDigitRaw(3, 0b1010000); // r

      matrix1.writeDisplay();
      matrix2.writeDisplay();
      errorDisplayed = true;
    }
    return;

  }
  if (lastDisplayedPos == targetPosition) return;

  long matrix2num = (long)floor(abs(targetPosition)) % 10000;
  long matrix1num = (long)floor(abs(targetPosition) / 10000.0f);

  if (targetPosition < 0) matrix1num *= -1;

  matrix1.clear();
  matrix2.clear();
  matrix1.print(matrix1num);

  if (targetPosition < 0) {
    if (targetPosition > -1000) matrix2num *= -1;
    else if ((targetPosition <= -1000) && (targetPosition > -10000)) {
      matrix1.writeDigitRaw(4, 0b1000000);
    }

  }

  matrix2.print(matrix2num);
  if (abs(targetPosition) > 9999) {
    if (abs(matrix2num) < 1000) matrix2.writeDigitNum(0, 0);
    if (abs(matrix2num < 100)) matrix2.writeDigitNum(1, 0);
    if (abs(matrix2num < 10)) matrix2.writeDigitNum(3, 0);
  }

  if (matrix2num == 0) matrix2.writeDigitNum(4, 0);

  matrix1.writeDisplay();
  matrix2.writeDisplay();
  lastDisplayedPos = targetPosition;
  errorDisplayed = false;

}

void initDisplay() {
  matrix1.begin(0x71);
  matrix2.begin(0x70);

  lastDisplayedPos = 1;

  //updateDisplay();
  //matrix2.print(0xffff, HEX);
  matrix1.clear();
  matrix2.clear();
  matrix1.writeDigitRaw(4, 0b1010000); // r
  matrix2.writeDigitRaw(0, 0b1111011); // e
  matrix2.writeDigitRaw(1, 0b1101101); // s
  matrix2.writeDigitRaw(3, 0b1111011); // e
  matrix2.writeDigitRaw(4, 0b1111000); // t

  matrix1.writeDisplay();
  matrix2.writeDisplay();
  delay(1000);

}

#endif


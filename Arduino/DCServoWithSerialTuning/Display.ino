
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
      matrix2.writeDigitRaw(3, 0b1011100); // o
      matrix2.writeDigitRaw(4, 0b1010000); // r

      matrix1.writeDisplay();
      matrix2.writeDisplay();
      errorDisplayed = true;
    }
    return;

  }
  if (lastDisplayedPos == targetPositionDouble) return;

  long matrix2num = (long)floor(abs(targetPositionDouble)) % 10000;
  long matrix1num = (long)floor(abs(targetPositionDouble) / 10000.0f);

  if (targetPositionDouble < 0) matrix1num *= -1;

  matrix1.clear();
  matrix2.clear();
  matrix1.print(matrix1num);

  if (targetPositionDouble < 0) {
    if (targetPositionDouble > -1000) matrix2num *= -1;
    else if ((targetPositionDouble <= -1000) && (targetPositionDouble > -10000)) {
      matrix1.writeDigitRaw(4, 0b1000000);
    }

  }

  matrix2.print(matrix2num);
  if (abs(targetPositionDouble) > 9999) {
    if (abs(matrix2num) < 1000) matrix2.writeDigitNum(0, 0);
    if (abs(matrix2num < 100)) matrix2.writeDigitNum(1, 0);
    if (abs(matrix2num < 10)) matrix2.writeDigitNum(3, 0);
  }

  if (matrix2num == 0) matrix2.writeDigitNum(4, 0);

  matrix1.writeDisplay();
  matrix2.writeDisplay();
  lastDisplayedPos = targetPositionDouble;
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
  tone(buzzerPin, 2000, 300); 
  delay(320); 
  tone(buzzerPin, 1000, 300); 
  delay(400);

}

#endif


#ifdef USE_NEOPIXEL

int numPixels = 24; 
unsigned long lastPixelUpdate = 0; 
int lastpos = 0; 

void updatePixelDisplay() { 
  
  if(millis()  - lastPixelUpdate<10) return; 
  
  double error = targetPosition - position; 
  
  int pos = 0; 
  if(pos>=0) { 
    pos = (int) error; 
    if(pos>numPixels/2) pos = numPixels/2; 
  } else { 
    pos = 24 + (int) error; 
    if(pos<numPixels/2) pos = numPixels/2; 
  } 
  
  if(pos == lastpos) return; 
  strip.setPixelColor(0, strip.Color(100, 0, 0));
  for(int i = 0; i<numPixels; i++) { 
    
   if(pos == i)  strip.setPixelColor(i, strip.Color(255, 255, 255));
   else if(i!=0) strip.setPixelColor(i, strip.Color(0, 0, 0));
    
  }
  lastPixelUpdate = millis(); 
  strip.show(); 
 // Serial.println(error, pos); 
  
  lastpos = pos; 
  
}


#endif

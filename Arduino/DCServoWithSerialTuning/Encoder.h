
#ifdef USE_ENCODER_LIBRARY
//#define ENCODER_OPTIMIZE_INTERRUPTS

#include <Encoder.h>
#endif

#ifdef USE_ENCODER_LIBRARY

Encoder encoder(aPin, bPin);


void initEncoder() {
}

void updateEncoder() { 
  position = encoder.read(); 
  
  
}

#else

#define aPort PIND
#define aPortPin PD1
#define bPort PIND
#define bPortPin PD0

volatile unsigned int aState, bState;



void updateEncoder() { 
  
  
}


void aChange() {

  aState = (aPort & (1 << aPortPin)) >> aPortPin;

  // oooooo XOR!
  (bState ^ aState) ? position-- : position++;


  //  if (aState == HIGH) {
  //    // rising
  //    if (bState == LOW)
  //      position++;
  //    else
  //      position--;
  //  }
  //  else {
  //    // falling
  //    if (bState == HIGH)
  //      position++;
  //    else
  //      position --;
  //
  //  }

}

void bChange() {

  bState = (bPort & (1 << bPortPin)) >> bPortPin;
  bState^aState ? position++ : position--;

  //  if (bState == HIGH) {
  //    // rising
  //    if (aState == HIGH)
  //      position++;
  //    else
  //      position--;
  //  }
  //  else {
  //    // falling
  //    if (aState == LOW)
  //      position++;
  //    else
  //      position --;
  //
  //  }

}


void initEncoder() {
  
  pinMode(aPin, INPUT);
  pinMode(bPin, INPUT);
  attachInterrupt(3, aChange, CHANGE);
  attachInterrupt(2, bChange, CHANGE);

}

#endif



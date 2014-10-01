
#ifdef USE_ENCODER_LIBRARY
#define ENCODER_OPTIMIZE_INTERRUPTS

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



}

void bChange() {

  bState = (bPort & (1 << bPortPin)) >> bPortPin;
  bState^aState ? position++ : position--;

}


void initEncoder() {
  
  pinMode(aPin, INPUT);
  pinMode(bPin, INPUT);
  attachInterrupt(3, aChange, CHANGE);
  attachInterrupt(2, bChange, CHANGE);

}

#endif



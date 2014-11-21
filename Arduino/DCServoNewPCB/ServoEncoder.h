
#define ENCODER_OPTIMIZE_INTERRUPTS

Encoder encoder(aPin, bPin);

void initEncoder() {
}

void updateEncoder() { 
  position = encoder.read(); 
  
  
}


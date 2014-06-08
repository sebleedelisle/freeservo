#define PORTC_BIT_STEP 0x01  //PC0
#define PORTC_BIT_DIR 0x02  //PC1

volatile byte lastStepDir=0;
volatile long count = 0;


ISR(TIMER2_COMPA_vect)
{
  // Capture current port value 
  register byte thisStepDir = (PINC & (PORTC_BIT_STEP|PORTC_BIT_DIR));
  
  // check if there has been any change in the step bit
  if((thisStepDir ^ lastStepDir) & PORTC_BIT_STEP)
  {
    // check if the step pulse is now LOW (falling edge)
    if(!(thisStepDir & PORTC_BIT_STEP))
    {
      // apply direction (HIGH increments, LOW decrements)
      if(thisStepDir & PORTC_BIT_DIR)
        ++count;
      else
        --count;
    }
    
    // Store the new value (only need to record this when there
    // is a change in the step bit)
    lastStepDir = thisStepDir;    
  }  
}


void setup()
{
  pinMode(14,INPUT_PULLUP); // PCO on UNO: Step
  pinMode(15,INPUT_PULLUP); // PC1 on UNO: Direction
  delay(1); // allow inputs to settle;

  
  cli();
  // Configure timer 2 to call timer interrupt handler
  TCCR2A = _BV(WGM21); // clear timer on compare match
  TCCR2B = _BV(CS21); // timer 2 is at osc/8 = 2MHz
  TIMSK2 = _BV(OCIE2A); // interrupt on compare match A
  OCR2A = 20; // period register controls sample rate of step input, which in theory 
              // needs to be at least double the maximum allowed pulse rate 
              // In practice I found a value of 20 (100ksps) can reliably 
              // count pulses at 20kHz but not much higher
              // 
              // value = 2000000/samplerate
              //
              // too low and the interrupt handler disappears up its own backside!
  sei();

  Serial.begin(9600);    
}

void loop()
{
  Serial.println(count);
  delay(100);
}


#ifdef TRACE_TO_EEPROM

#define NUM_SAMPLES 30        // total number of samples to grab
#define EEPROM_LOCATION 100   // EPPROM address of the saved sample buffer

class CTracer {
  typedef struct {
    byte ms;                  // 1 byte
    double position;          // 8 bytes
    long targetPositionLong;  // 4 bytes
  } TRACE_ENTRY;
  
  TRACE_ENTRY buf[NUM_SAMPLES];
  int index;
  
public:
  CTracer() {
    memset(buf, 0, sizeof buf);
    index = 0;
  }
  void sample(double position, long targetPositionLong) {
    if(index >= NUM_SAMPLES)
      index = 0;
    TRACE_ENTRY& e=buf[index];
    e.ms = (byte)millis();
    e.position = position;
    e.targetPositionLong = targetPositionLong;
    ++index;
  }
  void save(int addr) {
    addr += EEPROM_writeAnything(addr, ':'); // "magic values" to tell us a log was indeed saved
    addr += EEPROM_writeAnything(addr, '-');
    addr += EEPROM_writeAnything(addr, '(');
    for(int i=0; i<NUM_SAMPLES; ++i) {
      TRACE_ENTRY& e=buf[index];
      addr += EEPROM_writeAnything(addr, e.ms);
      addr += EEPROM_writeAnything(addr, e.position);
      addr += EEPROM_writeAnything(addr, e.targetPositionLong);
      if(++index > NUM_SAMPLES)
        index = 0;
    }
    addr += EEPROM_writeAnything(addr, '.');
  }
};

CTracer _tracer;
#define TRACE_SAMPLE(a,b) _tracer.sample(a,b)
#define TRACE_SAVE() _tracer.save(EEPROM_LOCATION)

//////
#else 
//////

#define TRACE_SAMPLE(a,b)
#define TRACE_SAVE()  

//////
#endif



boolean on;

int lastT;
int t;
long duration = 50;

int pins[] = { 0,3,1,2 };
byte direction = 0;

byte pulse = 1;

void setup(){
  lastT = 0;
  
  pinMode(pins[0],OUTPUT);
  pinMode(pins[1],OUTPUT);
  pinMode(pins[3],OUTPUT);
  pinMode(pins[4],OUTPUT);
  
  
}

void loop(){
  
  t = micros();
  int elapsed = (t-lastT);
  if( elapsed > duration ){
    
      byte offset = direction<<1;
      digitalWrite(pins[offset+0],pulse?HIGH:LOW);
      digitalWrite(pins[offset+1],pulse?HIGH:LOW);
      lastT = t;
      pulse = !pulse;
  }
  
  
}

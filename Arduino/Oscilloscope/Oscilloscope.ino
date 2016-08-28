
const int pin1 = A0; 
const int pin2 = A1; 

byte data[2] = {0, 0};  

void setup() { 
  
  Serial.begin(115200);
  pinMode(pin1, INPUT); 
  pinMode(pin2, INPUT); 
  
  
}


void loop() { 
  //data[0] = analogRead(pin1)>>2; 
  //data[1] = analogRead(pin2)>>2;
  Serial.write( analogRead(pin1)>>2 );
  Serial.write( analogRead(pin2)>>2 ); 
  
}

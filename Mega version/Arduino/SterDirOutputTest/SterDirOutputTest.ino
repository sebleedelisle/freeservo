
//Alter the speed
int timerDelay = 10;


void setup() {
  // put your setup code here, to run once:
  pinMode(12, OUTPUT); 
  pinMode(13, OUTPUT); 
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000); 
   
  digitalWrite(13, HIGH); //direction
  delay(1);  //1 milisecond pause
  for(int i = 0; i<1000; i++) { 
    digitalWrite(12, HIGH); 
    delayMicroseconds(1); 
    digitalWrite(12, LOW); 
    delay(timerDelay);
    
  }
  digitalWrite(13, LOW); 
  delay(1000);  //1 second pause
  for(int i = 0; i<2000; i++) { 
    digitalWrite(12, HIGH); 
    delayMicroseconds(1); 
    digitalWrite(12, LOW); 
    delay(timerDelay);
    
  }
 digitalWrite(13, HIGH); 
  delay(1000); //1 second pause
  for(int i = 0; i<1000; i++) { 
    digitalWrite(12, HIGH); 
    delayMicroseconds(1); 
    digitalWrite(12, LOW); 
    delay(timerDelay);

  }
  
 

}

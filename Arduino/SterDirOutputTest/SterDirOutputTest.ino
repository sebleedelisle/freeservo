void setup() {
  // put your setup code here, to run once:
  pinMode(2, OUTPUT); 
  pinMode(3, OUTPUT); 
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000); 
   
  digitalWrite(3, HIGH); 
  delay(1); 
  for(int i = 0; i<1000; i++) { 
    digitalWrite(2, HIGH); 
    delayMicroseconds(1); 
    digitalWrite(2, LOW); 
    delay(10); 
    
  }
  digitalWrite(3, LOW); 
  delay(1000); 
  for(int i = 0; i<2000; i++) { 
    digitalWrite(2, HIGH); 
    delayMicroseconds(1); 
    digitalWrite(2, LOW); 
    delay(10); 
    
  }
 digitalWrite(3, HIGH); 
  delay(1000); 
  
  for(int i = 0; i<1000; i++) { 
    digitalWrite(2, HIGH); 
    delayMicroseconds(1); 
    digitalWrite(2, LOW); 
    delay(10); 
    
  }
  
 

}

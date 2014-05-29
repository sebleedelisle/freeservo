int brightness = 0; 
int change = 1; 

void setup() {
  // put your setup code here, to run once:
  pinMode(5,OUTPUT); 

  pinMode(13,OUTPUT); 
  

}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(5, brightness);
  analogWrite(13,brightness);
  delay(1); 
  
  brightness+=change; 
  if(brightness>=255) { 
    change*=-1; 
    brightness = 255;  
    
  } else if(brightness<0) { 
    change*=-1; 
   brightness = 0;  
    
  }
  
}

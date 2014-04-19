
int pwmPin = 3;
int dirPin = 12; 

void setup() {
  
//  TCCR0B - timer for PWM pins 13 and 4
//  TCCR1B - timer for PWM pins 12 and 11
//  TCCR2B - timer for PWM pins 10 and 9
//  TCCR3B - timer for PWM pins 5,3 and 2
//  TCCR4B - timer for PWM pins 8,7 and 6
  
  
  // right most three bits of the times are a frequency scaler 
  // if 1 then freq is 31000. 
  // scaler = 1 ---> PWM frequency is 31000 Hz
  // scaler = 2 ---> PWM frequency is 4000 Hz
  // scaler = 3 ---> PWM frequency is 490 Hz (default value)
  // scaler = 4 ---> PWM frequency is 120 Hz
  // scaler = 5 ---> PWM frequency is 30 Hz
  // scaler = 6 ---> PWM frequency is <20 Hz

  TCCR3B &= (0xff & 0x1); 
  
  
  
  pinMode (pwmPin, OUTPUT); 
  pinMode (dirPin, OUTPUT); 

  Serial.begin(9600); 
}

void loop() {
  // put your main code here, to run repeatedly:

  // speed goes between -1 and 1 on a sin wave
  float speed = sin(millis()/500.0f/PI) * 0.7f; 
  
  
  if(speed <0) digitalWrite(dirPin, LOW); 
  else digitalWrite(dirPin, HIGH); 
  
  analogWrite(pwmPin, abs(speed) *255.0f); 
  //Serial.print(speed, 2); 
  //Serial.print(" "); 
  //Serial.println((int)(abs(speed) *255.0f));
}

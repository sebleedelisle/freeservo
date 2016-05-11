class TrimPot {
  
  public : 

  TrimPot(int inputpin, float inputSmoothing = 0.1) { 

      pin = inputpin;
      smoothing = inputSmoothing;   
      previousValue = currentValue = targetValue = analogRead(pin);  
      minValue = 0; 
      maxValue = 1; 
      
  }

  boolean update() { 

    targetValue = analogRead(pin); 

    currentValue += ((float)targetValue-currentValue)*smoothing; 

    int change = abs(previousValue-currentValue); 
    previousValue = currentValue;
    
    return (change>1); 
    
  }

  void setLimits(float minvalue, float maxvalue) { 
      minValue = minvalue; 
      maxValue = maxvalue; 
  }

  float getValue() { 
      //Serial.println(currentValue); 
      return (((float)round(currentValue)/1023.0f) * (maxValue - minValue)) +minValue; 

    
  }

  int pin; 

  float currentValue; 
  float previousValue;
  int targetValue; 
  float smoothing; 

  float minValue; 
  float maxValue; 
  

  
};



#ifdef USE_7SEG_DISPLAY
void updateDisplay() { 
 //targetPosition = -6450210; 
  
  if(servoError) { 
   matrix1.printError(); 
   matrix2.printError();  
   matrix1.writeDisplay(); 
   matrix2.writeDisplay(); 
   return; 
    
  }
  if(lastDisplayedPos == targetPosition) return;
   
  long matrix2num = (long)floor(abs(targetPosition)) %10000; 
  long matrix1num = (long)floor(abs(targetPosition)/10000.0f);
  
  if(targetPosition<0) matrix1num*=-1; 
  
  matrix1.clear(); 
  matrix2.clear(); 
  matrix1.print(matrix1num); 
  
  if(targetPosition<0) { 
     if(targetPosition>-1000) matrix2num*=-1; 
     else if((targetPosition<=-1000) && (targetPosition>-10000)) { 
       matrix1.writeDigitRaw(4,0b1000000); 
     }
    
  }
  
  matrix2.print(matrix2num);
  if(abs(targetPosition)>9999) { 
    if(abs(matrix2num)<1000) matrix2.writeDigitNum(0, 0); 
    if(abs(matrix2num<100)) matrix2.writeDigitNum(1, 0); 
    if(abs(matrix2num<10)) matrix2.writeDigitNum(3, 0);
  }
  
  if(matrix2num==0) matrix2.writeDigitNum(4, 0);

  matrix1.writeDisplay(); 
  matrix2.writeDisplay(); 
  lastDisplayedPos = targetPosition; 
  
}

#endif


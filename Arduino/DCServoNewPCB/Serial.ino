#pragma once

#define serial Serial

void initSerial(){
 
    while( !Serial );
    serial.begin(115200);
}


void checkSerial() {

  // TODO - ADD ERROR CHECKING!
  
  if ( commandComplete ) {
    char commandStr[command.length() + 1];
    char *ptr;

    command.toCharArray( commandStr, command.length() + 1 );

    Kp = strtod(commandStr, &ptr);    
    ptr++;
    Ki = strtod(ptr, &ptr);
    ptr++;
    Kd = strtod(ptr, &ptr);
    myPID.SetTunings(Kp, Ki, Kd);
    
    ptr++; 
    sendPos = strtol(ptr, &ptr, 10); 
    
    ptr++;
    motorDeadZone = strtod(ptr, &ptr);

    writeEepromData(); 
    
    sendPIDOverSerial();
 
    commandComplete = false;
    command = "";

  }
}



void sendPIDOverSerial(){
  
  char buf[12];
  dtostrf( Kp, 12, 8, buf );
  serial.print("PID:");
  serial.print( buf );
  serial.print( ',' );
  dtostrf( Ki, 12, 8, buf );
  serial.print( buf );
  serial.print( ',' );
  dtostrf( Kd, 12, 8, buf );
  serial.print( buf );
  serial.print( ',' );
  dtostrf( motorDeadZone, 12, 8, buf );
  serial.print( buf );
  serial.println();

}

void incomingSerial() { 
    if (serial.available()) {
    // get the new byte:
    char inChar = (char)serial.read();
    // add it to the inputString:
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      commandComplete = true;
    } else {
      command += inChar;
    }
  }
}

void serialEvent() { 
  incomingSerial(); 
}



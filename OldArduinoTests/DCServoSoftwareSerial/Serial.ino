
void checkSerial() {

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

    byte eepromAddr = eepromDataAddr;
    EEPROM.write(eepromAddr++,eepromValidateData);
    eepromAddr+=EEPROM_writeAnything(eepromAddr,Kp);
    eepromAddr+=EEPROM_writeAnything(eepromAddr,Ki);
    eepromAddr+=EEPROM_writeAnything(eepromAddr,Kd);    
    //sendPIDOverSerial();
    //targetPosition = random(0,8000);

    commandComplete = false;
    command = "";

  }
}



void sendPIDOverSerial(){
  
  char buf[12];
  dtostrf( Kp, 12, 8, buf );
  Serial.print("PID:");
  Serial.print( buf );
  Serial.print( ',' );
  dtostrf( Ki, 12, 8, buf );
  Serial.print( buf );
  Serial.print( ',' );
  dtostrf( Kd, 12, 8, buf );
  Serial.print( buf );
  Serial.println();

}



void serialEvent() {
  if (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
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

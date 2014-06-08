import controlP5.*;
import processing.serial.*;
import java.text.*;

Serial serial;
ControlP5 cp5;

int myColorBackground = color(0,0,0);

Knob knobKp;
Knob knobKi;
Knob knobKd;
boolean supressSerialSend;
float KiValue = 0;
float kdScale = 100;

void setup() {
  size(640,200);
  smooth();
  noStroke();
  
  cp5 = new ControlP5(this);
  
  knobKp = cp5.addKnob("Kp")
               .setRange(0.001,5)
               .setValue(0.88)
               .setPosition(20,20)
               .setRadius(50)
               .setDragDirection(Knob.VERTICAL)
               ;
  
  knobKi = cp5.addKnob("Ki")
               .setRange(0,1)
               .setValue(0.16)
               .setPosition(120,20)
               .setRadius(50)
               .setDragDirection(Knob.VERTICAL)
               ;
               
   knobKd = cp5.addKnob("Kd")
               .setRange(0,1)
               .setValue(0.9)
               .setPosition(220,20)
               .setRadius(50)
               .setDragDirection(Knob.VERTICAL)
               ;
  
  DropdownList l = cp5.addDropdownList("Select port",400,20,200,200).setId(0);
  String[] serialPorts = Serial.list();      
  for(int i=0;i<serialPorts.length;i++){
    l.addItem(serialPorts[i],i);
  }
               
}

void draw() {
  background(myColorBackground);
  fill(0);
  rect(0,0,width,height);
  if( serial!=null && serial.available() > 0 ) {
     String valuesStr = serial.readStringUntil('\n');
     if( valuesStr != null ) {
       float[] values = float( valuesStr.split(",") );
       print( "Receiving: "+valuesStr );
       //println( values );
       
       supressSerialSend = true;
       knobKp.setValue( values[0] );
       knobKi.setValue( values[1] );
       knobKd.setValue( values[2] / kdScale );
       supressSerialSend = false;
       
     }
  }
  
}



void controlEvent(ControlEvent theEvent) {
  if (theEvent.isGroup()) {
    int value = (int)theEvent.getGroup().getValue();
    
    try{
      if( serial != null ){
        serial.clear();
        serial.stop();
      }
      serial = new Serial(this, Serial.list()[value], 9600);
     
    }
    catch(Exception e){
      
    }
  }
  
  else {
     if( (knobKp == null) || (knobKi == null) || (knobKd == null) ) return; 
  
        ResendValues();
          
      

  }
}

void keyPressed() {
    ResendValues();
  
}

void ResendValues(){
  if( supressSerialSend ) return;
  String v1 = String.format("%e",knobKp.getValue());
  String v2 = String.format("%e",knobKi.getValue());
  String v3 = String.format("%e",knobKd.getValue()*kdScale);
  String msg = v1 +","+ v2 +","+ v3+"\n";  
  print( "Sending: "+msg );
  if( serial != null ){
    try{
      serial.write( msg.getBytes() );
    }
    catch(Exception e){
      
    }
  }
}

import controlP5.*;
import processing.serial.*;

Serial serial;
ControlP5 cp5;

int myColorBackground = color(0,0,0);
int knobValue = 100;

Knob myKnobA;

void setup() {
  size(640,200);
  smooth();
  noStroke();
  
  cp5 = new ControlP5(this);
  
  myKnobA = cp5.addKnob("knob")
               .setRange(0,255)
               .setValue(50)
               .setPosition(20,20)
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
  
}


void knob(float theValue) {
  //myColorBackground = color(theValue);
  //println("a knob event. setting background to "+theValue);
  //serial.write( ("").getBytes() );
  println( theValue );
  if( serial != null ){
    try{
      serial.write( ( theValue+"\n" ).getBytes() );
    }
    catch(Exception e){
      
    }
      
  }
}

void controlEvent(ControlEvent theEvent) {
  if (theEvent.isGroup()) {
    int value = (int)theEvent.getGroup().getValue();
    print(value);
    serial.clear();
    serial.stop();
    try{
      serial = new Serial(this, Serial.list()[value], 28800);
    }
    catch(Exception e){
      
    }
  }
  
  else if( theEvent.getController().getId() == 0 ){
    
   
  }
}

void keyPressed() {
  
  
}

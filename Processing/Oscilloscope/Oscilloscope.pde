import controlP5.*;
import processing.serial.*;
import java.text.*;

Serial serial;
ControlP5 cp5;

int myColorBackground = color(0, 0, 0);

Slider knobXScale;
Slider knobYScale;

IntList values1;
IntList values2; 

void setup() {
  size(800, 600);
  smooth();
  noStroke();

  values1 = new IntList(); 
  values2 = new IntList(); 

  cp5 = new ControlP5(this);

  knobXScale = cp5.addSlider("X")
    .setSize(width-80, 15)
      .setRange(1, 200)
        .setValue(1)
          .setPosition(20, height - 130)
            ;

  knobYScale = cp5.addSlider("Y")
    .setSize(width-80, 15)
      .setRange(1, 200)
        .setValue(1)
          .setPosition(20, height-100)
           ;


  DropdownList l = cp5.addDropdownList("Select port")
    .setId(0)
      .setPosition(20, 20)
        .setSize(600, 600);

  String[] serialPorts = Serial.list();      
  for (int i=0;i<serialPorts.length;i++) {
    l.addItem(serialPorts[i], i);
  }
}

void draw() {
  background(myColorBackground);
  fill(0);
  rect(0, 0, width, height);
  while ( serial!=null && serial.available () > 0 ) {
    String message = serial.readStringUntil('\n');
    // String[] parts = message.split(":") ; 
    if ((message!=null) && (message.indexOf(':')>-1)) { 
      String[] parts = message.split(":") ; 
      if (parts[0].equals("PID")) { 
        println();
        print( "Receiving: "+message );

        if (( parts.length>1) && (parts[1] != null )) {
          float[] values = float( parts[1].split(",") );
          if (values.length==4) { 
            //println( values );
            print( "Setting PID : "+values[0]+" " + values[1] + " " + values[2]* kdScale + " " + values[3] );
            supressSerialSend = true;
            knobKp.setValue( values[0] );
            knobKi.setValue( values[1] );
            knobKd.setValue( values[2] * kdScale );
            knobDeadZone.setValue( values[3]  );
            supressSerialSend = false;
          }
        }
      } 
      else if (parts[0].equals("POS")) {
        if (( parts.length>1) && ( parts[1] != null )) {
          println(message); 
          float[] values = float( parts[1].split(",") );
          if (values.length>1) {
            float pos = values[0]; 
            float target = values[1]; 
            float error = target-pos; 
            errors.append(error); 
            if (errors.size()>width-10) errors.remove(0);
          }
        }
      }
    }
  }
  noFill(); 
  stroke(100); 
  line(0, 300, width, 300); 
  stroke(255); 
  beginShape(); 
  for (int i =0; i<errors.size(); i++) { 
    vertex(i, 300 + errors.get(i));
  }
  endShape(); 
  noStroke();
}



void controlEvent(ControlEvent theEvent) {
  if (theEvent.isGroup()) {
    int value = (int)theEvent.getGroup().getValue();

    try {
      if ( serial != null ) {
        serial.clear();
        serial.stop();
      }
      serial = new Serial(this, Serial.list()[value], 115200);
    }
    catch(Exception e) {
    }
  }

  else {

    if ( (knobKp == null) || (knobKi == null) || (knobKd == null)|| (knobDeadZone == null) ) return; 

    // knob has changed
  }
}

void keyPressed() {
}


import controlP5.*;
import processing.serial.*;
import java.text.*;

Serial serial;
ControlP5 cp5;

int myColorBackground = color(0, 0, 0);

Slider knobKp;
Slider knobKi;
Slider knobKd;

Slider knobDeadZone;

Toggle sendPos; 

boolean supressSerialSend;
float KiValue = 0;
float kdScale = 100;
FloatList errors, targetPositions, positions; 

void setup() {
    size(800, 600);
    smooth();
    noStroke();
 errors = new FloatList(); 
 positions = new FloatList(); 
 targetPositions = new FloatList(); 
    cp5 = new ControlP5(this);

    knobKp = cp5.addSlider("P")
        .setSize(width-80, 15)
            .setRange(0.001, 200)
                .setValue(0.88)
                    .setPosition(20, height - 130)
                        // .setRadius(50)
                        // .setDragDirection(Knob.VERTICAL)
                        ;

    knobKi = cp5.addSlider("I")
        .setSize(width-80, 15)
            .setRange(0, 20)
                .setValue(0.16)
                    .setPosition(20, height-100)
                        //.setRadius(50)
                        //.setDragDirection(Knob.VERTICAL)
                        ;

    knobKd = cp5.addSlider("D")
        .setSize(width-80, 15)
            .setRange(0, 100)
                .setValue(0.9)
                    .setPosition(20, height-70)
                        //.setRadius(50)
                        //.setDragDirection(Knob.VERTICAL)
                        ;
   knobDeadZone = cp5.addSlider("DeadZone")
        .setSize(width-80, 15)
            .setRange(0, 0.5)
                .setValue(0)
                    .setPosition(20, height-40)
                        //.setRadius(50)
                        //.setDragDirection(Knob.VERTICAL)
                        ;
    sendPos = cp5.addToggle("send data");
    
    
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
    while ( serial!=null && serial.available() > 0 ) {
        String message = serial.readStringUntil('\n');
        // String[] parts = message.split(":") ; 
        if ((message!=null) && (message.indexOf(':')>-1)) { 
            String[] parts = message.split(":") ; 
            if (parts[0].equals("PID")) { 
             println();
            print( "Receiving: "+message );
         
                if (( parts.length>1) && (parts[1] != null )) {
                    float[] values = float( parts[1].split(",") );
                    if(values.length==4) { 
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
            } else if (parts[0].equals("POS")) {
                if (( parts.length>1) && ( parts[1] != null )) {
                    println(message); 
                    float[] values = float( parts[1].split(",") );
                    if(values.length>1) {
                        float pos = values[0]; 
                        float target = values[1]; 
                        float error = target-pos; 
                        errors.append(error); 
                        positions.append(pos); 
                        targetPositions.append(target); 
                        if(errors.size()>width-10) errors.remove(0); 
                        if(positions.size()>width-10) positions.remove(0); 
                        if(targetPositions.size()>width-10) targetPositions.remove(0); 
                        
                        
                    }
                }
            }
        }
    }
    noFill(); 
    stroke(100); 
    line(0,300, width,300); 
    stroke(255); 
    beginShape(); 
    for(int i =0; i<errors.size(); i++) { 
        vertex(i, 300 + errors.get(i)); 
        
    }
    endShape(); 
    
    stroke(0,255,0); 
    beginShape(); 
    for(int i =0; i<targetPositions.size(); i++) { 
        vertex(i, ((targetPositions.get(i)/1.0f)%600) ); 
        
    }
    endShape(); 
    
   stroke(255,0,0); 
    beginShape(); 
    for(int i =0; i<positions.size(); i++) { 
        vertex(i,  ((positions.get(i)/1.0f)%600) ); 
        
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

        ResendValues();
    }
}

void keyPressed() {
    ResendValues();
}

void ResendValues() {
    if ( supressSerialSend ) return;
    String v1 = String.format("%e", knobKp.getValue());
    String v2 = String.format("%e", knobKi.getValue());
    String v3 = String.format("%e", knobKd.getValue()/kdScale);
    String v4 = (sendPos.getValue()!=0) ? "1" : "0"; 
    String v5 = String.format("%e", knobDeadZone.getValue());
    
    String msg = v1 +","+ v2 +","+ v3+","+v4+","+v5+"\n";  
    println( "Sending: "+msg );
    if ( serial != null ) {
        try {
            serial.write( msg.getBytes() );
        }
        catch(Exception e) {
        }
    }
}


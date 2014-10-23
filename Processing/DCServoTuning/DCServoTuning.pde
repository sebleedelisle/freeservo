import controlP5.*;
import processing.serial.*;
import java.text.*;

Serial serial;
ControlP5 cp5;

int myColorBackground = color(0, 0, 0);

Slider knobKp;
Slider knobKi;
Slider knobKd;

Toggle sendPos; 

boolean supressSerialSend;
float KiValue = 0;
float kdScale = 100;
FloatList errors; 

void setup() {
    size(800, 600);
    smooth();
    noStroke();
 errors = new FloatList(); 
    cp5 = new ControlP5(this);

    knobKp = cp5.addSlider("Kp")
        .setSize(width-40, 15)
            .setRange(0.001, 200)
                .setValue(0.88)
                    .setPosition(20, height - 100)
                        // .setRadius(50)
                        // .setDragDirection(Knob.VERTICAL)
                        ;

    knobKi = cp5.addSlider("Ki")
        .setSize(width-40, 15)
            .setRange(0, 20)
                .setValue(0.16)
                    .setPosition(20, height-70)
                        //.setRadius(50)
                        //.setDragDirection(Knob.VERTICAL)
                        ;

    knobKd = cp5.addSlider("Kd")
        .setSize(width-40, 15)
            .setRange(0, 100)
                .setValue(0.9)
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
                    //println( values );
                    print( "Setting PID : "+values[0]+" " + values[1] + " " + values[2]* kdScale );
                    supressSerialSend = true;
                    knobKp.setValue( values[0] );
                    knobKi.setValue( values[1] );
                    knobKd.setValue( values[2] * kdScale );
                    supressSerialSend = false;
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
                        if(errors.size()>width-10) errors.remove(0); 
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
        if ( (knobKp == null) || (knobKi == null) || (knobKd == null) ) return; 

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
    String v4 = (sendPos.getValue()) ? "1" : "0"; 
    
    String msg = v1 +","+ v2 +","+ v3+","+v4+"\n";  
    print( "Sending: "+msg );
    if ( serial != null ) {
        try {
            serial.write( msg.getBytes() );
        }
        catch(Exception e) {
        }
    }
}


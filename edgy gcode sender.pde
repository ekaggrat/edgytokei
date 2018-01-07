/**
* ControlP5 Textfield
*
*
* find a list of public methods available for the Textfield Controller
* at the bottom of this sketch.
*
* by Andreas Schlegel, 2012
* www.sojamo.de/libraries/controlp5
*
*/


import controlP5.*;
import processing.serial.*;

ControlP5 cp5;
Textarea myTextarea;
String inBuffer;
Serial myPort;  // The serial port
Knob rot_x;
Knob rot_y;
Knob feed;
int feed_value = 500;

String textValue = "";

void setup() {
  size(710,360);
  
  PFont font = createFont("arial",20);
  
  cp5 = new ControlP5(this);
  
  
                 
  cp5.addTextfield("commands")
     .setPosition(10,300)
     .setSize(270,40)
     //.setFont(createFont("arial",20))
     .setAutoClear(false)
     ;
 
       
       
  cp5.addBang("clear")
     .setPosition(310,300)
     .setSize(50,40)
     .getCaptionLabel().align(ControlP5.CENTER, ControlP5.CENTER)
     ;    
  
  
     
  myTextarea = cp5.addTextarea("txt")
                  .setPosition(500,10)
                  .setSize(200,330)
                  
                  .setLineHeight(14)
                  .setColor(color(200))
                  .setColorBackground(color(255,100))
                  ;   
     
  textFont(font);
  
   cp5.addButton("X_left")
     .setValue(0)
     .setPosition(70,120)
     .setSize(50,50)
     ;
  cp5.addButton("X_right")
     .setValue(0)
     .setPosition(170,120)
     .setSize(50,50)
     ;
  cp5.addButton("Y_up")
     .setValue(0)
     .setPosition(120,70)
     .setSize(50,50)
     ;
  cp5.addButton("Y_down")
     .setValue(0)
     .setPosition(120,170)
     .setSize(50,50)
     ;
  //fast buttons
  
  cp5.addButton("X_left_10")
     .setValue(0)
     .setPosition(10,120)
     .setSize(50,50)
     ;
  cp5.addButton("X_right_10")
     .setValue(0)
     .setPosition(230,120)
     .setSize(50,50)
     ;
  cp5.addButton("Y_up_10")
     .setValue(0)
     .setPosition(120,10)
     .setSize(50,50)
     ;
  cp5.addButton("Y_down_10")
     .setValue(0)
     .setPosition(120,230)
     .setSize(50,50)
     ;
  
  // homes
  cp5.addButton("X_top")
     .setValue(0)
     .setPosition(10,10)
     .setSize(80,80)
     ;
  
    cp5.addButton("X_bot")
     .setValue(0)
     .setPosition(10,200)
     .setSize(80,80)
     ;
  
  cp5.addButton("Y_top")
     .setValue(0)
     .setPosition(200,10)
     .setSize(80,80)
     ;
  
    cp5.addButton("Y_bot")
     .setValue(0)
     .setPosition(200,200)
     .setSize(80,80)
     ;
  //REPORT BUTTONS
    
      cp5.addButton("mot_on_y")
     .setValue(0)
     .setPosition(310,200)
     .setSize(50,35)
     ;
     
     cp5.addButton("mot_off_y")
     .setValue(0)
     .setPosition(310,245)
     .setSize(50,35)
     ;
     
     cp5.addButton("position")
     .setValue(0)
     .setPosition(310,120)
     .setSize(50,50)
     ; 
       cp5.addButton("mot_on_x")
     .setValue(0)
     .setPosition(310,10)
     .setSize(50,35)
     ; 
     cp5.addButton("mot_off_x")
     .setValue(0)
     .setPosition(310,55)
     .setSize(50,35)
     ;
     
     
  // utilities
   cp5.addButton("end_stop")
     .setValue(0)
     .setPosition(380,300)
     .setSize(50,40)
     ; 
   cp5.addButton("sens_val")
     .setValue(0)
     .setPosition(440,300)
     .setSize(50,40)
     ; 
     
  cp5.addButton("quad")
     .setValue(0)
     .setPosition(380,245)
     .setSize(110,35)
     ; 
     
  //knobs
  
   rot_x = cp5.addKnob("feed")
               .setRange(10,400)
               .setValue(100)
               .setPosition(390,10)
               .setRadius(40)
               .setDragDirection(Knob.HORIZONTAL)
               ;
   
  
  
  printArray(Serial.list());
  // Open the port you are using at the rate you want:
  myPort = new Serial(this, Serial.list()[1], 57600);
   background(0);
  fill(255);
  
}

void draw() {
  background(0);
  fill(255);
 
  
  
  
  
  //myPort.write("hello");
}

public void clear() {
  cp5.get(Textfield.class,"textValue").clear();
}

//void controlEvent(ControlEvent theEvent) {
//  if(theEvent.isAssignableFrom(Textfield.class)) {
 //   println(theEvent.getName()+": "+theEvent.getStringValue() );
 /// }
  
//}


public void commands(String theText) {
  // automatically receives results from controller input
  println("a textfield event for controller 'input' : "+theText);
  myPort.write(theText);
  //myPort.write(';');
  
  
}

void dumpport(){
  while (myPort.available() > 0) {
     inBuffer = myPort.readString();   
    if (inBuffer != null) {
      println(inBuffer);
      myTextarea.setText(inBuffer);
    }
  
  }
} 


// button functions

public void X_left(int theValue) {
  println("a button event from X+: "+theValue);
  myPort.write ("G91;");
  myPort.write ("G0 X-2"+" "+"F"+feed_value+";");
  dumpport();
}

public void X_right(int theValue) {
  println("a button event from X+: "+theValue);
  myPort.write ("G91;");
  myPort.write ("G0 X2"+" "+"F"+feed_value+";");
  dumpport();
}

public void Y_up(int theValue) {
  println("a button event from X+: "+theValue);
  myPort.write ("G91;");
  myPort.write ("G0 Y2"+" "+"F"+feed_value+";");
  dumpport();
}

public void Y_down(int theValue) {
  println("a button event from X+: "+theValue);
  myPort.write ("G91;");
  myPort.write ("G0 Y-2"+" "+"F"+feed_value+";");
  dumpport();
}  
// FAST BUTTONS

public void X_left_10(int theValue) {
  println("a button event from X+: "+theValue);
  myPort.write ("G91;");
  myPort.write ("G0 X-10"+" "+"F"+feed_value+";");
  dumpport();
}
  
public void X_right_10(int theValue) {
  println("a button event from X+: "+theValue);
  myPort.write ("G91;");
  myPort.write ("G0 X10"+" "+"F"+feed_value+";");
  dumpport();
}

public void Y_up_10(int theValue) {
  println("a button event from X+: "+theValue);
  myPort.write ("G91;");
  myPort.write ("G0 Y10"+" "+"F"+feed_value+";");
  println("a button event from X+: "+"G0 Y10"+"F"+feed_value+";");
  dumpport();
}
public void Y_down_10(int theValue) {
  println("a button event from X+: "+theValue);
  myPort.write ("G91;");
  myPort.write ("G0 Y-10"+" "+"F"+feed_value+";");
  dumpport();
}

// home 

public void X_top(int theValue) {
  println("a button event from X+: "+theValue);
  myPort.write ("G28;");
  //myPort.write ("G0 Y-10;");
  dumpport();
}

public void X_bot(int theValue) {
  println("a button event from X+: "+theValue);
  myPort.write ("G29;");
  //myPort.write ("G0 Y-10;");
  dumpport();
}

public void Y_top(int theValue) {
  println("a button event from X+: "+theValue);
  myPort.write ("G30;");
  //myPort.write ("G0 Y-10;");
  dumpport();
 
}




public void Y_bot(int theValue) {
  println("a button event from X+: "+theValue);
  myPort.write ("G31;");
  //myPort.write ("G0 Y-10;");
  dumpport();
}

// extra

public void motor_off(int theValue) {
  println("a button event from X+: "+theValue);
  myPort.write ("M18;");
  //myPort.write ("G0 Y-10;");
  dumpport();
}

public void mot_on_x(int theValue) {
  println("a button event from X+: "+theValue);
  myPort.write ("M40;");
  //myPort.write ("G0 Y-10;");
  dumpport();
}

public void mot_off_x(int theValue) {
  println("a button event from X+: "+theValue);
  myPort.write ("M41;");
  //myPort.write ("G0 Y-10;");
  dumpport();
}


public void mot_on_y(int theValue) {
  println("a button event from X+: "+theValue);
  myPort.write ("M50;");
  //myPort.write ("G0 Y-10;");
  dumpport();
}

public void mot_off_y(int theValue) {
  println("a button event from X+: "+theValue);
  myPort.write ("M51;");
  //myPort.write ("G0 Y-10;");
  dumpport();
}


public void position(int theValue) {
  println("a button event from X+: "+theValue);
  myPort.write ("M114;");
  dumpport();
  
  
  
  //myPort.write ("G0 Y-10;");
}


// rotary knobs






void feed(int theValue) {
   feed_value = theValue;
   //myPort.write("G0 Y" +position +"F"+feed+";");
  println("a knob event. setting background to "+theValue);
}

//utilities


public void sens_val(int theValue) {
  println("a button event from X+: "+theValue);
  myPort.write ("M256;");
  dumpport();
  //myPort.write ("G0 Y-10;");
}

public void end_stop(int theValue) {
  println("a button event from X+: "+theValue);
  myPort.write ("M119;");
  dumpport();
  //myPort.write ("G0 Y-10;");
}


public void quad(int theValue) {
  println("a button event from X+: "+theValue);
  myPort.write ("M200;");
  dumpport();
  //myPort.write ("G0 Y-10;");
}

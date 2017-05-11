/* -----------------------------------------------------------------------------
 * Example .ino file for arduino, compiled with CmdMessenger.h and
 * CmdMessenger.cpp in the sketch directory. 
 *----------------------------------------------------------------------------*/

#include "CmdMessenger.h"

#include <Servo.h> //servo library
#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX
Servo myservo; // create servo object to control servo
int Echo = A4;  
int Trig = A5; 
int in1 = 6; // These are the motor pins...
int in2 = 7;
int in3 = 8;
int in4 = 9;
int ENA = 5; // These are the pins that enable all motors
int ENB = 11;
int Speed = 150;
int SpeedL = 50;
int SpeedR = 50;
int rightDistance = 0,leftDistance = 0,middleDistance = 0 ;

/* Define available CmdMessenger commands */
enum {
    motors,
    get_sonar,
    sonar,
    sonar_angle,
    line_tracker,
    ir_in,
    error
};

/* Initialize CmdMessenger -- this should match PyCmdMessenger instance */
const int BAUD_RATE = 9600;
CmdMessenger c = CmdMessenger(Serial,',',';','/');

/* Create callback functions to deal with incoming messages */



/* callbacks */
void on_motors(void){
  // do motor stuff
  digitalWrite(in1,LOW);//digital output
  digitalWrite(in2,HIGH);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
  int Speed = c.readBinArg<int>();
  int SpeedL = c.readBinArg<int>();
  int SpeedR = c.readBinArg<int>();
  SpeedL = (SpeedL + Speed);
  SpeedR = (SpeedR + Speed);
  analogWrite(ENA,SpeedL);
  analogWrite(ENB,SpeedR);
}

void on_get_sonar(void){
   
// do sonar stuff
  digitalWrite(Trig, LOW);   
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  
  delayMicroseconds(20);
  digitalWrite(Trig, LOW);   
  int distance = pulseIn(Echo, HIGH);  
  distance= distance/58;       
  c.sendBinCmd(sonar,distance);
} 

void on_sonar_angle(void){
    // do sonar angle stuff
}

void on_line_tracker(void){
    // do line tracker stuff
}

void on_ir_in(void){
    // do ir in stuff
}

void on_error(void){
    // do error stuff
}

void on_unknown_command(void){
    c.sendCmd(error,"Command without callback.");
}

/* Attach callbacks for CmdMessenger commands */
void attach_callbacks(void) { 
  
    c.attach(motors,on_motors);
    c.attach(get_sonar,on_get_sonar);
    c.attach(sonar_angle,on_sonar_angle);    
    c.attach(line_tracker,on_line_tracker);
    c.attach(ir_in,on_ir_in);
    c.attach(error,on_error);
    c.attach(on_unknown_command);
}

void setup() {
    Serial.begin(BAUD_RATE);
    mySerial.begin(9600);
    mySerial.println("Hello, BT world");
    attach_callbacks();
    myservo.attach(3);// attach servo on pin 3 to servo object
    pinMode(Echo, INPUT);    
    pinMode(Trig, OUTPUT);  
    pinMode(in1,OUTPUT);
    pinMode(in2,OUTPUT);
    pinMode(in3,OUTPUT);
    pinMode(in4,OUTPUT);
    pinMode(ENA,OUTPUT);
    pinMode(ENB,OUTPUT);   
}

void loop() {
    c.feedinSerialData();
    if (mySerial.available()) 
      {
    mySerial.println("status update");
      }
}




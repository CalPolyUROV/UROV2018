#include <Servo.h>
#include "Math.h"
#include "Accelerometer.h"
#include "dataStruc.h"
//#include "QuadMotorShields.h"
#include "gyroAccelerometer.h"
#include <SoftwareSerial.h>
#include <SPI.h>
//#include "pressure.h"
#include <Wire.h>
#include "ComsMasterArd.h"
#include "VectorMotors.h"
#include "cameras.h"

//all pins used must be listed here! either as a variable to change quickly later or as a comment if it is in another file

int serialWritePin = 2; //this is the pin to control whethgeter it is recieving or sending
// SDA, SCL to Slave arduino


///// Pins used by Quad Motor Shields //////
//
// 7, 6, 11, 12, 22, 24, 31, 33, 29, 25, 27, 23, A1, 
//42, 44, 40, 24, 26, 22
//
//////////////////////////////////

///// Pins used by Pressure Sensor //////
//
//45, 50, 51, 52
//
//////////////////////////////////

//End of pin listings

///// Timers used by Quad Motor Shields on MEGA 2560//////
//
// Timer0, Timer1, Timer4
//
//////////////////////////////////

ComsMasterArd coms;
//QuadMotorShields md;//Not being used anymore
bool pressure = false;
bool voltage = false;
bool temperature = false;
bool accel = false;
bool depth = false;
bool ypr = false;

bool debug = true;

int yaw;
int pch;
int rol;
//SoftwareSerial Serial3(14, 15);
void setup() {
	Serial3.begin(9600);   //the number in here is the baud rate, it is the communication speed, this must be matched in the python
	Serial.begin(9600);     //it does not seem to work at lower baud rates 
	pinMode(serialWritePin, OUTPUT);
	pinMode(13, OUTPUT);
	pinMode(22, OUTPUT);
	pinMode(24, OUTPUT);
	digitalWrite(serialWritePin, LOW);
        motorSetup();
}

//looks cleaner than the empty while loop being everywhere in the code
void wait(){
  while(!Serial3.available())
    ;
}

//detects the start signal without accidently overshooting it because of short circuting logic
//except for input such as SSTR, that will be skipped. There should be multiple characters to 
//prevent random bytes getting past
void waitForStart(){
  while(true){
    wait();
    if('S' == Serial3.read()){
      wait();
      if('T' == Serial3.read()){
        wait();
        if('R' == Serial3.read()){
          break;
        }
      }
    }
  }
}

Input readBuffer() {  
        Input input;
        wait();//makes sure that a byte of data is not missed 
        input.buttons1 = Serial3.read();
        wait();
        input.buttons2 = Serial3.read();
        wait();
        input.primaryX = Serial3.parseInt();
        Serial3.read();
        wait();
        input.primaryY = Serial3.parseInt();
        Serial3.read();
        wait();
        input.secondaryX = Serial3.parseInt();
        Serial3.read();
        wait();
        input.secondaryY = Serial3.parseInt();
        Serial3.read();
        wait();
        input.triggers = Serial3.parseInt();
        Serial3.read();
        return input;
}
void processInput(Input i){
  /*
    if((CHECK_BIT(i.buttons1, 2))){digitalWrite(22, HIGH);}
        else {digitalWrite(22, LOW);}
    if((CHECK_BIT(i.buttons1, 2))){digitalWrite(24, LOW);}
        else {digitalWrite(24, HIGH);}
    */
  setCameras(i.buttons1);
  setMotors(i.primaryX, i.primaryY, i.triggers, i.secondaryX,i.buttons1);
}

void writeToCommand(Input i){
  Serial3.print("STR");
  int lines = 0;
  if (pressure) lines += 2;
  if (voltage) lines += 2;
  if (temperature) lines += 2;
  if (depth) lines += 2;
  if (accel) lines += 4;
  if (ypr) lines += 6;
  String numberOfLines = String(lines);
  int counter = 0;
  while ((counter + numberOfLines.length()) != 3) {
	  Serial3.print("0");
	  counter++;
  }
  Serial3.print(numberOfLines); //print the number of lines of input the python program can read in three digits
  if (pressure) {
	  Serial3.println("PSR"); //tell it the next line is Pressure
    coms.sendSlaveCmd(GET_PRES);
	  Serial3.print(coms.getSlaveData());
	  Serial3.println(" mbars");
  }
  if (voltage) {
	  Serial3.println("VLT"); //tell it the next line is Power info
	  Serial3.print( (((float)(analogRead(A1))*(5.0/1023.0))-2.52)   /.066 );
	  Serial3.println(" amps");
  }
  if (temperature) {
	  Serial3.println("TMP"); //tell it the next line is Temperature
    coms.sendSlaveCmd(GET_TEMP);
	  Serial3.print((float)analogRead(A0)/(2.048)-273.15);
	  //Serial3.print(coms.getSlaveData());
	  Serial3.println(" degrees C");
  }
  if (ypr) {
    
    coms.sendSlaveCmd(GET_YAW);
    Serial3.println("YAW");
    Serial3.println(coms.getSlaveData());
    //Serial3.println(coms.getSlaveData());
    
    coms.sendSlaveCmd(GET_PCH);
    Serial3.println("PCH");
    Serial3.println(coms.getSlaveData());
    
    coms.sendSlaveCmd(GET_ROL);
    Serial3.println("ROL");
    Serial3.println(coms.getSlaveData());


    if(debug){
      
    coms.sendSlaveCmd(GET_YAW);
    Serial.println("YAW");
    yaw = coms.getSlaveData();
    
    Serial.println(yaw);
    //Serial3.println(coms.getSlaveData());
    
    coms.sendSlaveCmd(GET_PCH);
    Serial.println("PCH");
    Serial.println(coms.getSlaveData());
    
    coms.sendSlaveCmd(GET_ROL);
    Serial.println("ROL");
    Serial.println(coms.getSlaveData());
    
    }
  }
  if (accel) {
	  Serial3.println("ACL"); //tell it the next line is Accelerometer
	  Serial3.print("Accel: X: ");
	  Serial3.print(getAccelX());
	  Serial3.print(" Y: ");
	  Serial3.print(getAccelY());
	  Serial3.print(" Z: ");
	  Serial3.print(getAccelZ());
	  Serial3.print("\nGyro: X: ");
	  Serial3.print(getGyroX());
	  Serial3.print(" Y: "); 
	  Serial3.print(getGyroY());
	  Serial3.print(" Z: ");
	  Serial3.print(getGyroZ());
	  Serial3.print("\nMag: X: ");
	  Serial3.print(getMagX());
	  Serial3.print(" Y: ");
	  Serial3.print(getMagY());
	  Serial3.print(" Z: ");
	  Serial3.print(getMagZ());
	  Serial3.println();
  }
  if (depth) {
	  Serial3.println("DPT"); //tell it the next line is Depth
    coms.sendSlaveCmd(GET_DEPT);
	  Serial3.print(coms.getSlaveData());
	  Serial3.println(" feet");
  }
}
void debugInput(Input i){
  //the following is for debugging, prints all input back out on the serial used for programming the arduino
  Serial.print("buttons: ");
  Serial.print(i.buttons2);
  Serial.print(" ");
  Serial.print(i.buttons1);
  Serial.print(" X1: ");
  Serial.print(i.primaryX);
  Serial.print(" Y1: ");
  Serial.print(i.primaryY);
  Serial.print(" X2: ");
  Serial.print(i.secondaryX);
  Serial.print(" Y2: ");
  Serial.print(i.secondaryY);
  Serial.print(" Trig: ");
  Serial.println(i.triggers);

}
void loop() { 
     if (Serial3.available()) {
		 //digitalWrite(13, HIGH);
        waitForStart();
        Input i = readBuffer();
        digitalWrite(serialWritePin, HIGH);
        if(debug==true)debugInput(i);
        writeToCommand(i); //this is where the code to write back to topside goes.
		Serial3.flush();
        delay(50);         //this delay allows for hardware serial to work with rs485
        digitalWrite(serialWritePin, LOW);
        
        processInput(i);//gives the inputs to the motors
    }
}


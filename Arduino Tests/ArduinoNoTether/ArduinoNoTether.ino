

#include <Servo.h>
#include "Math.h"
#include "Accelerometer.h"
#include "dataStruc.h"
//#include "QuadMotorShields.h"
#include "gyroAccelerometer.h"
#include <SoftwareSerial.h>
#include <SPI.h>
#include "pressure.h"
#include <Wire.h>
//#include "ComsMasterArd.h"
#include "VectorMotors.h"
#include "cameras.h"
//#include "currentSensing.h"

#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "imumaths.h"

//Declarations for temp readings
#include <OneWire.h>

#include "DallasTemperature.h"

// Data wire is plugged into pin 26 on the Arduino
#define ONE_WIRE_BUS 3

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

Adafruit_BNO055 bno = Adafruit_BNO055();



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

//CHANGE//ComsMasterArd coms;
//QuadMotorShields md;//Not being used anymore
bool reportPressure = false;
bool reportVoltage = false;
bool reportTemperature = true;
bool reportAccel = false;
bool reportDepth = false;
bool reportYPR = true;
bool reportAmperage = false;

bool debug = false;

int yaw;
int pch;
int rol;
float fast;

uint16_t amperages[8] = {0};
uint16_t* p_amperages;

imu::Vector<3> euler;



//SoftwareSerial Serial3(14, 15);
void setup() {

  //Serial3.begin(115200);   //the number in here is the baud rate, it is the communication speed, this must be matched in the python
  Serial.begin(9600);     //it does not seem to work at lower baud rates
  Serial.println("beginning setup");
  pinMode(serialWritePin, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(24, OUTPUT);

  digitalWrite(4, LOW); //Relay 4
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW); //Relay 1

  //delay(5000);
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  sensors.begin(); // IC Default 9 bit. If you have troubles consider upping it 12.
  //Ups the delay giving the IC more time to process the temperature measurement

  digitalWrite(serialWritePin, LOW);
  motorSetup();
  if (reportYPR) {
    bno.setExtCrystalUse(true);
  }
  Serial.println("setup complete");
}

//looks cleaner than the empty while loop being everywhere in the code
void wait() {
  //while(!Serial.available());
}

//detects the start signal without accidently overshooting it because of short circuting logic
//except for input such as SSTR, that will be skipped. There should be multiple characters to
//prevent random bytes getting past
void waitForStart() {
    while (true) {
      wait();
      if ('S' == Serial.read()) {
        wait();
        if ('T' == Serial.read()) {
          wait();
          if ('R' == Serial.read()) {
            break;
          }
        }
      }
    }
}

Input readBuffer() {
  Input input;
  wait();//makes sure that a byte of data is not missed
  input.buttons1 = Serial.read();
  wait();
  input.buttons2 = Serial.read();
  wait();
  input.primaryX = Serial.parseInt();
  Serial.read();
  wait();
  input.primaryY = Serial.parseInt();
  Serial.read();
  wait();
  input.secondaryX = Serial.parseInt();
  Serial3.read();
  wait();
  input.secondaryY = Serial.parseInt();
  Serial.read();
  wait();
  input.triggers = Serial.parseInt();
  Serial.read();
  return input;
}
void processInput(Input i) {

  if ((CHECK_BIT(i.buttons1, 3))) {
    digitalWrite(22, HIGH);
  }
  else {
    digitalWrite(22, LOW);
  }
  if ((CHECK_BIT(i.buttons1, 4))) {
    digitalWrite(24, HIGH);
  }
  else {
    digitalWrite(24, LOW);
  }

  setCameras(i.buttons1);
  setMotors(i.primaryX, i.primaryY, i.triggers, i.secondaryX, i.buttons1);
}

void writeToCommand(Input i) {
  Serial.println("STR");
  int lines = 0;
  if (reportPressure) lines += 2;
  if (reportVoltage) lines += 2;
  if (reportTemperature) lines += 2;
  if (reportDepth) lines += 2;
  if (reportAccel) lines += 4;
  if (reportYPR) lines += 6;
  String numberOfLines = String(lines);
  int counter = 0;
  while ((counter + numberOfLines.length()) != 3) {
    Serial.print("0");
    counter++;
  }
  Serial.println(numberOfLines); //print the number of lines of input the python program can read in three digits
  
//  if (reportPressure) {
//    Serial.println("PSR"); //tell it the next line is Pressure
//    //coms.sendSlaveCmd(GET_PRES);
//    //Serial3.print(coms.getSlaveData());
//    Serial.println(" mbars");
//  }

//  if (reportVoltage) {
//    Serial.println("VLT"); //tell it the next line is Power info
//    Serial.println( (((float)(analogRead(A1)) * (5.0 / 1023.0)) - 2.52)   / .066 );
//    //Serial3.println(" amps");
//  }

//  if (reportTemperature) {
//
//    Serial.println("TMP"); //tell it the next line is Temperature
//    //coms.sendSlaveCmd(GET_TEMP);
//    sensors.requestTemperatures(); // Send the command to get temperatures
//    fast = (sensors.getTempCByIndex(0));
//    Serial.println(fast);
//    //Serial3.println(((float)analogRead(A0)/(2.048))-273.15);
//    //Serial3.print(coms.getSlaveData());
//    //Serial3.println(" degrees C");
//  }
  if (reportYPR) {

    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    //coms.sendSlaveCmd(GET_YAW);
    Serial.println("YAW");
    Serial.println((int)(euler.x()));
    //Serial3.println(coms.getSlaveData());

    //CHANGE//coms.sendSlaveCmd(GET_PCH);
    Serial.println("PCH");
    Serial.println((int)(euler.y()));

    //CHANGE//coms.sendSlaveCmd(GET_ROL);
    Serial.println("ROL");
    Serial.println((int)(euler.z()));


    if (debug) {

      //CHANGE//coms.sendSlaveCmd(GET_YAW);
      //Serial.println("YAW");
      //yaw = coms.getSlaveData();
      /*
        Serial.println(yaw);
        //Serial3.println(coms.getSlaveData());

        coms.sendSlaveCmd(GET_PCH);
        Serial.println("PCH");
        Serial.println(coms.getSlaveData());

        coms.sendSlaveCmd(GET_ROL);
        Serial.println("ROL");
        Serial.println(coms.getSlaveData());
      */
    }
  }
  if (reportAccel) {
    Serial.println("ACL"); //tell it the next line is Accelerometer
    Serial.print("Accel: X: ");
    Serial.print(getAccelX());
    Serial.print(" Y: ");
    Serial.print(getAccelY());
    Serial.print(" Z: ");
    Serial.print(getAccelZ());
    Serial.print("\nGyro: X: ");
    Serial.print(getGyroX());
    Serial.print(" Y: ");
    Serial.print(getGyroY());
    Serial.print(" Z: ");
    Serial.print(getGyroZ());
    Serial.print("\nMag: X: ");
    Serial.print(getMagX());
    Serial.print(" Y: ");
    Serial.print(getMagY());
    Serial.print(" Z: ");
    Serial.print(getMagZ());
    Serial.println();
  }
  if (reportDepth) {
    Serial.println("DPT"); //tell it the next line is Depth
    //CHANGE//coms.sendSlaveCmd(GET_DEPT);
    //CHANGE//Serial3.print(coms.getSlaveData());
    Serial.println(" feet");
  }
}
void debugInput(Input i) {
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
  Serial.println("made it to loop");
  if (true //Serial.available()
     ) {
    //digitalWrite(13, HIGH);
    //waitForStart();
    Input i = readBuffer();
    // digitalWrite(serialWritePin, HIGH);
    if (debug == true)debugInput(i);
    writeToCommand(i); //this is where the code to write back to topside goes.
    // Serial.flush();
    sensors.requestTemperatures();
    delay(50);         //this delay allows for hardware serial to work with rs485
    //digitalWrite(serialWritePin, LOW);

    //processInput(i);//gives the inputs to the motors
  }
}


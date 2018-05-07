#include <SoftwareSerial.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>

#include "Settings.h"

#include "dataStruct.h"

//#include "QuadMotorShields.h"

#include "gyroAccelerometer.h"
#include "pressure.h"

#include "VectorMotors.h"
#include "cameras.h"
#include "toolMotor.h"

//#include "currentSensing.h"

#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "imumaths.h"
#include "Math.h"
#include "Accelerometer.h"

//#include "ComsMasterArd.h"

//Declarations for temp readings
#include <OneWire.h>

#include "DallasTemperature.h"

// Data wire is plugged into pin 26 on the Arduino
#define ONE_WIRE_BUS 3
//--End Pinout---------------------------------

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

Adafruit_BNO055 bno = Adafruit_BNO055();

//CHANGE//ComsMasterArd coms;
//QuadMotorShields md;//Not being used anymore

int yaw;
int pch;
int rol;
float fast;

//uint16_t amperages[8] = {0};
//uint16_t* p_amperages;

imu::Vector<3> euler;

void setup() {
  // Serial Comms
  Serial3.begin(SERIAL3_BAUD, SERIAL_8N1);   //the number in here is the baud rate, it is the communication speed, this must be matched in the python
  Serial.begin(SERIAL_BAUD);     //it does not seem to work at lower baud rates

  // Thrusters
  pinMode(MOTOR_A_PIN, OUTPUT); // motor PWM
  pinMode(MOTOR_B_PIN, OUTPUT); // motor PWM
  pinMode(MOTOR_C_PIN, OUTPUT); // motor PWM
  pinMode(MOTOR_D_PIN, OUTPUT); // motor PWM
  pinMode(MOTOR_E_PIN, OUTPUT); // motor PWM
  pinMode(MOTOR_F_PIN, OUTPUT); // motor PWM
  motorSetup();

  //Cameras
  pinMode(CAMERA_SERVO_PIN, OUTPUT);
  
  pinMode(CAMERA_ENABLE_PIN, OUTPUT);
  pinMode(CAMERA_SEL_PIN_C, OUTPUT);
  pinMode(CAMERA_SEL_PIN_B, OUTPUT);
  pinMode(CAMERA_SEL_PIN_A, OUTPUT);
  camera_setup();

  //Tool Motor
  pinMode(TOOL_MOTOR_PWM_PIN, OUTPUT);
  pinMode(TOOL_MOTOR_FWD_PIN, OUTPUT);
  pinMode(TOOL_MOTOR_BWD_PIN, OUTPUT);
  tool_motor_setup();

  // Accelerometer
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //while (1);
  }
  sensors.begin(); // IC Default 9 bit. If you have troubles consider upping it 12.
  //Ups the delay giving the IC more time to process the temperature measurement
  if (REPORT_YPR) //if using accel and orrientation,
  {
    if (!bno.begin())
    {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      //while (1); // skipping for now
    }
    bno.setExtCrystalUse(true);
  }

  if(DEBUG)
  {
  Serial.println("Setup complete");
  }
}

//looks cleaner than the empty while loop being everywhere in the code
void wait() {
  while (!(Serial3.available() > 0));
}

//detects the start signal without accidently overshooting it because of short circuting logic
//except for input such as SSTR, that will be skipped. There should be multiple characters to
//prevent random bytes getting past
void waitForStart() {
  return;  //short circuit
  while (true) {
    wait();
    if ('S' == Serial3.read()) {
      wait();
      if ('T' == Serial3.read()) {
        wait();
        if ('R' == Serial3.read()) {
          //break;
          return;
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

void processInput(Input i)
{
  setCameras(i.buttons1, i.secondaryY);
  //setMotors(X, Y, Z, R, buttons)
  setMotors(i.primaryX, i.primaryY, i.triggers, i.secondaryX, i.buttons1);
  set_tool_motor(i.buttons1);
}

void writeToCommand(Input i) {
  Serial3.print("STR");
  int lines = 0;
  if (REPORT_PRESSURE) lines += 2;
  if (REPORT_VOLTAGE) lines += 2;
  if (REPORT_TEMPERATURE) lines += 2;
  if (REPORT_DEPTH) lines += 2;
  if (REPORT_ACCEL) lines += 4;
  if (REPORT_YPR) lines += 6;
  String numberOfLines = String(lines);
  int counter = 0;
  while ((counter + numberOfLines.length()) != 3) // pad zeros in front
  {
    Serial3.print("0");
    counter++;
  }
  Serial3.print(numberOfLines); //print the number of lines of input the python program can read in three digits

  if (REPORT_PRESSURE) {
    Serial3.println("PSR"); //tell it the next line is Pressure
    //coms.sendSlaveCmd(GET_PRES);
    //Serial3.print(coms.getSlaveData());
    Serial3.println(" mbars");
  }
  if (REPORT_VOLTAGE) {
    Serial3.println("VLT"); //tell it the next line is Power info
    Serial3.println( (((float)(analogRead(A1)) * (5.0 / 1023.0)) - 2.52)   / .066 );
    //Serial3.println(" amps");
  }
  if (REPORT_TEMPERATURE) {

   // Serial3.println("TMP"); //tell it the next line is Temperature
    //coms.sendSlaveCmd(GET_TEMP);
//    sensors.requestTemperatures(); // Send the command to get temperatures
   // fast = (sensors.getTempCByIndex(0));
   // Serial3.println(fast);
    //Serial3.println(((float)analogRead(A0)/(2.048))-273.15);
    //Serial3.print(coms.getSlaveData());
    //Serial3.println(" degrees C");
  }
  if (REPORT_YPR) {

    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    int yaw = (int)(euler.x());
    int pitch = (int)(euler.y());
    int roll = (int)(euler.z());

    Serial3.println("YAW");
    Serial3.println(yaw);

    Serial3.println("PCH");
    Serial3.println(pitch);

    Serial3.println("ROL");
    Serial3.println(roll);


    if (DEBUG) {
      Serial.print("YAW: ");
      Serial.println(yaw);
    
      Serial.print("PCH: ");
      Serial.println(pitch);
    
      Serial.print("ROL: ");
      Serial.println(roll);

    }
  }
  if (REPORT_ACCEL) {
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
  if (REPORT_DEPTH) {
    // BROKEN
    //Serial3.println("DPT"); //tell it the next line is Depth
    //CHANGE//coms.sendSlaveCmd(GET_DEPT);
    //CHANGE//Serial3.print(coms.getSlaveData());
    //Serial3.println(" feet");
    //Serial3.println("
  }
}

void debugInput(Input i) {
  //the following is for debugging, prints all input back out on the serial used for programming the arduino
  Serial.print("Buttons2: ");
  Serial.print(i.buttons2);
  Serial.print(" Buttons1: ");
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

void loop()
{
  Serial.print("Starting loop code: serial3 avaiable:");
  Serial.println(Serial3.available());
  if (Serial3.available() > 0)
  {
    waitForStart();
    Input i = readBuffer();
    if (DEBUG)
    {
      Serial.println("debugging input");
      debugInput(i);
    }
    writeToCommand(i); //this is where the code to write back to topside goes.
    Serial3.flush();
    //sensors.requestTemperatures();
    delay(50);       

    processInput(i);//gives the inputs to the motors
  }
}


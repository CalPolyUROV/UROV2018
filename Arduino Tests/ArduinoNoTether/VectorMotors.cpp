#include "arduino.h"
//
#include <wire.h>
#define MOTORACCELERATIONMAX 80

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
//20 motor speed unit things per interval (maybe change to dv/dt later)

///////////connecting the ESC to the arduino (switch the pin to the one in use)
/*
 * connect three 24 gauge wires to the arduino (can power the arduino)
 * the yellow wire is the signal wire (connect to any pin on the arduino)
 * the red and black wires are power and ground, they can supple the arduino, but are not nessesary
 * now for the three wires connected to the motor
 * any combination works, if the motor turns the wrong way, switch two wires
 * test the direction without the propellers!
 */

/////////////////////////////////////////////////////////////////globals
#include <Servo.h>

//motor pins
int _m1 = 13;
int _m2 = 12;
int _m3 = 11;
int _m4 = 10;
int _m5 = 9;
int _m6 = 8;
//limiting variable
int currentMotor1speed = 0;
int currentMotor2speed = 0;
int currentMotor3speed = 0;
int currentMotor4speed = 0;
int currentZspeed = 0;
//
int motor1speedX; // initialize variables (used to calculate final motorspeed)
  int motor2speedX;
  int motor3speedX;
  int motor4speedX;
  
  int motor1speedY;
  int motor2speedY;
  int motor3speedY;
  int motor4speedY;
  
  int motor1speedR;
  int motor2speedR;
  int motor3speedR;
  int motor4speedR;

  int motor1speed; // (final motorspeeds)
  int motor2speed;
  int motor3speed;
  int motor4speed;


  int a, b;
  int A,B;
  //int motor1transspeed, motor2transspeed, motor3transspeed, motor4transspeed;

  

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;
Servo motor5;
Servo motor6;//the "servo" the pin will be connected to

int brownOutPrevent(int currentSpeed, int targetSpeed);

//////////////////////////////////////////////////////////////////////////attach ESCs to pins and set current to 0 Amps

void motorSetup() 
{
  motor1.attach(_m1); // make the pin act like a servo
  motor2.attach(_m2);
  motor3.attach(_m3);
  motor4.attach(_m4);
  motor5.attach(_m5);
  motor6.attach(_m6);
  motor1.writeMicroseconds(1500); // set the ESC to 0 Amps (1500us +-25us is the center)
  motor2.writeMicroseconds(1500);
  motor3.writeMicroseconds(1500);
  motor4.writeMicroseconds(1500);
  motor5.writeMicroseconds(1500);
  motor6.writeMicroseconds(1500);
  delay(100); // ensure that the signal was recieved
}

///////////////////////////////////////////////////////////////////////////how to write motorspeeds
// function that takes input from -400 to 400
////////how to use the motor
// servo.writeMicroseconds(number from 1100 to 1900)
// less than 1500 should be backward (limit 1100)
// more than 1500 should be forward  (limit 1900)
void motor_1(int mspeed) 
{
  int mspeed1 = map(mspeed,-400,400,1100,1900);
  motor1.writeMicroseconds(mspeed1);
}

void motor_2(int mspeed)
{
  int mspeed2 = map(mspeed,-400,400,1100,1900);
  motor2.writeMicroseconds(mspeed2);
}

void motor_3(int mspeed)
{
  int mspeed3 = map(mspeed,400,-400,1100,1900);
  motor3.writeMicroseconds(mspeed3);
}

void motor_4(int mspeed)
{
  int mspeed4 = map(mspeed,-400,400,1100,1900);
  motor4.writeMicroseconds(mspeed4);
}

void motor_5(int mspeed)
{
  int mspeed5 = map(mspeed,400,-400,1100,1900);
  //mspeed1 is reversed here
  motor5.writeMicroseconds(mspeed5);
}

void motor_6(int mspeed)
{
  int mspeed6 = map(mspeed,-400,400,1100,1900);
  motor6.writeMicroseconds(mspeed6);
}


///////////////////////////////////////////// allow rotation and planar movement simultaniously (takes x y z and r, then sets motorspeeds)
void setMotors(int X,int Y,int Z,int R,unsigned char buttons)
{
  /*
  a = (( (-1*X*1000/707) + (Y*1000/707) ) /2);
  b = (( (X*1000/707) + (Y*1000/707) ) /2);

  motor1speed = b;
  motor2speed = a;
  motor3speed = -a;
  motor4speed = -b;

  if(motor1speed<0)motor1speed*= 1.3; 
  if(motor2speed<0)motor2speed*= 1.3;
  if(motor3speed<0)motor3speed*= 1.3;
  if(motor4speed<0)motor4speed*= 1.3;
*/

  motor1speedY = Y; // get directions forward backward
  motor2speedY = Y;
  motor3speedY = -1*Y;
  motor4speedY = Y;
  
  motor1speedX = X; // get directions right left
  motor2speedX = -1*X;
  motor3speedX = X;
  motor4speedX = X;

  //motor1speedR = -1*R; // get directions turning
  //motor2speedR = -1*R;
  //motor3speedR = R;
  //motor4speedR = R;


  motor1speed = (motor1speedX + motor1speedY) / 2; // add and divide to get motor speeds (no rotation included yet)
  motor2speed = (motor2speedX + motor2speedY) / 2;
  motor3speed = (motor3speedX + motor3speedY) / 2;
  motor4speed = (motor4speedX + motor4speedY) / 2;

  motor1speed = motor1speed + R/2;
  motor2speed = motor2speed - R/2;
  motor3speed = motor3speed - R/2;
  motor4speed = motor4speed - R/2;

  motor1speed = constrain(motor1speed,-400,400);
  motor2speed = constrain(motor2speed,-400,400);
  motor3speed = constrain(motor3speed,-400,400);
  motor4speed = constrain(motor4speed,-400,400);

 /*
  motor1speed += motor1speedR;
  motor2speed += motor2speedR;
  motor3speed += motor3speedR;
  motor4speed += motor4speedR;
*/



  
  currentMotor1speed = brownOutPrevent(currentMotor1speed, motor1speed);
  currentMotor2speed = brownOutPrevent(currentMotor2speed, motor2speed);
  currentMotor3speed = brownOutPrevent(currentMotor3speed, motor3speed);
  currentMotor4speed = brownOutPrevent(currentMotor4speed, motor4speed);
  currentZspeed = brownOutPrevent(currentZspeed, Z);

//limiting code end

  motor_1(currentMotor1speed); // write motorspeeds
  motor_2(currentMotor2speed);
  motor_3(currentMotor3speed);
  motor_4(currentMotor4speed);
  motor_5(currentZspeed);
  if(CHECK_BIT(buttons, 1)){motor_6(currentZspeed);}
  else{motor_6(currentZspeed);}
  Serial.println(motor1speed);
  Serial.println(motor2speed);
  Serial.println(motor3speed);
  Serial.println(motor4speed);
  Serial.println(currentZspeed);
  
}

int brownOutPrevent(int currentSpeed, int targetSpeed){   //Comments use 20 for MOTORACCELERATIONMAX
  //adding change limiting code here
  if((targetSpeed - currentSpeed)> MOTORACCELERATIONMAX){   //If target is over 20 above, only increase by 20
    return currentSpeed + MOTORACCELERATIONMAX; 
  } else if((currentSpeed - targetSpeed)> MOTORACCELERATIONMAX){ //Else, If target is over 20 below, only decrease by 20
    return currentSpeed - MOTORACCELERATIONMAX;
  } 
  else {
    return targetSpeed; //Else, it is okay to set to target
  }
}




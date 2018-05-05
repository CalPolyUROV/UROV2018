

#include "Settings.h"

#include "toolMotor.h"

void setup() {
 
  Serial.begin(9600);
  
  //Tool Motor
  pinMode(TOOL_MOTOR_PWM_PIN, OUTPUT);
  pinMode(TOOL_MOTOR_FWD_PIN, OUTPUT);
  pinMode(TOOL_MOTOR_BWD_PIN, OUTPUT);
  tool_motor_setup();

}

void loop() {
 delay(5000);
 Serial.println("Sending command");
 set_tool_motor(63);
}

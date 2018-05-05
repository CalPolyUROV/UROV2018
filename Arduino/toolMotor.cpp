#include "arduino.h"
#include <Wire.h>
#include <Servo.h>

#include "toolMotor.h"
#include "Settings.h"

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

int motor_speed = 0;
bool motor_direction_fwd = true;

void tool_motor_setup()
{
  set_tool_motor(0);
}

void set_direction()
{
  if (motor_direction_fwd)
  {

    digitalWrite(TOOL_MOTOR_FWD_PIN, HIGH);
    digitalWrite(TOOL_MOTOR_BWD_PIN, LOW);
  }
  else
  {

    digitalWrite(TOOL_MOTOR_FWD_PIN, LOW);
    digitalWrite(TOOL_MOTOR_BWD_PIN, HIGH);
  }
}

void set_tool_motor(unsigned char buttons)
{
  if (CHECK_BIT(buttons, 3) /* && debounce */)
  {
    if (motor_speed != FULL_SPEED)
    {
      motor_speed = FULL_SPEED;
    }
    else {
      motor_speed = OFF_SPEED;
    }
  }
  write_motor();
}

void write_motor()
{
  analogWrite(TOOL_MOTOR_PWM_PIN, constrain(motor_speed, 0, 255));
  set_direction();
}


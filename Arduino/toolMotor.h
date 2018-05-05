#ifndef TOOLMOTOR_H
#define TOOLMOTOR_H

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

#define FULL_SPEED 255
#define OFF_SPEED 0

void tool_motor_setup();

void set_direction();
void set_tool_motor(unsigned char);
void write_motor();

#endif

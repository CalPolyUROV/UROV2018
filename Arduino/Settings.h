#ifndef _SETTINGS_h
#define _SETTINGS_h

//---------------FEATURES:--------------------
//-- set true or false to enable or disable --
//--------------------------------------------
#define REPORT_PRESSURE false
#define REPORT_VOLTAGE false
#define REPORT_TEMPERATURE false
#define REPORT_ACCEL true
#define REPORT_DEPTH false
#define REPORT_YPR true
#define REPORT_AMPERAGE false

#define DEBUG false
//-- end features-----------------------------

//--Pinouts:---------------------------------
//Thrusters
#define MOTOR_A_PIN 8
#define MOTOR_B_PIN 9
#define MOTOR_C_PIN 12
#define MOTOR_D_PIN 13
#define MOTOR_E_PIN 10
#define MOTOR_F_PIN 11
// Cameras
#define CAMERA_SERVO_PIN 2
#define CAMERA_ENABLE_PIN 44
#define CAMERA_SEL_PIN_C 45
#define CAMERA_SEL_PIN_B 46
#define CAMERA_SEL_PIN_A 47

//--Settings--------------------------------
//Comms Baudrates
#define SERIAL_BAUD 19200
#define SERIAL3_BAUD 9600
//Motors
#define MOTOR_A_DIR 1
#define MOTOR_B_DIR 1
#define MOTOR_C_DIR -1
#define MOTOR_D_DIR 1
#define MOTOR_E_DIR -1
#define MOTOR_F_DIR 1

//#define MOTOR_INPUT_MIN 15 // Not yet implemented
#define MOTOR_JERK_MAX 50
/* crazy people use 80, og version used 20 */

//Cameras
#define NUM_CAMERAS 2
#define CAMERA_ENABLE true
#define CAMERA_ANGLE_DELTA 15

#endif

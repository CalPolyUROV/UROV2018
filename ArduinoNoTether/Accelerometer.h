// Accelerometer.h

#ifndef _ACCELEROMETER_h
#define _ACCELEROMETER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
// Accelerometer
// "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define ACCEL_X_MIN ((float) -250)
#define ACCEL_X_MAX ((float) 250)
#define ACCEL_Y_MIN ((float) -250)
#define ACCEL_Y_MAX ((float) 250)
#define ACCEL_Z_MIN ((float) -250)
#define ACCEL_Z_MAX ((float) 250)

// Magnetometer (standard calibration mode)
// "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define MAGN_X_MIN ((float) -600)
#define MAGN_X_MAX ((float) 600)
#define MAGN_Y_MIN ((float) -600)
#define MAGN_Y_MAX ((float) 600)
#define MAGN_Z_MIN ((float) -600)
#define MAGN_Z_MAX ((float) 600)

// Gyroscope
// "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
#define GYRO_AVERAGE_OFFSET_X ((float) 0.0)
#define GYRO_AVERAGE_OFFSET_Y ((float) 0.0)
#define GYRO_AVERAGE_OFFSET_Z ((float) 0.0)

#define OUTPUT__HAS_RN_BLUETOOTH false

#include <Wire.h>

// Sensor calibration scale and offset values
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))


// Gain for gyroscope (ITG-3200)
#define GYRO_GAIN 0.06957 // Same gain on all axes
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN)) // Calculate the scaled gyro readings in radians per second

// DCM parameters
#define Kp_ROLLPITCH 0.02f
#define Ki_ROLLPITCH 0.00002f
#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f

// Stuff
#define STATUS_LED_PIN 13  // Pin number of status LED
#define GRAVITY 256.0f // "1G reference" used for DCM filter and accelerometer calibration
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

#define OUTPUT__DATA_INTERVAL 20  // in milliseconds

class Accelerometer
{
 protected:
	 float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
	 float accel_min[3];
	 float accel_max[3];

	 float magnetom[3];
	 float magnetom_min[3];
	 float magnetom_max[3];
	 float magnetom_tmp[3];

	 float gyro[3];
	 float gyro_average[3];
	 int gyro_num_samples = 0;

	 float MAG_Heading;
	 float Accel_Vector[3] = { 0, 0, 0 }; // Store the acceleration in a vector
	 float Gyro_Vector[3] = { 0, 0, 0 }; // Store the gyros turn rate in a vector
	 float Omega_Vector[3] = { 0, 0, 0 }; // Corrected Gyro_Vector data
	 float Omega_P[3] = { 0, 0, 0 }; // Omega Proportional correction
	 float Omega_I[3] = { 0, 0, 0 }; // Omega Integrator
	 float Omega[3] = { 0, 0, 0 };
	 float errorRollPitch[3] = { 0, 0, 0 };
	 float errorYaw[3] = { 0, 0, 0 };
	 float DCM_Matrix[3][3] = { { 1, 0, 0 },{ 0, 1, 0 },{ 0, 0, 1 } };
	 float Update_Matrix[3][3] = { { 0, 1, 2 },{ 3, 4, 5 },{ 6, 7, 8 } };
	 float Temporary_Matrix[3][3] = { { 0, 0, 0 },{ 0, 0, 0 },{ 0, 0, 0 } };

	 // Euler angles
	 float yaw;
	 float pitch;
	 float roll;

	 // DCM timing in the main loop
	 unsigned long timestamp;
	 unsigned long timestamp_old;
	 float G_Dt; // Integration time for DCM algorithm

   void Normalize();
   void Drift_correction();
   void Matrix_update();
   void Euler_angles();
   void Compass_Heading();
   void Read_Gyro(); // Read gyroscope
   void Read_Accel(); // Read accelerometer
   void Read_Magn(); // Read magnetometer 
   void I2C_Init();
   void read_sensors();
   void Accel_Init();
   void Gyro_Init();
 public:
	 Accelerometer();
	 void Update();
	 int getPitch();
	 int getYaw();
	 int getRoll();
};

#endif


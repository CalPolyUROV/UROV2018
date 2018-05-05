// Math.h

#ifndef _MATH_h
#define _MATH_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
#include <math.h>
float Vector_Dot_Product(const float v1[3], const float v2[3]);
void Vector_Cross_Product(float out[3], const float v1[3], const float v2[3]);
void Vector_Scale(float out[3], const float v[3], float scale);
void Vector_Add(float out[3], const float v1[3], const float v2[3]);
void Matrix_Multiply(const float a[3][3], const float b[3][3], float out[3][3]);
void Matrix_Vector_Multiply(const float a[3][3], const float b[3], float out[3]);
void init_rotation_matrix(float m[3][3], float yaw, float pitch, float roll);

#endif


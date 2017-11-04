cmake_minimum_required(VERSION 3.9)
project(Arduino)

set(CMAKE_CXX_STANDARD 11)

add_executable(Arduino
        Accelerometer.cpp
        Accelerometer.h
        Adafruit_BNO055.cpp
        Adafruit_BNO055.h
        Adafruit_Sensor.h
        cameras.cpp
        cameras.h
        dataStruc.h
        gyroAccelerometer.cpp
        gyroAccelerometer.h
        Math.cpp
        Math.h
        pressure.cpp
        pressure.h
        VectorMotors.cpp
        VectorMotors.h)

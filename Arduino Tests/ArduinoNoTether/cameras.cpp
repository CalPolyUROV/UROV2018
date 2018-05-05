#include "arduino.h"
#include <Wire.h>
#include <Servo.h>
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
#define NUM_CAMERAS 2

//camera pins
int _c1 = 22;
int _c2 = 24;
int _c3 = 26;
int currentCamera = 0;
bool debounce = 0;

void setCameras(unsigned char buttons)
{
    
    if(CHECK_BIT(buttons, 2)&&debounce){
        debounce = 0;
        //digitalWrite(13, HIGH);
        currentCamera++;
        if(currentCamera==NUM_CAMERAS) {currentCamera = 0;}
        
        digitalWrite(22,!(CHECK_BIT(currentCamera, 0)));
        digitalWrite(24,CHECK_BIT(currentCamera, 1));
        digitalWrite(26,CHECK_BIT(currentCamera, 2));
       
    }
    else {
      debounce = 1;
        //digitalWrite(13, LOW);
    }
    
    
    
}

#include "arduino.h"
#include <Wire.h>
#include <Servo.h>
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
#define NUM_CAMERAS 2

//camera pins
int camera_enable = 44;
int _c1 = 45;
int _c2 = 46;
int _c3 = 47;
int currentCamera = 0;
bool debounce = 0;

void enableCameras(bool enable)
{
  //camera multiplexor enable is active low
  if (enable) {
    digitalWrite(camera_enable, LOW);
  }
  else {
    digitalWrite(camera_enable, HIGH);
  }
}
void setCameras(unsigned char buttons)
{

  if (CHECK_BIT(buttons, 2) && debounce) {
    debounce = 0;
    //digitalWrite(13, HIGH);
    currentCamera++;
    if (currentCamera == NUM_CAMERAS) {
      currentCamera = 0;
    }
    digitalWrite(_c1, !(CHECK_BIT(currentCamera, 0)));
    digitalWrite(_c2, CHECK_BIT(currentCamera, 1));
    digitalWrite(_c3, CHECK_BIT(currentCamera, 2));
  }
  else {
    debounce = 1;
    //digitalWrite(13, LOW);
  }



}

#include "arduino.h"
#include <Wire.h>
#include <Servo.h>

#include "cameras.h"
#include "Settings.h"

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

//camera pins

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
    currentCamera++;
    if (currentCamera == NUM_CAMERAS) {
      currentCamera = 0;
    }
    digitalWrite(CAMERA_SEL_PIN_A, !(CHECK_BIT(currentCamera, 0)));
    digitalWrite(CAMERA_SEL_PIN_B, CHECK_BIT(currentCamera, 1));
    digitalWrite(CAMERA_SEL_PIN_C, CHECK_BIT(currentCamera, 2));
  }
  else
  {
    debounce = 1;
  }



}

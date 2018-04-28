#ifndef CAMERAS_H
#define CAMERAS_H

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

void camera_setup();
void enable_cameras(bool);
void setCameras(unsigned char, int Y);
void write_cameras();

#endif

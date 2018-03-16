#ifndef CAMERAS_H
#define CAMERAS_H

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

void enableCameras(bool);
void setCameras(unsigned char);

#endif

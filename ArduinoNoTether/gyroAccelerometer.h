#ifndef GYROACCEL_H
#define GYROACCEL_H

void init_adxl345();
void init_hmc5843();
void init_itg3200();
void read_adxl345();
void read_hmc5843();
void read_itg3200();
void accelSetup();
void updateAccelRaw();
int getAccelX();

int getAccelY();

int getAccelZ();

int getGyroX();

int getGyroY();

int getGyroZ();

int getMagX();

int getMagY();

int getMagZ();
#endif

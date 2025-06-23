#ifndef SENSORS_H
#define SENSORS_H

#include "data_types.h"

bool initSensors();
float readMQ135();
bool readDHT11(float* temperature, float* humidity);
bool validateReadings(sensor_data_t* data);

#endif

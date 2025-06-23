#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "data_types.h"

int calculateAQI(float gas_ppm, float temperature, float humidity);
//float convertPPM(int analogValue);
bool validateReading(float value, float min_val, float max_val);

#endif
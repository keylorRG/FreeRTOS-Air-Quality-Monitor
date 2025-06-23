#ifndef DISPLAY_H
#define DISPLAY_H

#include "data_types.h"

bool initLCD();
void updateDisplay(display_data_t* data);
void showSplashScreen();
void formatValues(display_data_t* data, char* line1, char* line2);

#endif

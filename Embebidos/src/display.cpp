#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "display.h"
#include "config.h"

LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16x2 display

bool initLCD() {
    lcd.init();
    lcd.backlight();
    showSplashScreen();
    delay(2000);
    lcd.clear();
    return true;
}

void updateDisplay(display_data_t* data) {
    char line1[17], line2[17];
    formatValues(data, line1, line2);
    
    lcd.setCursor(0, 0);
    lcd.print(line1);
    lcd.setCursor(0, 1);
    lcd.print(line2);
}

void showSplashScreen() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Calidad Aire");
    lcd.setCursor(0, 1);
    lcd.print("Iniciando...");
}

void formatValues(display_data_t* data, char* line1, char* line2) {
    // Line 1: Gas PPM and status
    const char* status_text[] = {"BUENO", "MEDIO", "MALO", "ERROR"};
    snprintf(line1, 17, "Gas:%3.0fppm %s", data->gas_ppm, status_text[data->air_quality_status]);
    
    // Line 2: Temperature and humidity
    snprintf(line2, 17, "T:%2.1fC H:%2.0f%%", data->temperature, data->humidity);
}
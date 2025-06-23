#include <Arduino.h>
#include "alerts.h"
#include "config.h"

void initAlerts() {
    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_YELLOW_PIN, OUTPUT);
    
    // Turn off all LEDs
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_YELLOW_PIN, LOW);
}

void controlLEDs(int air_quality) {
    // Turn off all LEDs first
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_YELLOW_PIN, LOW);
    
    // Turn on appropriate LED based on air quality
    switch(air_quality) {
        case AQ_GOOD:
            digitalWrite(LED_GREEN_PIN, HIGH);
            break;
        case AQ_MODERATE:
            digitalWrite(LED_YELLOW_PIN, HIGH);
            break;
        case AQ_BAD:
        case AQ_ERROR:
            digitalWrite(LED_RED_PIN, HIGH);
            break;
    }
}



void setAlertState(int air_quality) {
    controlLEDs(air_quality);
    //aquí pudo estar el buzzer(aie_quality)
    
}
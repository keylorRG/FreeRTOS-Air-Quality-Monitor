#include <Arduino.h>
#include "algorithms.h"
#include "config.h"

int calculateAQI(float gas_ppm, float temperature, float humidity) {
    // Evaluar gases
    int gas_status = AQ_GOOD;
    if (gas_ppm > 400) gas_status = AQ_BAD;
    else if (gas_ppm > 250) gas_status = AQ_MODERATE;
    
    // Evaluar humedad  
    int hum_status = AQ_GOOD;
    if (humidity > 70 || humidity < 30) hum_status = AQ_MODERATE;
    if (humidity > 85 || humidity < 20) hum_status = AQ_BAD;
    
    // Evaluar temperatura
    int temp_status = AQ_GOOD;
    if (temperature > 30 || temperature < 18) temp_status = AQ_MODERATE;
    if (temperature > 35 || temperature < 15) temp_status = AQ_BAD;
    
    // Retornar el PEOR estado
    return max({gas_status, hum_status, temp_status});
}

// float convertPPM(int analogValue) {
//     // Convert 12-bit ADC reading to voltage
//     float voltage = (analogValue / 4095.0) * 3.3;
    
//     // Conversión a PPM
//     // Calibración apropiada
//     float ppm = voltage * 300; // Approximate conversion factor
    
//     return ppm;
// }

bool validateReading(float value, float min_val, float max_val) {
    return (value >= min_val && value <= max_val && !isnan(value));
}

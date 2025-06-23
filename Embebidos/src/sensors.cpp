#include <Arduino.h>
#include <DHT.h>
#include "sensors.h"
#include "config.h"

DHT dht(DHT11_PIN, DHT11);

bool initSensors() {
    // Initialize DHT11
    dht.begin();
    
    // Initialize MQ135 ADC
    analogReadResolution(12); // 12-bit resolution (0-4095)
    
    Serial.println("Sensors initialized");
    return true;
}

 float readMQ135() {
    int analogValue = analogRead(MQ135_PIN);

     float voltage = (analogValue / 4095.0) * 3.3 * 1.5; // Convert ADC value to voltage (assuming 3.3V reference)

     float ppm = (voltage * 1000) / 3.3; // Basic linear conversion
    
     return ppm;
 }

bool readDHT11(float* temperature, float* humidity) {
    // DHT11 necesita más tiempo entre lecturas
    static unsigned long lastRead = 0;
    if (millis() - lastRead < 2000) {  // Mínimo 2 segundos
        return false; // Muy pronto para leer
    }
    
    *temperature = dht.readTemperature();
    *humidity = dht.readHumidity();
    
    lastRead = millis();
    
    // DHT11 a veces necesita intentos múltiples
    if (isnan(*temperature) || isnan(*humidity)) {
        delay(100);
        *temperature = dht.readTemperature();
        *humidity = dht.readHumidity();
        
        if (isnan(*temperature) || isnan(*humidity)) {
            Serial.println("DHT11 read error");
            return false;
        }
    }
    
    return true;
}

bool validateReadings(sensor_data_t* data) {
    // DHT11 rangos más limitados
    if (data->temperature < 0 || data->temperature > 50) {  // DHT11 típico: 0-50°C
        return false;
    }
    
    if (data->humidity < 20 || data->humidity > 95) {  // DHT11 típico: 20-95%
        return false;
    }
    
    if (data->gas_ppm < 0 || data->gas_ppm > 1000) {
        return false;
    }
    
    return true;
}
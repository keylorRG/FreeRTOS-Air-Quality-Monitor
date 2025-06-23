

//============================================================================
// main.cpp
//============================================================================
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include "config.h"
#include "data_types.h"
#include "sensors.h"
#include "display.h"
#include "alerts.h"
#include "algorithms.h"

// Global Variables
QueueHandle_t sensorQueue;
QueueHandle_t displayQueue;
QueueHandle_t alertQueue;
SemaphoreHandle_t i2cMutex;
system_state_t systemState = STATE_INIT;

// Task Declarations
//void TaskSensoreo(void *parameter);
void TaskMQ135(void *parameter);
void TaskDHT11(void *parameter);
void TaskProcesamiento(void *parameter);
void TaskDisplay(void *parameter);
void TaskAlertas(void *parameter);

// ====== CONTROL DE DEBUG ======
#define ENABLE_TASK_SENSOR_MQ      true  
#define ENABLE_TASK_SENSOR_DHT11   true  
#define ENABLE_TASK_PROCESS        true   
#define ENABLE_TASK_DISPLAY        true   // ← HABILITAR LCD
#define ENABLE_TASK_ALERTS         true

void setup() {
    Serial.begin(115200);
    Serial.println("==================================================");
    Serial.println("Monitor Calidad del Aire");
    Serial.println("==================================================");
    
    // Initialize hardware
    if (!initSensors()) {
        Serial.println("Error: Fallo al inicializar sensores");
        systemState = STATE_ERROR;
        return;
    }
    Serial.println("Sensores inicializados");
    
    if (!initLCD()) {
        Serial.println("Error: Fallo al inicializar LCD");
        systemState = STATE_ERROR;
        return;
    }
    Serial.println("LCD inicializado");
    
    initAlerts();
    Serial.println("Sistema de alertas inicializado");
    
    // Crear colas
    // xQueueCreate (tamaño de la cola, tipo de datos)
    sensorQueue = xQueueCreate(SENSOR_QUEUE_SIZE, sizeof(sensor_data_t));
    displayQueue = xQueueCreate(DISPLAY_QUEUE_SIZE, sizeof(display_data_t));
    alertQueue = xQueueCreate(ALERT_QUEUE_SIZE, sizeof(int));
    i2cMutex = xSemaphoreCreateMutex(); //semaforo mutex 
    
    if (sensorQueue == NULL || displayQueue == NULL || alertQueue == NULL || i2cMutex == NULL) {
        Serial.println("Error: Fallo al crear colas/semaforos");
        systemState = STATE_ERROR;
        return;
    }
    Serial.println("Colas y semáforos creados");
    
    // Create tasks
    if (ENABLE_TASK_SENSOR_MQ) {
        xTaskCreatePinnedToCore(TaskMQ135, "TaskSensoreo_MQ", 2048, NULL, TASK_SENSOR_PRIORITY, NULL, 1);
        Serial.println("TaskSensoreo_MQ creada");
    }

        if (ENABLE_TASK_SENSOR_DHT11) {
        xTaskCreatePinnedToCore(TaskDHT11, "TaskSensoreo_DHT11", 2048, NULL, TASK_SENSOR_PRIORITY, NULL, 1);
        Serial.println("TaskSensoreo_DHT11 creada");
    }
    
    if (ENABLE_TASK_PROCESS) {
        xTaskCreatePinnedToCore(TaskProcesamiento, "TaskProcesamiento", 4096, NULL, TASK_PROCESS_PRIORITY, NULL, 1);
        Serial.println("TaskProcesamiento creada");
    }
    
    if (ENABLE_TASK_DISPLAY) {
        xTaskCreatePinnedToCore(TaskDisplay, "TaskDisplay", 3072, NULL, TASK_DISPLAY_PRIORITY, NULL, 0);
        Serial.println("TaskDisplay creada");
    } else {
        Serial.println("TaskDisplay deshabilitada");
    }
    
    if (ENABLE_TASK_ALERTS) {
        xTaskCreatePinnedToCore(TaskAlertas, "TaskAlertas", 2048, NULL, TASK_ALERT_PRIORITY, NULL, 0);
        Serial.println("TaskAlertas creada");
    }
    
    systemState = STATE_NORMAL;
    Serial.printf("Sistema inicializado | Heap libre: %d bytes\n", ESP.getFreeHeap());
    Serial.println("==================================================");
    Serial.println("Sistema en funcionamiento");
    Serial.println("Leyenda: VERDE=BUENO | AZUL=MEDIO | ROJO=MALO");
    Serial.println("===================Taller Embebidos======================");
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}

// void TaskSensoreo(void *parameter) {
//     sensor_data_t sensorData;
//     TickType_t xLastWakeTime = xTaskGetTickCount();
//     const TickType_t xPeriod = pdMS_TO_TICKS(10000); // 10 segundos para valores reales
//     int readCount = 0;
    
//     Serial.println("TaskSensoreo: INICIADA");
//     Serial.println("Usando valores reales de sensores");
    
//     for (;;) {
//         readCount++;
//         Serial.printf("=== Lectura #%d ===\n", readCount);
        
//         // Read MQ135 - VALORES REALES
//         sensorData.gas_ppm = readMQ135();
//         Serial.printf("MQ135: %.1f ppm\n", sensorData.gas_ppm);
        
//         // Read DHT11
//         delay(100); // DHT11 necesita tiempo
//         if (!readDHT11(&sensorData.temperature, &sensorData.humidity)) {
//             Serial.println("ATENCION: DHT11 fallo de lectura ");
//             sensorData.temperature = -999;
//             sensorData.humidity = -999;
//         } else {
//             Serial.printf("DHT11: %.1f°C, %.1f%%\n", sensorData.temperature, sensorData.humidity);
//         }
        
//         sensorData.timestamp = millis();
        
//         // Validate readings
//         if (validateReadings(&sensorData)) {
//             if (xQueueSend(sensorQueue, &sensorData, pdMS_TO_TICKS(100)) != pdTRUE) {
//                 Serial.println("ATENCIÓN: cola de sensor lleno");
//             } else {
//                 Serial.println("Datos enviados correctamente");
//             }
//         } else {
//             Serial.println("Lecturas inválidas - no enviadas");
//         }
        
//         vTaskDelayUntil(&xLastWakeTime, xPeriod);
//     }
// }
void TaskMQ135(void *parameter) {
    sensor_data_t mqData;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(5000); // 5 segundos
    
    for (;;) {
        mqData.gas_ppm = readMQ135();
        mqData.sensor_type = 0; // MQ135
        mqData.timestamp = millis();
        
        xQueueSend(sensorQueue, &mqData, pdMS_TO_TICKS(100));
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}
void TaskDHT11(void *parameter) {
    sensor_data_t dhtData;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(3000); //cada 3 segundos
    
    for (;;) {
        // SIEMPRE envía datos, exitosos o de error
        if (readDHT11(&dhtData.temperature, &dhtData.humidity)) {
            // Lectura exitosa
            Serial.println("DHT11: Lectura exitosa");
        } else {
            // Lectura falló - enviar valores de error
            dhtData.temperature = -999.0;
            dhtData.humidity = -999.0;
            Serial.println("DHT11: Lectura falló");
        }
        
        dhtData.sensor_type = 1; // DHT11
        dhtData.timestamp = millis();
        
        // SIEMPRE envía, exitoso o error
        xQueueSend(sensorQueue, &dhtData, pdMS_TO_TICKS(100));
        
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

// void TaskProcesamiento(void *parameter) {
//     sensor_data_t sensorData;
//     display_data_t displayData;
//     int alertLevel;
//     int processCount = 0;
    
//     Serial.println("TaskProcesamiento: INICIADA ");
    
//     for (;;) {
//         if (xQueueReceive(sensorQueue, &sensorData, portMAX_DELAY) == pdTRUE) {
//             processCount++;
//             Serial.printf("--- Procesando datos #%d ---\n", processCount);
            
//             // Process the data
//             displayData.gas_ppm = sensorData.gas_ppm;
//             displayData.temperature = sensorData.temperature;
//             displayData.humidity = sensorData.humidity;
            
//             // Calculate air quality index
//             if (sensorData.temperature == -999 || sensorData.humidity == -999) {
//                 displayData.air_quality_status = AQ_ERROR;
//                 Serial.println("Estado: ERROR (sensor DHT11 falló)");
//             } else {
//                 displayData.air_quality_status = calculateAQI(sensorData.gas_ppm, 
//                                               sensorData.temperature, 
//                                               sensorData.humidity);
//                 const char* status_names[] = {"BUENO", "MEDIO", "MALO", "ERROR"};
//                 Serial.printf("Estado: %s\n", status_names[displayData.air_quality_status]);
//             }
            
//             // Send to display task
//             if (ENABLE_TASK_DISPLAY) {
//                 if (xQueueOverwrite(displayQueue, &displayData) != pdTRUE) {
//                     Serial.println("Error: No se pudo enviar a la cola del display");
//                 } else {
//                     Serial.println("Datos enviados a cola display");
//                 }
//             }
            
//             // Send to alerts task
//             if (ENABLE_TASK_ALERTS) {
//                 alertLevel = displayData.air_quality_status;
//                 if (xQueueOverwrite(alertQueue, &alertLevel) != pdTRUE) {
//                     Serial.println("Error: No se pudo enviar a cola alertas");
//                 }
//             }
            
//             // Resumen final
//             Serial.printf("Gas: %.1f ppm | Temp: %.1f°C | Hum: %.1f%% | Status: %d\n",
//                          displayData.gas_ppm, displayData.temperature, 
//                          displayData.humidity, displayData.air_quality_status);
//         }
//     }
// }
void TaskProcesamiento(void *parameter) {
    sensor_data_t sensorData;
    static float last_gas = 0.0;
    static float last_temp = 25.0;    // Valor inicial válido
    static float last_hum = 50.0;     // Valor inicial válido
    static bool dht_error = false;    // Flag de error DHT
    
    for (;;) {
        if (xQueueReceive(sensorQueue, &sensorData, portMAX_DELAY) == pdTRUE) {
            
            if (sensorData.sensor_type == 0) {
                // Datos del MQ135
                last_gas = sensorData.gas_ppm;
                Serial.printf("MQ135: %.1f ppm\n", last_gas);
                
            } else if (sensorData.sensor_type == 1) {
                // Datos del DHT11 - VERIFICAR ERRORES
                if (sensorData.temperature == -999 || sensorData.humidity == -999) {
                    dht_error = true;
                    Serial.println("DHT11: Sensor en error - usando últimos valores válidos");
                } else {
                    // Actualizar con valores nuevos válidos
                    last_temp = sensorData.temperature;
                    last_hum = sensorData.humidity;
                    dht_error = false;
                    Serial.printf("DHT11: %.1f°C, %.1f%%\n", last_temp, last_hum);
                }
            }
            
            // Calcular estado considerando errores
            display_data_t displayData;
            displayData.gas_ppm = last_gas;
            displayData.temperature = last_temp;
            displayData.humidity = last_hum;
            
            if (dht_error) {
                displayData.air_quality_status = AQ_ERROR;
                Serial.println("Estado: ERROR (DHT11 falló)");
            } else {
                displayData.air_quality_status = calculateAQI(last_gas, last_temp, last_hum);
                const char* status_names[] = {"BUENO", "MEDIO", "MALO", "ERROR"};
                Serial.printf("✓ Estado: %s\n", status_names[displayData.air_quality_status]);
            }
            
            // Enviar resultados
            xQueueOverwrite(displayQueue, &displayData);
            int alertLevel = displayData.air_quality_status;
            xQueueOverwrite(alertQueue, &alertLevel);
        }
    }
}

void TaskDisplay(void *parameter) {
    display_data_t displayData;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(2000); // 2 segundos para update LCD
    int displayCount = 0;
    
    Serial.println("TaskDisplay: INICIADA");
    
    // Test inicial del LCD usando las funciones de display.h
    Serial.println("TaskDisplay: Probando comunicación I2C...");
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        Serial.println("TaskDisplay: Mutex I2C tomado");
        
        // Test básico usando showSplashScreen que ya funciona
        Serial.println("TaskDisplay: Mostrando mensaje de test...");
        showSplashScreen(); // Calidad aire... (mensaje)
        delay(2000);
        
        xSemaphoreGive(i2cMutex);
        Serial.println("TaskDisplay: Test inicial completado, mutex liberado");
    } else {
        Serial.println("TaskDisplay: ERROR - No se pudo tomar mutex inicial");
    }
    
    for (;;) {
        if (xQueueReceive(displayQueue, &displayData, 0) == pdTRUE) {
            displayCount++;
            Serial.printf("TaskDisplay: === Actualizando LCD #%d ===\n", displayCount);
            Serial.printf("TaskDisplay: Gas=%.1f, Temp=%.1f, Hum=%.1f, Status=%d\n",
                         displayData.gas_ppm, displayData.temperature, 
                         displayData.humidity, displayData.air_quality_status);
            
            // Stack check
            UBaseType_t stackRemaining = uxTaskGetStackHighWaterMark(NULL);
            
            
            if (stackRemaining < 200) {
                Serial.println("TaskDisplay: ADVERTENCIA - Stack bajo!");
            }
            
            Serial.println("TaskDisplay: Intentando tomar mutex I2C...");
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
                Serial.println("TaskDisplay: Mutex tomado, actualizando LCD...");
                
                try {
                    Serial.println("TaskDisplay: Llamando updateDisplay()...");
                    
                    // Usar la función updateDisplay() del módulo display.cpp
                    updateDisplay(&displayData);
                    
                    Serial.println("TaskDisplay: updateDisplay() completado exitosamente");
                    
                } catch (...) {
                    Serial.println("TaskDisplay: EXCEPCIÓN durante updateDisplay()");
                }
                
                Serial.println("TaskDisplay: Liberando mutex...");
                xSemaphoreGive(i2cMutex);
                Serial.println("TaskDisplay: Mutex liberado");
                
            } else {
                Serial.println("TaskDisplay: ERROR - Timeout tomando mutex I2C");
            }
            
            Serial.printf("TaskDisplay: === Actualización #%d completada ===\n", displayCount);
            
        } else {
            // No hay datos nuevos - esto es normal
            if (displayCount == 0) {
                Serial.println("TaskDisplay: Esperando primeros datos...");
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

void TaskAlertas(void *parameter) {
    int alertLevel;
    int alertCount = 0;
    
    Serial.println("TaskAlertas: INICIADA");
    
    for (;;) {
        if (xQueueReceive(alertQueue, &alertLevel, portMAX_DELAY) == pdTRUE) {
            alertCount++;
            
            // Validar valor de entrada
            if (alertLevel < 0 || alertLevel > 3) {
                Serial.printf("TaskAlertas: Valor inválido: %d\n", alertLevel);
                continue;
            }
            
            // Control de LEDs - directamente sin funciones externas
            digitalWrite(LED_RED_PIN, LOW);
            digitalWrite(LED_GREEN_PIN, LOW);
            digitalWrite(LED_YELLOW_PIN, LOW);
            
            switch(alertLevel) {
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
            
            // Logging conciso
            const char* alert_names[] = {"BUENO", "MODERADO", "MALO", "ERROR"};
            const char* led_names[] = {"VERDE", "AZUL", "ROJO", "ROJO"};
            
            Serial.printf("Alerta #%d: %s → %s\n", 
                         alertCount, alert_names[alertLevel], led_names[alertLevel]);
            
            // Buzzer está deshabilitado
            if (alertLevel == AQ_BAD) {
                Serial.println("BUZZER: Activaría (deshabilitado)");
            }
        }
    }
}
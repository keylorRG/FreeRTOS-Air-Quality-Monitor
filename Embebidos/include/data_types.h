#ifndef DATA_TYPES_H
#define DATA_TYPES_H

typedef struct {
    float gas_ppm;
    float temperature;
    float humidity;
    uint32_t timestamp;
    uint8_t sensor_type;  // 0=MQ135, 1=DHT11 (alternativa es usar la estructura sin esta ultima y usar dos colas separas)
} sensor_data_t;

typedef struct {
    float gas_ppm;
    float temperature;
    float humidity;
    int air_quality_status;
} display_data_t;

typedef enum {
    STATE_INIT,
    STATE_NORMAL,
    STATE_ALERT,
    STATE_ERROR
} system_state_t;

#endif
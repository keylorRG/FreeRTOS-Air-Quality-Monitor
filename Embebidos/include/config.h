#ifndef CONFIG_H
#define CONFIG_H

// Pin Definitions
//#define MQ135_PIN       36  // VP pin (ADC1_CH0) 
#define MQ135_PIN       4   // GPIO4 - digital input
#define DHT11_PIN       2   // GPIO2  REVISAR

#define LCD_RST_PIN     4   // GPIO4
#define LCD_SDA_PIN     21  // GPIO21
#define LCD_SCL_PIN     22  // GPIO22

#define LED_RED_PIN     18  // GPIO18
#define LED_GREEN_PIN   19  // GPIO19
#define LED_YELLOW_PIN  23  // GPIO23


// Air Quality Thresholds (ppm)
#define AQ_GOOD_MAX     250 
#define AQ_MODERATE_MAX 400  

// Air Quality States
#define AQ_GOOD         0
#define AQ_MODERATE     1
#define AQ_BAD          2
#define AQ_ERROR        3

// Task Priorities
#define TASK_SENSOR_PRIORITY    3
#define TASK_PROCESS_PRIORITY   2
#define TASK_DISPLAY_PRIORITY   1
#define TASK_ALERT_PRIORITY     4

// Queue Sizes
#define SENSOR_QUEUE_SIZE   5
#define DISPLAY_QUEUE_SIZE  1
#define ALERT_QUEUE_SIZE    1

#endif
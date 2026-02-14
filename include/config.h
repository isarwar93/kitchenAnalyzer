#ifndef CONFIG_H
#define CONFIG_H

// Pin definitions for ESP32 LOLIN32
#define PIN_GAS_SENSOR      32   // GPIO for gas sensor
#define PIN_TEMP_HUM_SENSOR 13   // GPIO for temp/humidity sensor
#define PIN_MOTION_SENSOR   34   // GPIO for motion sensor
#define PIN_LED             22    // GPIO for status LED

// LCD I2C configuration
#define LCD_I2C_ADDR        0x27  // I2C address for 16x2 LCD
#define LCD_SDA_PIN         17    // I2C SDA pin
#define LCD_SCL_PIN         19    // I2C SCL pin
#define LCD_COLS            16
#define LCD_ROWS            2

// Power management
#define WAKEUP_PIN          PIN_MOTION_SENSOR
#define DEEP_SLEEP_TIME_S   300  // Deep sleep duration in seconds
#define NO_MOTION_TIMEOUT_S 20   // Time to wait after no motion before deep sleep

#endif // CONFIG_H
